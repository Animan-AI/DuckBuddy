# -*- coding: utf-8 -*-
"""Export rsl_rl checkpoints to ONNX on the CPU, by swapping their actor weights into an ONNX
that `scripts/export.py` already wrote for the same task (same graph, normalizer baked in).

`export.py` builds the whole environment (MuJoCo Warp, so the GPU) just to trace a fixed MLP.
For a checkpoint sweep that means one GPU job per checkpoint; this needs none. Each export is
checked: the template's own checkpoint must reproduce the template bit for bit, and every
output is compared against a plain numpy forward pass of the checkpoint.

    # on the server, from microduck_rl (CPU only)
    CUDA_VISIBLE_DEVICES= .venv/bin/python ckpt_onnx.py --template RUN/RUN.onnx \
        --out-dir sweep/ RUN/model_2000.pt RUN/model_2250.pt ...
"""
import argparse
import os

import numpy as np
import onnx
import torch
from onnx import numpy_helper

LAYERS = ("mlp.0", "mlp.2", "mlp.4", "mlp.6")


def actor(path):
    ck = torch.load(path, map_location="cpu", weights_only=False)
    return {k: v.numpy().astype(np.float32) for k, v in ck["actor_state_dict"].items() if hasattr(v, "numpy")}


def normalizer_names(model):
    """The graph is obs -> Sub(mean) -> Div(std) -> MLP; find the Div's initializer by tracing."""
    inits = {i.name for i in model.graph.initializer}
    sub = next(n for n in model.graph.node if n.op_type == "Sub")
    div = next(n for n in model.graph.node if n.op_type == "Div" and sub.output[0] in n.input)
    mean = next(x for x in sub.input if x in inits)
    std = next(x for x in div.input if x in inits)
    return mean, std


def numpy_forward(w, obs, eps):
    x = (obs - w["obs_normalizer._mean"]) / (w["obs_normalizer._std"] + eps)
    for i, layer in enumerate(LAYERS):
        x = x @ w[layer + ".weight"].T + w[layer + ".bias"]
        if i < len(LAYERS) - 1:
            x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))  # ELU
    return x


def normalizer_eps(template, weights):
    """The exported divisor is std + eps (rsl_rl's EmpiricalNormalization); read eps off the template."""
    model = onnx.load(template)
    _, std = normalizer_names(model)
    div = next(numpy_helper.to_array(i) for i in model.graph.initializer if i.name == std)
    eps = div - weights["obs_normalizer._std"]
    if np.ptp(eps) > 1e-5:
        raise SystemExit("template divisor is not checkpoint std + a constant — wrong template checkpoint?")
    return float(eps.mean())


def export(template, weights, out, eps):
    model = onnx.load(template)
    mean, std = normalizer_names(model)
    source = {mean: weights["obs_normalizer._mean"], std: weights["obs_normalizer._std"] + eps}
    for layer in LAYERS:
        for p in ("weight", "bias"):
            source[f"{layer}.{p}"] = weights[f"{layer}.{p}"]
    for init in model.graph.initializer:
        new = source[init.name].reshape(tuple(init.dims))
        init.CopyFrom(numpy_helper.from_array(new.astype(np.float32), init.name))
    onnx.save(model, out)


def run(path, obs):
    import onnxruntime as ort
    s = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
    return np.concatenate([s.run(None, {s.get_inputs()[0].name: o[None]})[0] for o in obs])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--template", required=True, help="an export.py ONNX of the same task")
    ap.add_argument("--template-ckpt", help="the checkpoint the template came from (default: newest model_*.pt next to it)")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--prefix", default="", help="output name prefix, e.g. hl2915_walk_v4bl_")
    ap.add_argument("checkpoints", nargs="+")
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    obs = np.random.default_rng(0).normal(size=(256, 61)).astype(np.float32)

    ref = a.template_ckpt or max((os.path.join(os.path.dirname(a.template), f) for f in os.listdir(os.path.dirname(a.template))
                                  if f.startswith("model_") and f.endswith(".pt")),
                                 key=lambda p: int(os.path.basename(p)[6:-3]))
    check = os.path.join(a.out_dir, "_template_check.onnx")
    eps = normalizer_eps(a.template, actor(ref))
    export(a.template, actor(ref), check, eps)
    diff = np.abs(run(check, obs) - run(a.template, obs)).max()
    os.remove(check)
    print(f"template check ({os.path.basename(ref)}, normalizer eps {eps:.4f}): max |diff| {diff:.2e}")
    if diff > 1e-4:  # float32 rounding of std + eps is ~5e-6
        raise SystemExit("the template's own checkpoint does not reproduce it — wrong template or mapping")

    for ck in a.checkpoints:
        it = int(os.path.basename(ck)[6:-3])
        out = os.path.join(a.out_dir, f"{a.prefix}{it}.onnx")
        w = actor(ck)
        export(a.template, w, out, eps)
        err = np.abs(run(out, obs) - numpy_forward(w, obs, eps)).max()
        print(f"{os.path.basename(ck)} -> {out}  (vs numpy forward {err:.1e})")
        if err > 1e-4:
            raise SystemExit("export does not match the checkpoint")


if __name__ == "__main__":
    main()
