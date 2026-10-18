#!/bin/bash
# 2026-10-15: when queue_v17 (LieDown v4, soft landing charged late) finishes on the server, export
# its checkpoints to ONNX (CPU), copy them here and score them with lie_eval_hl2915.py on floor
# friction 1.0 / 2.0 (the rise on the stand-up network, as robotd does it).
set -u
cd <repo>
OUT=training/data/sim-hl2915/lie4-20261015
mkdir -p "$OUT" training/policies/sweep/lie4
L=/data/microduck/bam-hl2915
R=/data/microduck/microduck_rl
export PYTHONPATH=tools/pydeps PYTHONIOENCODING=utf-8
until ssh -o BatchMode=yes v100 "grep -q 'queue_v17 done' $L/queue_v17.out"; do sleep 300; done
echo "LieDown v4 trained $(date)" >> "$OUT/pipeline.log"
ssh -o BatchMode=yes v100 "cd $R && d=\$(ls -d logs/rsl_rl/microduck_sitstand/*hl2915-liedown-v4 | tail -1); o=$L/sweep_lie4; mkdir -p \$o
    CUDA_VISIBLE_DEVICES= .venv/bin/python $L/ckpt_onnx.py --template \$d/\$(basename \$d).onnx --out-dir \$o \
        \$d/model_1000.pt \$d/model_1500.pt \$d/model_2000.pt \$d/model_2500.pt \$d/model_3000.pt \
        \$d/model_3500.pt \$d/model_4000.pt \$d/model_4500.pt \$d/model_4999.pt" >> "$OUT/pipeline.log" 2>&1
scp -q v100:$L/sweep_lie4/*.onnx training/policies/sweep/lie4/ >> "$OUT/pipeline.log" 2>&1
for n in 1000 1500 2000 2500 3000 3500 4000 4500 4999; do
  python training/bam/lie_eval_hl2915.py training/policies/sweep/lie4/$n.onnx --friction 1.0,2.0 --workers 12 \
      --out "$OUT/lie_$n.md" >> "$OUT/pipeline.log" 2>&1
  echo "lie $n done $(date)" >> "$OUT/pipeline.log"
done
echo "pipeline done $(date)" >> "$OUT/pipeline.log"
