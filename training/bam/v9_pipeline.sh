#!/bin/bash
# 2026-10-13: when queue_v12 (v9 walks) finishes on the server, export checkpoints 2500-3999 of each
# run to ONNX (CPU), copy them here and score them in the robot-matched BAM simulation (new zeros):
# 0.3 m/s on floor friction 0.5 / 1.0 / 2.0, and standing still (vx 0) on 1.0, against slip3999 and wf3750.
set -u
cd <repo>
OUT=training/data/sim-hl2915/v9-sweep-20261013
mkdir -p "$OUT" training/policies/sweep/v9
until ssh -o BatchMode=yes v100 'grep -q "queue_v12 done" /data/microduck/bam-hl2915/queue_v12.out'; do sleep 300; done
echo "training done $(date)" > "$OUT/pipeline.log"
ssh -o BatchMode=yes v100 'cd /data/microduck/microduck_rl && for d in logs/rsl_rl/velocity/*hl2915-walk-v9-*; do
    n=${d##*hl2915-walk-v9-}; o=/data/microduck/bam-hl2915/sweep_v9/$n; mkdir -p $o
    CUDA_VISIBLE_DEVICES= .venv/bin/python /data/microduck/bam-hl2915/ckpt_onnx.py --template $d/$(basename $d).onnx \
        --out-dir $o $d/model_2500.pt $d/model_2750.pt $d/model_3000.pt $d/model_3250.pt $d/model_3500.pt $d/model_3750.pt $d/model_3999.pt
  done' >> "$OUT/pipeline.log" 2>&1
scp -q -r v100:/data/microduck/bam-hl2915/sweep_v9/* training/policies/sweep/v9/ >> "$OUT/pipeline.log" 2>&1
export PYTHONPATH=tools/pydeps
POLS="slip3999=training/policies/hl2915_walk_v6lp_slip_3999.onnx wf-sym-2500=training/policies/sweep/v7/wf-sym/2500.onnx"
for f in training/policies/sweep/v9/*/*.onnx; do r=$(basename $(dirname $f)); POLS="$POLS $r-$(basename $f .onnx)=$f"; done
for p in $POLS; do
  label=${p%%=*}; path=${p#*=}
  python training/bam/walk_zero_ab_hl2915.py "$path@0.9" --zeros new --friction 0.5,1.0,2.0 --vx 0.3 --workers 12 \
      --out "$OUT/walk_$label.md" >> "$OUT/pipeline.log" 2>&1
  python training/bam/walk_zero_ab_hl2915.py "$path@0.9" --zeros new --friction 1.0 --vx 0.0 --workers 12 \
      --out "$OUT/stand_$label.md" >> "$OUT/pipeline.log" 2>&1
  echo "$label done $(date)" >> "$OUT/pipeline.log"
done
echo "pipeline done $(date)" >> "$OUT/pipeline.log"
