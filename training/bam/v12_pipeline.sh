#!/bin/bash
# 2026-10-17 night: when queue_v18 (v12 walks: v9 fine-tuned at 920 g, with and without the head-swing
# band) finishes on the server, export the last checkpoints to ONNX (CPU), copy them here and score them
# in the robot-matched BAM simulation like v11 (training/data/sim-hl2915/v11-lie3-20261015): 0.3 m/s on
# floor friction 0.5 / 1.0 / 2.0 and standing still, with the head yaw swing column. v9 wf-sym-drift-3750
# (on the robot) and v11 calm-3999 were scored the same way then; v9 is rescored here as the control.
# Note: the BAM model is the 850 g one - the robot decides what the extra 70 g does.
set -u
cd <repo>
OUT=training/data/sim-hl2915/v12-20261017
mkdir -p "$OUT" training/policies/sweep/v12
L=/data/microduck/bam-hl2915
R=/data/microduck/microduck_rl
export PYTHONPATH=tools/pydeps PYTHONIOENCODING=utf-8

until ssh -o BatchMode=yes v100 "grep -q 'queue_v18 done' $L/queue_v18.out"; do sleep 300; done
echo "v12 trained $(date)" >> "$OUT/pipeline.log"
ssh -o BatchMode=yes v100 "cd $R && for d in logs/rsl_rl/velocity/*hl2915-walk-v12-*; do
    n=\${d##*hl2915-walk-v12-}; o=$L/sweep_v12/\$n; mkdir -p \$o
    ck=\$(ls \$d/model_*.pt | sort -V | tail -4)
    CUDA_VISIBLE_DEVICES= .venv/bin/python $L/ckpt_onnx.py --template \$d/\$(basename \$d).onnx --out-dir \$o \$ck
  done" >> "$OUT/pipeline.log" 2>&1
scp -q -r v100:$L/sweep_v12/* training/policies/sweep/v12/ >> "$OUT/pipeline.log" 2>&1

POLS="v9-3750=training/policies/sweep/v9/wf-sym-drift/3750.onnx"
for f in training/policies/sweep/v12/*/*.onnx; do r=$(basename $(dirname $f)); POLS="$POLS $r-$(basename $f .onnx)=$f"; done
for p in $POLS; do
  label=${p%%=*}; path=${p#*=}
  python training/bam/walk_zero_ab_hl2915.py "$path@0.9" --zeros new --friction 0.5,1.0,2.0 --vx 0.3 --workers 12 \
      --out "$OUT/walk_$label.md" >> "$OUT/pipeline.log" 2>&1
  python training/bam/walk_zero_ab_hl2915.py "$path@0.9" --zeros new --friction 1.0 --vx 0.0 --workers 12 \
      --out "$OUT/stand_$label.md" >> "$OUT/pipeline.log" 2>&1
  echo "walk $label done $(date)" >> "$OUT/pipeline.log"
done
echo "pipeline done $(date)" >> "$OUT/pipeline.log"
