#!/bin/bash
# 2026-10-15: when queue_v16 (v11 walks + LieDown v3) finishes on the server, export their
# checkpoints to ONNX (CPU), copy them here and score them in the robot-matched BAM simulation:
#   walks  - 0.3 m/s on floor friction 0.5 / 1.0 / 2.0 and standing still, against v9 wf-sym-drift-3750
#            (on the robot now) and v7 wf-sym-2500, with the head yaw swing column;
#   LieDown - lie_eval_hl2915.py on friction 1.0 / 2.0.
set -u
cd <repo>
OUT=training/data/sim-hl2915/v11-lie3-20261015
mkdir -p "$OUT" training/policies/sweep/v11 training/policies/sweep/lie3
L=/data/microduck/bam-hl2915
R=/data/microduck/microduck_rl
export PYTHONPATH=tools/pydeps PYTHONIOENCODING=utf-8

walks() {
  until ssh -o BatchMode=yes v100 "grep -q 'queue_v16 done' $L/queue_v16.out"; do sleep 300; done
  echo "walks trained $(date)" >> "$OUT/pipeline.log"
  ssh -o BatchMode=yes v100 "cd $R && for d in logs/rsl_rl/velocity/*hl2915-walk-v11-*; do
      n=\${d##*hl2915-walk-v11-}; o=$L/sweep_v11/\$n; mkdir -p \$o
      CUDA_VISIBLE_DEVICES= .venv/bin/python $L/ckpt_onnx.py --template \$d/\$(basename \$d).onnx --out-dir \$o \
          \$d/model_2500.pt \$d/model_3000.pt \$d/model_3500.pt \$d/model_3750.pt \$d/model_3999.pt
    done" >> "$OUT/pipeline.log" 2>&1
  scp -q -r v100:$L/sweep_v11/* training/policies/sweep/v11/ >> "$OUT/pipeline.log" 2>&1
  POLS=""
  for f in training/policies/sweep/v11/*/*.onnx; do r=$(basename $(dirname $f)); POLS="$POLS $r-$(basename $f .onnx)=$f"; done
  for p in $POLS; do
    label=${p%%=*}; path=${p#*=}
    python training/bam/walk_zero_ab_hl2915.py "$path@0.9" --zeros new --friction 0.5,1.0,2.0 --vx 0.3 --workers 12 \
        --out "$OUT/walk_$label.md" >> "$OUT/pipeline.log" 2>&1
    python training/bam/walk_zero_ab_hl2915.py "$path@0.9" --zeros new --friction 1.0 --vx 0.0 --workers 12 \
        --out "$OUT/stand_$label.md" >> "$OUT/pipeline.log" 2>&1
    echo "walk $label done $(date)" >> "$OUT/pipeline.log"
  done
}

lie() {
  until ssh -o BatchMode=yes v100 "grep -q 'queue_v16 done' $L/queue_v16.out"; do sleep 300; done
  echo "LieDown trained $(date)" >> "$OUT/pipeline.log"
  ssh -o BatchMode=yes v100 "cd $R && d=\$(ls -d logs/rsl_rl/microduck_sitstand/*hl2915-liedown-v3 | tail -1); o=$L/sweep_lie3; mkdir -p \$o
      CUDA_VISIBLE_DEVICES= .venv/bin/python $L/ckpt_onnx.py --template \$d/\$(basename \$d).onnx --out-dir \$o \
          \$d/model_2000.pt \$d/model_2500.pt \$d/model_3000.pt \$d/model_3500.pt \$d/model_3999.pt" >> "$OUT/pipeline.log" 2>&1
  scp -q v100:$L/sweep_lie3/*.onnx training/policies/sweep/lie3/ >> "$OUT/pipeline.log" 2>&1
  for f in training/policies/sweep/lie3/*.onnx; do
    n=$(basename $f .onnx)
    python training/bam/lie_eval_hl2915.py "$f" --friction 1.0,2.0 --workers 12 --out "$OUT/lie_$n.md" >> "$OUT/pipeline.log" 2>&1
    echo "lie $n done $(date)" >> "$OUT/pipeline.log"
  done
}

# whichever finishes first is scored first; the sims share the CPU, so one after the other
if ssh -o BatchMode=yes v100 "grep -q 'queue_v16 done' $L/queue_v16.out"; then lie; walks; else walks; lie; fi
echo "pipeline done $(date)" >> "$OUT/pipeline.log"
