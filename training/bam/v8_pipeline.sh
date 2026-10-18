#!/bin/bash
# 2026-10-13: when queue_v11 (v8 VelStand) finishes, export checkpoints 3500-5999 of both runs, then in the
# robot-matched BAM simulation: standing still + duck-mcp head gestures (stand_head_eval) against the stand-up
# network at 0.6 and wf-sym-2500, and walking 0.3 m/s on friction 0.5 / 1.0 / 2.0 (walk_zero_ab).
set -u
cd <repo>
OUT=training/data/sim-hl2915/v8-velstand-20261013
mkdir -p "$OUT" training/policies/sweep/v8
until ssh -o BatchMode=yes v100 'grep -q "queue_v11 done" /data/microduck/bam-hl2915/queue_v11.out'; do sleep 300; done
echo "training done $(date)" > "$OUT/pipeline.log"
ssh -o BatchMode=yes v100 'cd /data/microduck/microduck_rl && for d in logs/rsl_rl/velstand/*hl2915-velstand-v8-*; do
    n=${d##*hl2915-velstand-v8-}; o=/data/microduck/bam-hl2915/sweep_v8/$n; mkdir -p $o
    CUDA_VISIBLE_DEVICES= .venv/bin/python /data/microduck/bam-hl2915/ckpt_onnx.py --template $d/$(basename $d).onnx \
        --out-dir $o $d/model_3500.pt $d/model_4000.pt $d/model_4500.pt $d/model_5000.pt $d/model_5500.pt $d/model_5999.pt
  done' >> "$OUT/pipeline.log" 2>&1
scp -q -r v100:/data/microduck/bam-hl2915/sweep_v8/* training/policies/sweep/v8/ >> "$OUT/pipeline.log" 2>&1
export PYTHONPATH=tools/pydeps
POLS="standup06=training/policies/hl2915_standup_v3bl_5999.onnx@0.6#13 wf-sym-2500=training/policies/sweep/v7/wf-sym/2500.onnx@0.9"
for f in training/policies/sweep/v8/*/*.onnx; do r=$(basename $(dirname $f)); POLS="$POLS $r-$(basename $f .onnx)=$f@0.9"; done
python training/bam/stand_head_eval_hl2915.py $POLS --out "$OUT/stand_head.md" >> "$OUT/pipeline.log" 2>&1
echo "stand/head done $(date)" >> "$OUT/pipeline.log"
for f in training/policies/sweep/v8/*/*.onnx; do
  label=$(basename $(dirname $f))-$(basename $f .onnx)
  python training/bam/walk_zero_ab_hl2915.py "$f@0.9" --zeros new --friction 0.5,1.0,2.0 --vx 0.3 --workers 12 \
      --out "$OUT/walk_$label.md" >> "$OUT/pipeline.log" 2>&1
  echo "$label walk done $(date)" >> "$OUT/pipeline.log"
done
echo "pipeline done $(date)" >> "$OUT/pipeline.log"
