#!/bin/bash
# 2026-10-15: a CosyVoice3 env for the duck voice (the old ~/cosyvoice_env lost its bin/ and parts of its packages).
export PATH=$HOME/.local/bin:$PATH UV_CACHE_DIR=/data/microduck/.uv-cache UV_PYTHON_INSTALL_DIR=/data/microduck/.uv-python
cd /data/microduck/tts
[ -x venv/bin/python ] || uv venv --python 3.10 venv
uv pip install --python venv/bin/python setuptools==69.5.1 wheel
uv pip install --python venv/bin/python --index-strategy unsafe-best-match --no-build-isolation-package openai-whisper \
  --extra-index-url https://download.pytorch.org/whl/cu121 \
  torch==2.3.1 torchaudio==2.3.1 conformer==0.3.2 diffusers==0.29.0 hydra-core==1.3.2 HyperPyYAML==1.2.2 \
  inflect==7.3.1 librosa==0.10.2 lightning==2.2.4 matplotlib==3.7.5 modelscope==1.20.0 networkx==3.1 numpy==1.26.4 \
  omegaconf==2.3.0 onnx==1.16.0 onnxruntime-gpu==1.18.0 openai-whisper==20231117 protobuf==4.25 pyarrow==18.1.0 \
  pydantic==2.7.0 pyworld==0.3.4 rich==13.7.1 soundfile==0.12.1 transformers==4.51.3 x-transformers==2.11.24 \
  wetext==0.0.4 wget==3.2 ruamel.yaml==0.17.40 gdown==5.1.0 fastapi==0.115.6 uvicorn==0.30.0
echo "venv done $?"
