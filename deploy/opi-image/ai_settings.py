"""Settings shared by duckweb, lifed and the r6 text gateway."""

import json
import os
import tempfile
import urllib.request
from pathlib import Path
from urllib.parse import urlsplit

CONFIG = Path.home() / ".config/duck"
PRESETS = {
    "dashscope": ("https://dashscope.aliyuncs.com/compatible-mode/v1", "qwen3.8-flash"),
    "ark": ("https://ark.cn-beijing.volces.com/api/v3", ""),
    "zhipu": ("https://open.bigmodel.cn/api/paas/v4", "glm-4.6v-flash"),
    "deepseek": ("https://api.deepseek.com/v1", "deepseek-flash"),
    "openrouter": ("https://openrouter.ai/api/v1", "openrouter/free"),
}
TTS_PRESETS = {
    "zhipu": ("https://open.bigmodel.cn/api/paas/v4", "glm-tts", "female"),
    "siliconflow": ("https://api.siliconflow.cn/v1", "FunAudioLLM/CosyVoice2-0.5B",
                    "FunAudioLLM/CosyVoice2-0.5B:anna"),
}


class KeepEndpoint(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        # An API key belongs to the configured endpoint.
        return None


HTTP = urllib.request.build_opener(KeepEndpoint())


def read_saved():
    try:
        return json.loads((CONFIG / "ai-settings.json").read_text(encoding="utf-8"))
    except FileNotFoundError:
        return {}


def read(saved=None):
    if saved is None:
        saved = read_saved()
    provider = os.environ.get("DUCK_BRAIN_PROVIDER", "openrouter")
    base, model = PRESETS.get(provider, PRESETS["openrouter"])
    return {
        "voice_mode": saved.get("voice_mode", "human"),
        "model": saved.get("model", {"provider": provider, "base_url": base, "model": model}),
        "tts": saved.get("tts", {"provider": "edge", "voice": "zh-CN-XiaoxiaoNeural",
                                 "base_url": "", "model": ""}),
    }


def api_key(settings):
    if "api_key" in settings:
        return settings["api_key"]
    provider = settings["provider"]
    preset = PRESETS.get(provider) or TTS_PRESETS.get(provider)
    if preset and settings["base_url"] == preset[0]:
        try:
            name = "bigmodel" if provider == "zhipu" else provider
            return (CONFIG / f"{name}.key").read_text(encoding="ascii").strip()
        except FileNotFoundError:
            pass
    return ""


def public():
    state = read()
    for name in ("model", "tts"):
        value = state[name]
        value["has_key"] = bool(api_key(value))
        value.pop("api_key", None)
    return state


def endpoint(value):
    value = value.strip().rstrip("/")
    address = urlsplit(value)
    if (address.scheme not in ("http", "https") or not address.hostname or
            address.username or address.password or address.query or address.fragment or len(value) > 400):
        raise ValueError("请填写有效的 API Base URL，例如 https://example.com/v1")
    return value


def field(body, name, required=True):
    value = body.get(name, "")
    if not isinstance(value, str) or len(value) > 512 or any(ord(c) < 32 for c in value):
        raise ValueError("配置字段格式不正确")
    value = value.strip()
    if required and not value:
        raise ValueError("请填写模型、音色和所需的接口地址")
    return value


def updated(name, body, old):
    provider = field(body, "provider")
    if name == "model":
        if provider in PRESETS:
            base, default = PRESETS[provider]
            model = field(body, "model", required=False) or default
            if not model:
                raise ValueError("请填写方舟控制台中的模型 ID 或推理接入点 ID")
        elif provider == "compatible":
            base, model = endpoint(field(body, "base_url")), field(body, "model")
        else:
            raise ValueError("请选择模型服务")
        value = {"provider": provider, "base_url": base, "model": model}
    elif provider == "edge":
        voice = field(body, "voice")
        return {"provider": provider, "base_url": "", "model": "", "voice": voice}
    elif provider in TTS_PRESETS:
        base, default_model, default_voice = TTS_PRESETS[provider]
        value = {"provider": provider, "base_url": base,
                 "model": field(body, "model", required=False) or default_model,
                 "voice": field(body, "voice", required=False) or default_voice}
    elif provider == "compatible":
        value = {"provider": provider, "base_url": endpoint(field(body, "base_url")),
                 "model": field(body, "model"), "voice": field(body, "voice")}
    else:
        raise ValueError("请选择语音合成服务")
    key = field(body, "api_key", required=False)
    same_endpoint = provider == old["provider"] and value["base_url"] == old["base_url"]
    if body.get("clear_key"):
        value["api_key"] = ""
    elif key:
        value["api_key"] = key
    elif same_endpoint and "api_key" in old:
        value["api_key"] = old["api_key"]
    return value


def save(changes):
    import fcntl

    CONFIG.mkdir(mode=0o700, parents=True, exist_ok=True)
    with (CONFIG / "ai-settings.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        saved = read_saved()
        state = read(saved)
        credentials = saved.get("credentials", {})
        if "voice_mode" in changes:
            mode = changes["voice_mode"]
            if mode not in ("human", "duck"):
                raise ValueError("请选择人声或鸭语")
            state["voice_mode"] = mode
        for name in ("model", "tts"):
            if name in changes:
                if not isinstance(changes[name], dict):
                    raise ValueError("配置格式不正确")
                old = state[name]
                value = updated(name, changes[name], old)
                keys = credentials.setdefault(name, {})
                if "api_key" in old and old["base_url"]:
                    keys.setdefault(old["provider"], {})[old["base_url"]] = old["api_key"]
                base = value["base_url"]
                if base:
                    by_endpoint = keys.setdefault(value["provider"], {})
                    if "api_key" in value:
                        by_endpoint[base] = value["api_key"]
                    elif base in by_endpoint:
                        value["api_key"] = by_endpoint[base]
                state[name] = value
        if credentials:
            state["credentials"] = credentials
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=CONFIG, delete=False) as out:
            json.dump(state, out, ensure_ascii=False)
            out.flush()
            os.fsync(out.fileno())
        try:
            os.replace(out.name, CONFIG / "ai-settings.json")
        finally:
            Path(out.name).unlink(missing_ok=True)
