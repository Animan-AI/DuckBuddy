# -*- coding: utf-8 -*-
"""Jev judgments for Xiaobai: typed facts at the duck's decision points
(docs/project/duck-life-jev-framework-20261017-CN.md §3).

Jev (TypeSafe's System One) answers atomic questions about a state with probabilities, no free
text. Following the official rules and what AI4Robot learned the hard way: ask facts, never the
next action (lifed's code decides); every choice has an `unclear` way out; each option says what it
is and what it is not; only the context the questions need.

    judge(kind, state, images) -> {"kind", "model", "llm_s", "answers": {name: {...}}}

`kind` is one of JUDGMENTS. Images (base64 JPEG) go in as assets, referenced from the state, with
the danger-zone header the API wants for them; Box questions need jev-1.14.0 or later. The key is
TYPESAFE_API_KEY in the environment (duckbrain's service reads ~/.config/duckbrain/typesafe.env).
Every call is logged (text, answers, timing; images only as size and hash) for offline calibration.
"""
from __future__ import annotations

import base64
import hashlib
import json
import os
import sys
import time

LOG = os.environ.get("DUCKBRAIN_JUDGE_LOG", os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                                         "judge-log.jsonl"))
TEXT_MODEL = os.environ.get("DUCKBRAIN_JEV_MODEL", "jev-latest")
IMAGE_MODEL = os.environ.get("DUCKBRAIN_JEV_IMAGE_MODEL", "jev-1.14.0")   # Box needs 1.14+
TIMEOUT_S = float(os.environ.get("DUCKBRAIN_JEV_TIMEOUT", "3.0"))
IMAGE_HEADER = {"TypeSafe-Danger-Zone-Images": "true"}

# what the duck is, and its name, from its persona (tools/persona); the questions are the framework
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "persona"))
from persona import load as load_persona  # noqa: E402

P = load_persona()
NAME = P.name
DUCK = P.description
CAMERA = (f"`view` 是{NAME}头上摄像头拍的一帧，画面中心是它正前方，水平视场约 62 度，"
          "离地约 20 厘米，所以人通常只拍到腿或者需要仰视。")


class JudgeError(Exception):
    """Why a judgment could not be made: no_api_key, sdk_missing, unknown_kind, bad_image, api, timeout."""


def _opt(what: str, not_for: str = "", examples: list[str] | None = None) -> dict:
    o = {"what": what}
    if not_for:
        o["not_for"] = not_for
    if examples:
        o["examples"] = examples
    return o


# ── J1 utterance (text) ────────────────────────────────────────────────────────────────────────

def _utterance(state: dict) -> tuple[dict, dict]:
    s = {
        "duck": DUCK,
        "text": state.get("text", ""),
        "named": bool(state.get("named")),
        "in_conversation": bool(state.get("in_conversation")),
        "duck_last_said": state.get("duck_last_said") or "",
        "duck_is": state.get("duck_is") or "",
        "note": f"`text` 是语音识别的结果，可能听错字；`named` 表示这句话里叫了它的名字{NAME}。",
    }
    q = {
        "for_duck": {
            "type": "noul",
            "instructions": f"`text` 是不是对{NAME}说的？",
            "criteria": {
                "true": "叫它、问它、让它做事、回应它刚才说的 `duck_last_said`、夸它或说它",
                "false": "对别人说的话、自言自语、电视或视频里的声音、识别出来没头没尾或不成句的乱码",
            },
        },
        "intent": {
            "type": "choice",
            "instructions": f"`text` 想让{NAME}做什么？",
            "criteria": {
                "sleep": _opt("让它睡觉", "只是问它困不困", ["睡觉吧", "该睡了", "晚安", f"去睡吧{NAME}"]),
                "rest": _opt("让它休息、歇一会儿、趴着不动", "问它累不累", ["休息一下", "你去歇着吧", "歇会儿"]),
                "sit": _opt("让它坐下", "", ["坐下", "坐好", "坐一会儿"]),
                "stand": _opt("让它从坐着站起来", "让它醒来（它在睡觉时用 wake）", ["站起来", "起来站好"]),
                "wake": _opt("叫醒正在睡觉或休息的它", "问它醒没醒", ["醒醒", "起床啦", "快起来"]),
                "net": _opt("让它连网、扫二维码", "问它能不能上网", ["连网", "扫码连一下", "连这个热点"]),
                "bye": _opt("道别、结束聊天、主人要离开", "", ["拜拜", "我去忙了", "不聊了", "回头见"]),
                "chat": _opt("聊天、提问、夸它、说它、讲事情", "让它做上面那些动作", ["你在干嘛", "你真可爱", "今天好累"]),
                "unclear": _opt("听不清在说什么，或是识别出来的乱码", "", ["申请吧", "あ好好", "嗯"]),
            },
        },
        "reply": {
            "type": "choice",
            "instructions": f"{NAME}需要怎样回应 `text`？",
            "criteria": {
                "none": _opt("不需要回应", "", ["嗯", "哦", "好的", "自言自语"]),
                "simple": _opt("一句固定的话就能回：打招呼、夸奖、简单问候、谢谢、道歉",
                               "需要想一想才能回的提问或聊天", ["你好呀", "真乖", "谢谢你", "早上好"]),
                "think": _opt("需要动脑回应：提问、聊天、讲事情、问它的感受", "",
                              ["你今天做了什么", "你喜欢下雨吗", "我今天好累啊"]),
            },
        },
    }
    return s, q


# ── J2 person (image) ──────────────────────────────────────────────────────────────────────────

def _person(state: dict) -> tuple[dict, dict]:
    s = {"duck": DUCK, "camera": CAMERA, "view": state["view"], "tof_distance_m": state.get("tof_distance_m")}
    q = {
        "person": {
            "type": "noul",
            "instructions": "`view` 里有没有真人？",
            "criteria": {"true": "真人，哪怕只拍到腿、手或半张脸",
                         "false": "没有人；或者只是照片、海报、玩偶、电视屏幕里的人"},
        },
        "activity": {
            "type": "choice",
            "instructions": f"`view` 里离{NAME}最近的那个人在做什么？",
            "criteria": {
                "looking_at_duck": _opt(f"看着{NAME}，或者朝它走过来", "背对着它或在看别处"),
                "waving": _opt("朝它挥手、招手"),
                "thumbs_up": _opt("朝它竖大拇指"),
                "reaching": _opt("伸手要摸它或抱它"),
                "passing": _opt("从旁边走过，没在看它"),
                "busy": _opt("在做别的事（看手机、干活），没注意它"),
                "unclear": _opt("看不清，或者画面里没有人"),
            },
        },
        "facing": {
            "type": "noul",
            "instructions": f"离{NAME}最近的那个人的脸或身体是不是朝着摄像头？",
        },
        "face": {
            "type": "box",
            "instructions": "`view` 里离摄像头最近的那个人的脸；看不到脸就框住这个人",
        },
    }
    return s, q


# ── J3 floor, J4 presence (image) ──────────────────────────────────────────────────────────────

def _floor(state: dict) -> tuple[dict, dict]:
    s = {"duck": DUCK, "camera": CAMERA, "view": state["view"], "tof_ahead_m": state.get("tof_ahead_m")}
    q = {
        "floor": {
            "type": "choice",
            "instructions": f"{NAME}脚下和正前方是什么地面？",
            "criteria": {
                "carpet": _opt("地毯、绒面地垫"),
                "smooth_hard": _opt("瓷砖、大理石、抛光水泥等光滑硬地"),
                "wood": _opt("木地板"),
                "mat_edge": _opt("地垫或地毯的边缘，前方地面会变"),
                "outdoor": _opt("户外地面：路面、草地、砖地"),
                "unclear": _opt("看不清地面"),
            },
        },
        "path_clear": {
            "type": "noul",
            "instructions": f"{NAME}正前方大约 1 米的地面上能走过去吗？",
            "criteria": {"true": "平整、没有障碍", "false": "有障碍物、台阶、桌子或沙发边、电线、地面高低不平"},
        },
    }
    return s, q


def _presence(state: dict) -> tuple[dict, dict]:
    s = {"camera": CAMERA, "view": state["view"]}
    q = {"anyone": {"type": "noul", "instructions": "`view` 里有没有人？",
                    "criteria": {"true": "有真人，哪怕只看到一部分", "false": "没有人，或只有照片、电视里的人"}}}
    return s, q


JUDGMENTS = {"utterance": (_utterance, False), "person": (_person, True),
             "floor": (_floor, True), "presence": (_presence, True)}


# ── Request assembly ───────────────────────────────────────────────────────────────────────────

def build(kind: str, state: dict, images: dict[str, str] | None = None):
    """The request for a judgment: (state, questions, assets, headers, model). Each image (base64
    JPEG) becomes an asset referenced from the state by a unique token in place of its name."""
    if kind not in JUDGMENTS:
        raise JudgeError("unknown_kind")
    make, needs_images = JUDGMENTS[kind]
    images = images or {}
    assets, state = {}, dict(state)
    if needs_images:
        if "view" not in images:
            raise JudgeError("bad_image")
        for name, b64 in images.items():
            try:
                raw = base64.b64decode(b64, validate=True)
            except ValueError as e:
                raise JudgeError("bad_image") from e
            if raw[:2] != b"\xff\xd8":
                raise JudgeError("bad_image")
            ref = f"img-{name}-{hashlib.sha1(raw).hexdigest()[:10]}"
            assets[ref] = {"type": "image", "format": "jpg", "data_base64": b64}
            state[name] = ref
    s, q = make(state)
    headers = IMAGE_HEADER if assets else {}
    model = IMAGE_MODEL if assets else TEXT_MODEL
    return s, q, assets, headers, model


_client = None


def _get_client():
    global _client
    if not os.environ.get("TYPESAFE_API_KEY"):
        raise JudgeError("no_api_key")
    if _client is None:
        try:
            from typesafe_sdk import TypeSafeClient
        except ImportError as e:
            raise JudgeError("sdk_missing") from e
        _client = TypeSafeClient()
    return _client


def _plain(answer) -> dict:
    if hasattr(answer, "model_dump"):
        answer = answer.model_dump(mode="json")
    return {k: v for k, v in dict(answer).items() if k != "legend"}


def _log(entry: dict):
    try:
        with open(LOG, "a", encoding="utf-8") as f:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")
    except OSError:
        pass


def judge(kind: str, state: dict, images: dict[str, str] | None = None) -> dict:
    """Ask Jev; JudgeError if it cannot be asked or does not answer."""
    s, q, assets, headers, model = build(kind, state, images)
    client = _get_client()
    t0 = time.time()
    try:
        r = client.system_one(s, q, model=model, timeout=TIMEOUT_S, extra_headers=headers or None,
                              extra_body={"assets": assets} if assets else None)
    except Exception as e:  # noqa: BLE001 - the SDK's errors, a timeout, the network
        name = type(e).__name__
        _log({"t": t0, "kind": kind, "error": name, "detail": str(e)[:200]})
        raise JudgeError("timeout" if "Timeout" in name or "Connection" in name else "api") from e
    out = {"kind": kind, "model": getattr(r, "model", model), "llm_s": round(time.time() - t0, 3),
           "answers": {name: _plain(a) for name, a in (getattr(r, "answers", {}) or {}).items()}}
    _log({"t": t0, "kind": kind, "state": {k: v for k, v in s.items() if k not in ("duck", "camera", "note")},
          "images": {n: {"bytes": len(a["data_base64"]) * 3 // 4} for n, a in assets.items()},
          "answers": out["answers"], "llm_s": out["llm_s"], "model": out["model"]})
    return out


# ── duckjev: this module as its own small service ──────────────────────────────────────────────
# typesafe-sdk needs pydantic >= 2.12 and the brain's venv has 2.7 (FastAPI, CosyVoice), so Jev runs
# in its own venv (/data/microduck/tts/jev-venv) as `duckjev` on 127.0.0.1:8211, and duckbrain's
# /judge forwards to it. Standard library only, besides the SDK.

def serve(port: int = 8211):
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, obj, code=200):
            body = json.dumps(obj, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/health":
                try:
                    import typesafe_sdk  # noqa: F401
                    sdk = True
                except ImportError:
                    sdk = False
                self._send({"ok": True, "key": bool(os.environ.get("TYPESAFE_API_KEY")), "sdk": sdk})
            else:
                self._send({"ok": False, "error": "not here"}, 404)

        def do_POST(self):
            if self.path != "/judge":
                return self._send({"ok": False, "error": "not here"}, 404)
            try:
                req = json.loads(self.rfile.read(int(self.headers.get("Content-Length") or 0)) or b"{}")
                out = judge(req.get("kind", ""), req.get("state") or {}, req.get("images") or None)
                self._send({"ok": True, **out})
            except JudgeError as e:
                self._send({"ok": False, "error": str(e)})
            except ValueError:
                self._send({"ok": False, "error": "bad_request"}, 400)

    srv = ThreadingHTTPServer(("127.0.0.1", port), H)
    print(time.strftime("%Y-%m-%d %H:%M:%S"), f"duckjev on 127.0.0.1:{port}, key {'set' if os.environ.get('TYPESAFE_API_KEY') else 'missing'}",
          flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    import sys
    serve(int(sys.argv[1]) if len(sys.argv) > 1 else 8211)
