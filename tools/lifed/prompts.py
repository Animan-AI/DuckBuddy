# -*- coding: utf-8 -*-
"""What the duck says in a person's words, where duck-speak would leave you guessing.

Everyday talk stays duck-speak. A few moments need instructions a person can follow, like
connecting it to a phone's hotspot outdoors: for those it plays a clip in a human voice
(edge-tts Xiaoxiao, the voice the user picked on 2026-10-09). The clips are recorded ahead of
time with `render_prompts.py` and kept on the robot (~/lifed/voice/<key>.wav), so they work
with no network. A missing clip falls back to the duck-speak line.

Pure data: core.py estimates from the text how long the duck is deaf to itself.
"""

PROMPTS = {
    # QR Wi-Fi provisioning (tools/visiond)
    "scan_start": "请把 Wi-Fi 或手机热点的二维码，放在我脸前面二三十厘米，保持一两秒。不想连了，就说取消。",
    "joining": "看到二维码了，正在连接，请稍等。",
    "joined": "网络连好啦，谢谢你！",
    "already": "我已经连着这个网络啦。",
    "timeout": "没有看到二维码，我先不扫了。想再试，就叫我连网。",
    "not_wifi": "这不是连网用的二维码，我先不扫了。请换成 Wi-Fi 或手机热点的分享二维码，再叫我连网。",
    "not_found": "没找到这个热点，我先不连了。苹果手机请打开最大兼容性，先停在个人热点页面，"
                 "叫我连网，等几秒，再切到二维码给我看。",
    "failed": "没有连上，我先不连了。可能是密码不对，检查一下再叫我连网。",
    "cancelled": "好的，不连了。",
    # its phone page's address, read out (on a phone's hotspot you cannot know it): the intro, the
    # address digit by digit (DIGITS), the port
    "addr_intro": "我的网页地址是：",
    "addr_port": "冒号，八零八八。在手机浏览器里打开就行。",
    "addr_none": "我现在还没有连上网络。",
    # look: what it sees is told by the brain on the V100; offline it cannot
    "look_offline": "我现在没联网，看不懂这个。",
    "look_failed": "我的眼睛现在看不了，等一下再试试。",
    # missions (mission.py): a spoken route, read back, done step by step
    "mission_ready": "我准备好了，说开始我就出发。",
    "mission_ready_dry": "我准备好了。这次是演练，我不会真的动，说开始就出发。",
    "mission_done": "完成啦！",
    "mission_failed": "这一步我没做到，先停下了。",
    "mission_paused": "好，我停下了。说继续，我就接着走。",
    "mission_obstacle": "前面有东西，我先停下了。",
    "mission_cancelled": "好的，不走了。",
    "ask_turn_or_side": "是转过去再走，还是横着走呀？",
    "ask_how_far": "要走多远呀？",
    "no_sidestep": "我还不会横着走。",
    "too_far": "这段路太长了，我一次最多走三米。",
    "too_far_back": "我看不见后面，最多只能后退三十厘米。",
    "mission_need_stand": "我得先站起来，才能出发。",
}

# one clip per character of an address, put together by lifed
DIGITS = {f"d{i}": c for i, c in enumerate("零一二三四五六七八九")} | {"dot": "点"}
DIGIT_S = 0.42          # about how long each lasts, with its gap
ADDRESS_S = 13.5        # intro + an address + the port: 12.9 s for <robot-ip>
# Never its name in a clip (2026-10-17: "叫我小白，说连网" came back through its microphone, the name
# cut the clip short as a call, and "连网" started another scan, again and again). The command words
# are in some (they say what to do); core.py drops what it hears of its own clip.

CHARS_PER_S = 4.2      # Xiaoxiao at the default rate, measured on these clips


def seconds(key: str) -> float:
    """About how long a clip lasts (lifed passes the real lengths of the recorded clips)."""
    if key == "address":
        return ADDRESS_S
    text = PROMPTS.get(key, "")
    return sum(1 for c in text if not c.isspace()) / CHARS_PER_S
