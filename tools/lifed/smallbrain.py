# -*- coding: utf-8 -*-
"""The duck's small brain: a whole answer without the network (step 3 of the 2026-10-16 plan).

When the online brain (tools/duckbrain, through frp) cannot be reached — outdoors with no hotspot,
the V100 box down — the duck still answers what is said to it, in the same expression format the
online brain returns (emotion, a few words of duck-speak with their pinyin, a call before them, a
head gesture, a body move, an action, and a meaning line for the human). Duck-speak carries no
words anyone understands, so what matters is the expression: an intent matched from keywords, and
one of a few answers for it, coloured by how the duck is (asleep, tired, hot, hungry, happy).

Pure: no clocks, no IO. `respond(text, mind, rng)`; the pinyin of every line is pre-computed in
`smallbrain_pinyin.py` (the robot has no pypinyin) — regenerate it after adding lines:

    /data/microduck/tts/venv/bin/python tools/lifed/smallbrain.py --pinyin > tools/lifed/smallbrain_pinyin.py
"""
from __future__ import annotations

import random
import re
import sys
from dataclasses import dataclass

try:
    from smallbrain_pinyin import PINYIN
except ImportError:          # while generating it
    PINYIN = {}


@dataclass
class Mind:
    """What the small brain knows of the duck right now (lifed fills it in)."""
    asleep: bool = False
    posture: str = "standing"
    energy: float | None = None        # 0 tired .. 1 fresh (legs)
    head_hot: bool = False
    battery_pct: float | None = None
    valence: float = 0.2               # -1 unhappy .. 1 happy
    arousal: float = 0.3
    hour: int | None = None            # local hour, None if the clock is not set (no RTC, no NTP yet)
    minute: int = 0


def E(emotion, say, pre, gesture, body, meaning, action="none"):
    return {"emotion": emotion, "say": say, "pre": pre, "gesture": gesture, "body": body,
            "action": action, "meaning": meaning}


# intent: (keywords, answers). Earlier intents win, so the specific ones go first.
INTENTS = [
    ("sorry", ("对不起", "抱歉", "不好意思"), [
        E("撒娇", "没关系～", "coo", "nuzzle", "lean_in", "没关系呀，我不生气啦。"),
        E("开心", "嗯嗯！", "coo", "nod", "none", "好啦好啦，原谅你啦。"),
    ]),
    ("thanks", ("谢谢", "多谢", "辛苦了"), [
        E("害羞", "嘿嘿～", "coo", "tilt", "shrink", "不客气呀，能帮上忙我很开心。"),
        E("开心", "不客气！", "chirp", "nod", "bounce", "不用谢！"),
    ]),
    ("love", ("喜欢你", "爱你", "最喜欢", "想你"), [
        E("害羞", "嘿嘿～", "coo", "nuzzle", "shrink", "我也最喜欢你啦，好害羞。"),
        E("撒娇", "我也是！", "coo", "nuzzle", "lean_in", "我也喜欢你，要一直陪着我哦。"),
    ]),
    ("praise", ("真棒", "好棒", "厉害", "聪明", "好乖", "真乖", "可爱", "好看", "漂亮", "不错"), [
        E("开心", "嘎嘎！", "chirp", "nod", "bounce", "被夸啦，好开心！"),
        E("害羞", "嘿嘿～", "coo", "tilt", "shrink", "哎呀，被你夸得有点不好意思。"),
        E("开心", "哈哈哈哈", "chirp", "look_up", "bounce", "哈哈，我也觉得自己很棒！"),
    ]),
    ("scold", ("笨", "坏", "讨厌", "不乖", "烦", "闭嘴", "别吵", "吵死", "傻"), [
        E("难过", "呜……", "coo", "droop", "shrink", "呜，你说我，我有点难过。"),
        E("难过", "嘎……", "peck", "shake", "shrink", "哼，我才不笨呢……"),
        E("撒娇", "不要嘛～", "peck", "shake", "none", "不要这样说我嘛。"),
    ]),
    ("hug", ("抱抱", "摸摸", "亲亲", "贴贴", "蹭蹭"), [
        E("撒娇", "抱抱～", "coo", "nuzzle", "lean_in", "抱抱！我最喜欢贴着你了。"),
        E("开心", "嘎～", "coo", "nuzzle", "none", "好舒服，再摸摸我嘛。"),
    ]),
    ("comfort", ("难过", "伤心", "不开心", "好累", "我累", "烦死", "哭"), [
        E("难过", "抱抱～", "coo", "nuzzle", "lean_in", "别难过，我陪着你，抱抱你。"),
        E("撒娇", "不哭不哭～", "coo", "nuzzle", "lean_in", "不哭不哭，有我在呢。"),
    ]),
    ("joke", ("笑话", "笑一个", "好笑", "逗我", "哈哈"), [
        E("开心", "哈哈哈哈", "chirp", "look_up", "bounce", "哈哈哈，你逗我我就想笑！"),
        E("开心", "嘎嘎嘎嘎", "chirp", "nod", "bounce", "嘎嘎嘎，笑死我啦！"),
    ]),
    ("sing", ("唱歌", "唱首歌", "唱一个", "来首歌"), [
        E("开心", "啦啦啦～", "chirp", "nod", "bounce", "啦啦啦，我给你唱鸭子歌！"),
        E("害羞", "嘎嘎嘎～", "coo", "tilt", "bounce", "我只会嘎嘎唱，你别笑我哦。"),
    ]),
    ("dance", ("跳舞", "跳个舞", "扭一扭", "陪我玩", "玩一会", "玩游戏"), [
        E("开心", "好呀好呀！", "chirp", "nod", "bounce", "好呀好呀，我们一起玩！"),
        E("开心", "嘎嘎！", "chirp", "look_up", "bounce", "太好啦，我最喜欢玩了！"),
    ]),
    ("look", ("看我", "看这里", "看这边", "过来", "来这里"), [
        E("好奇", "嗯？", "inquire", "look_up", "lean_in", "你叫我呀？我在看着你呢。"),
        E("开心", "来啦！", "chirp", "nod", "lean_in", "我在这儿呢，看着你。"),
    ]),
    ("nod", ("点头", "点点头", "同意吗", "对不对", "是不是"), [
        E("开心", "嗯嗯！", "none", "nod", "none", "嗯嗯，对的对的！"),
    ]),
    ("shake", ("摇头", "摇摇头"), [
        E("撒娇", "不要～", "peck", "shake", "none", "不要不要～"),
    ]),
    ("who", ("你是谁", "你叫什么", "什么名字", "你是什么"), [
        E("开心", "{name}！", "greet", "nod", "bounce", "我是{name}呀，一只小机器鸭子！"),
        E("开心", "嘎嘎！{name}！", "greet", "look_up", "bounce", "我叫{name}，是你的小鸭子！"),
    ]),
    ("age", ("几岁", "多大", "生日", "哪里来", "住哪"), [
        E("好奇", "嗯……", "inquire", "tilt", "none", "我也不知道我几岁，大概刚出生不久吧。"),
        E("开心", "嘎？", "inquire", "tilt", "lean_in", "我就住在你家里呀。"),
    ]),
    ("online", ("上网", "联网", "网络", "大脑", "断网"), [
        E("难过", "连不上～", "coo", "droop", "shrink", "我现在连不上网，只能用小脑袋想事情。"),
    ]),
    ("weather", ("天气", "下雨", "冷不冷", "外面"), [
        E("好奇", "嘎？", "inquire", "tilt", "none", "我不知道外面天气怎么样呢。"),
        E("好奇", "不知道～", "inquire", "look_up", "none", "我看不到外面，你告诉我好不好？"),
    ]),
    ("bye", ("再见", "拜拜", "回头见", "晚安"), [
        E("撒娇", "拜拜～", "coo", "nod", "none", "拜拜，早点回来陪我哦。"),
    ]),
    ("hello", ("你好", "哈喽", "嗨", "早上好", "早安", "下午好", "晚上好", "我回来了", "回来啦"), [
        E("开心", "嘎嘎！", "greet", "nod", "bounce", "你好呀，见到你真开心！"),
        E("开心", "你好呀！", "greet", "look_up", "bounce", "你来啦，我等你好久了！"),
    ]),
]

# dynamic intents: their answer depends on the duck's state (handled in respond)
DYNAMIC = [
    ("tired", ("累不累", "累吗", "困不困", "困吗", "想睡", "休息够")),
    ("hot", ("热不热", "热吗", "烫", "发烧")),
    ("hungry", ("饿不饿", "饿吗", "饿了", "电量", "没电", "吃饭", "充电")),
    ("time", ("几点", "时间", "现在什么时候")),
    # asking after the duck: the sure phrases come before the keyword intents ("你好吗" is no hello),
    # the loose ones after them ("今天天气怎么样" is about the weather)
    ("how", ("你好吗", "还好吗", "开心吗", "心情", "怎么了", "咋了", "咋啦", "你在干嘛", "你干嘛呢")),
    ("how", ("怎么样", "在干嘛", "干什么", "做什么", "干嘛呢")),
]

CN_NUM = "零一二三四五六七八九十"


def hour_words(h: int) -> str:
    """12-hour clock in characters: 0 -> 十二, 13 -> 一, 14 -> 两, 10 -> 十, 11 -> 十一."""
    n = h % 12 or 12
    return "两" if n == 2 else CN_NUM[n] if n <= 10 else "十" + CN_NUM[n - 10]


def part_of_day(h: int) -> str:
    return ("半夜" if h < 5 else "早上" if h < 11 else "中午" if h < 13 else "下午" if h < 18
            else "晚上" if h < 23 else "半夜")


def _dynamic(name: str, m: Mind) -> list[dict]:
    if name == "tired":
        if m.energy is not None and m.energy < 0.35:
            return [E("困", "有点累～", "coo", "droop", "shrink", "腿有点累了，想坐下歇一会儿。"),
                    E("困", "唔……累", "coo", "droop", "none", "嗯……我有点累了。")]
        return [E("开心", "不累！", "chirp", "shake", "bounce", "一点都不累，我还能玩！"),
                E("开心", "嘎嘎！", "chirp", "nod", "bounce", "精神好着呢！")]
    if name == "hot":
        if m.head_hot:
            return [E("难过", "好热～", "coo", "droop", "shrink", "我的头有点热，要歇一歇才行。")]
        return [E("开心", "不热！", "chirp", "shake", "none", "不热呀，凉快着呢。")]
    if name == "hungry":
        if m.battery_pct is not None and m.battery_pct < 25:
            return [E("撒娇", "饿了～", "coo", "droop", "lean_in", "我快没电了，好饿，想充电。")]
        return [E("开心", "不饿！", "chirp", "shake", "bounce", "电还够呢，不饿！")]
    if name == "time":
        if m.hour is None:
            return [E("好奇", "嗯？不知道～", "inquire", "tilt", "none", "我的钟还没对上，不知道现在几点。")]
        h = m.hour
        half = "半" if 20 <= m.minute < 50 else ""
        say = f"{hour_words(h)}点{half}！"
        late = h >= 23 or h < 5
        return [E("困" if late else "开心", say, "inquire" if late else "chirp", "nod", "none",
                  f"现在是{part_of_day(h)}{hour_words(h)}点{'半' if half else ''}" + ("，好晚了，该睡觉啦。" if late else "。"))]
    if name == "how":
        if m.asleep:
            return [E("困", "唔……", "coo", "droop", "none", "我在睡觉呢，迷迷糊糊的……")]
        if m.energy is not None and m.energy < 0.35:
            return [E("困", "有点累～", "coo", "droop", "none", "我有点累了，在歇着呢。")]
        if m.valence < -0.2:
            return [E("难过", "嗯……", "coo", "droop", "shrink", "我有点不开心……")]
        doing = {"sitting": "坐着发呆", "resting": "坐着休息", "walking": "在走路", "standing": "站着看你"}
        d = doing.get(m.posture, "陪着你")
        return [E("开心", "嘎嘎！", "chirp", "nod", "bounce", f"我很好呀，正{d}呢！"),
                E("开心", "好着呢！", "chirp", "tilt", "bounce", f"挺开心的，我在{d}。")]
    return []


GENERIC_QUESTION = [
    E("好奇", "嗯？", "inquire", "tilt", "lean_in", "嗯？我没太听懂，你再说一遍好不好？"),
    E("好奇", "嘎？", "inquire", "tilt", "none", "这个问题好难，我想不出来。"),
    E("好奇", "不知道～", "inquire", "look_up", "none", "我也不知道呢。"),
]
GENERIC_STATEMENT = [
    E("开心", "嗯嗯！", "none", "nod", "none", "嗯嗯，我在听呢。"),
    E("好奇", "嘎～", "coo", "tilt", "lean_in", "是吗？然后呢？"),
    E("开心", "好呀！", "chirp", "nod", "none", "好呀。"),
]
ASLEEP = [
    E("困", "唔……", "coo", "droop", "none", "我在睡觉呢……迷迷糊糊的。"),
    E("困", "嗯……困", "coo", "none", "none", "好困呀，让我再睡一会儿。"),
]
QUESTION_RE = re.compile(r"[吗呢么？?]\s*$|^(为什么|怎么|什么|哪|谁|多少)")


def match(text: str) -> str | None:
    t = text.lower()
    for name, words in DYNAMIC[:5]:            # the state questions first: "累不累" is no scolding
        if any(w in t for w in words):
            return name
    for name, words, _ in INTENTS:
        if any(w in t for w in words):
            return name
    for name, words in DYNAMIC[5:]:
        if any(w in t for w in words):
            return name
    return None


def respond(text: str, mind: Mind, rng: random.Random | None = None, last_say: str = "",
            name: str = "小白", name_pinyin: str = "xiao3 bai2") -> dict:
    """The duck's answer to `text` as an expression, with `pinyin` and `intent`. Never fails. A line
    with `{name}` in it says the duck's own (its persona's) name."""
    rng = rng or random.Random()
    intent = match(text)
    if mind.asleep and intent not in ("hug", "love", "bye", "how", "time"):
        options, intent = ASLEEP, intent or "asleep"
    elif intent is None:
        options = GENERIC_QUESTION if QUESTION_RE.search(text.strip()) else GENERIC_STATEMENT
        intent = "question" if options is GENERIC_QUESTION else "statement"
    else:
        options = _dynamic(intent, mind) or next(a for n, _, a in INTENTS if n == intent)
    # not the same words twice in a row
    fresh = [o for o in options if o["say"].replace("{name}", name) != last_say] or options
    e = dict(rng.choice(fresh))
    # how it feels colours the delivery a little: unhappy, no bouncing; low, a smaller call
    if mind.valence < -0.3 and e["body"] == "bounce":
        e["body"] = "none"
    e["pinyin"] = PINYIN.get(e["say"], "").replace("{name}", name_pinyin)
    e["say"] = e["say"].replace("{name}", name)
    e["meaning"] = e["meaning"].replace("{name}", name)
    e["intent"] = intent
    return e


def all_says() -> list[str]:
    says = {o["say"] for _, _, opts in INTENTS for o in opts}
    says |= {o["say"] for o in GENERIC_QUESTION + GENERIC_STATEMENT + ASLEEP}
    for name, _ in DYNAMIC:
        for m in (Mind(energy=0.1, head_hot=True, battery_pct=10, valence=-0.5), Mind(), Mind(asleep=True)):
            for h in range(24):
                for minute in (0, 30):
                    m.hour, m.minute = h, minute
                    says |= {o["say"] for o in _dynamic(name, m)}
    says |= {o["say"] for o in _dynamic("time", Mind(hour=None))}
    return sorted(s for s in says if s)


if __name__ == "__main__" and "--pinyin" in sys.argv:
    from pypinyin import Style, lazy_pinyin
    print("# -*- coding: utf-8 -*-")
    print('"""Generated by `smallbrain.py --pinyin` (pypinyin, TONE3, neutral 5) — do not edit."""')
    print("PINYIN = {")
    for s in all_says():
        # a line saying the duck's name keeps `{name}`; respond() puts the persona's pinyin there
        p = " {name} ".join(" ".join(lazy_pinyin(part, style=Style.TONE3, neutral_tone_with_five=True))
                            for part in s.split("{name}")).strip()
        print(f"    {s!r}: {p!r},")
    print("}")
