# -*- coding: utf-8 -*-
"""What lifed makes of Jev's answers about what is said (docs/project/duck-life-jev-framework-20261017-CN.md
§4.1). Jev's look at a person becomes an event in visiond (tools/visiond/jevlook.py).

Jev states facts with probabilities; these functions turn them into what the duck does, with the
thresholds in one place and the duck's own safety rules on top: getting up needs the name or a
conversation and a sure answer; anything unsure falls back to what lifed did before Jev. Pure: no
IO, no clocks — tested in test_judgments.py.
"""
from __future__ import annotations

from dataclasses import dataclass

# Starting thresholds (to be calibrated on the logged judgments, framework §7)
# 2026-10-17, 12 sentences through the real API: for_duck runs low on clear commands said without
# the name ("拜拜我去忙了" 0.34, "扫一下这个码连网" 0.35, both intent 1.00) and very low on words for
# someone else ("妈我晚上不回来吃饭" 0.09); garbage is better caught by intent=unclear (0.97-0.99).
T_FOR = 0.20          # below this (and no clear command), an utterance not said with the name is not for the duck
T_CMD_FOR = 0.15      # ... but a command said without the name needs at least this (from the robot,
                      # "妈我晚上不回来吃饭" came back as intent bye with for_duck 0.06)
T_INTENT = 0.70       # a command it was not asked in the command words
T_WAKE = 0.85         # ... and for one that gets it up
T_REPLY = 0.60        # how it answers (none / simple / think)
# Right after the duck has said something, what comes back is most likely an answer to it, however it
# was heard: "你真棒" after a mission came out "嗯，周棒" / "50到" / "李真大" (2026-10-17), Jev said
# unclear, and the duck stayed silent three times. Then the brain gets it (it says "没听清" well),
# and only a near-certain "not for the duck" is dropped.
RECENT_S = 12.0
T_FOR_RECENT = 0.10

COMMANDS = ("sleep", "rest", "sit", "stand", "wake", "net", "bye")
UP = ("stand", "wake")
MODES = ("off", "shadow", "on")


@dataclass
class UtteranceDecision:
    action: str            # "brain" | "small" | "nod" | "ignore" | "command"
    intent: str | None     # the command for "command"
    why: str
    summary: dict          # what the page shows: for_duck, intent, intent_p, reply


def _choice(answers: dict, name: str) -> tuple[str | None, float]:
    a = answers.get(name) or {}
    c = a.get("choice")
    p = (a.get("probabilities") or {}).get(c, a.get("confidence", 0.0)) if c else 0.0
    return c, float(p or 0.0)


def decide_utterance(answers: dict | None, named: bool, mode: str = "shadow", recent: bool = False) -> UtteranceDecision:
    """An utterance lifed already took as addressed (its name, or inside a conversation), and that the
    command words did not catch: what to do with Jev's view of it.

    off / no answers: the brain answers, as before Jev.
    shadow: Jev may drop garbage that was not said with the name, and catch commands the keywords
            missed; how to answer stays the brain's.
    on:     also no reply for murmurs, the small brain for greetings and praise (no 0.7 s wait).
    recent: the duck spoke within RECENT_S — an unclear line is then likely a misheard answer.
    """
    if mode == "off" or not answers:
        return UtteranceDecision("brain", None, "no judgment", {})
    p_for = float((answers.get("for_duck") or {}).get("noul", 1.0))
    intent, p_intent = _choice(answers, "intent")
    reply, p_reply = _choice(answers, "reply")
    summary = {"for_duck": round(p_for, 2), "intent": intent, "intent_p": round(p_intent, 2), "reply": reply}
    need = T_WAKE if intent in UP else T_INTENT
    command = intent in COMMANDS and p_intent >= need and (named or p_for >= T_CMD_FOR)
    floor = T_FOR_RECENT if recent else T_FOR
    garbage = intent == "unclear" and p_intent >= T_INTENT and not recent
    if not named and not command and (p_for < floor or garbage):
        return UtteranceDecision("ignore", None, f"not for the duck ({p_for:.2f}, {intent})", summary)
    if command:
        return UtteranceDecision("command", intent, f"{intent} ({p_intent:.2f})", summary)
    if mode == "on" and p_reply >= T_REPLY:
        if reply == "none":
            return UtteranceDecision("nod", None, f"no reply needed ({p_reply:.2f})", summary)
        if reply == "simple":
            return UtteranceDecision("small", None, f"a simple answer ({p_reply:.2f})", summary)
    return UtteranceDecision("brain", None, "the brain answers", summary)
