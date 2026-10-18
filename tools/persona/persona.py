# -*- coding: utf-8 -*-
"""Who the duck is — its name, its character, its voice — apart from the framework that runs it:
the framework and an example duck are shared; a duck's own persona stays with its owner.

One JSON file. The services (voiced, lifed, duckweb on the robot; duckbrain and its judge on the
brain's box) load it from, in order:

    $DUCK_PERSONA  ->  ~/.config/duck/persona.json  ->  example.json next to this file

and whatever it holds is laid over example.json, so a persona names only what differs. JSON rather
than TOML: the brain's Python is 3.10, without tomllib. A text field may be a list of strings,
joined with nothing between them (write "\\n" where a line should break) — long prompts stay
readable.

    from persona import load
    P = load()          # P.name, P.character, ...

Each service finds this module in the `persona` directory beside its own (tools/persona in the
repo; ~/persona on the robot; the brain's directory's sibling on the brain box) — see
`sys.path` in each.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, fields

HERE = os.path.dirname(os.path.abspath(__file__))
EXAMPLE = os.path.join(HERE, "example.json")
USER_PATH = "~/.config/duck/persona.json"


@dataclass(frozen=True)
class Persona:
    # its name, as said to it and as ASR writes it
    name: str
    name_pinyin: str                  # for duck-speak lines that say it ("xiao3 bai2")
    name_aliases: tuple               # what the ASR writes for the name when it mishears it
    # who it is, for the language models
    description: str                  # third person, for the judgments (Jev): what the duck is
    character: str                    # first person: the brain's expression prompt opens with it
    chat_character: str               # first person, for plain chat (the brain's /chat)
    say_style: str                    # how its duck-speak sounds ("简单的口头话")
    say_examples: str                 # a few duck-speak lines, to set the register
    express_style: str                # how it answers like something alive: "- ...\n" lines
    # the life layer's knobs (lifed Config): 0-1
    lively: float
    talkative: float
    # the brain's spoken voice (CosyVoice, when on): style instruction, reference clip, ducktalk level
    tts_instruct: str
    tts_voice_prompt: str             # a file name under the brain's TTS directory, or a full path
    tts_level: int
    source: str = ""                  # which file it came from


def _text(v):
    return "".join(v) if isinstance(v, list) else v


def _read(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        d = json.load(f)
    if not isinstance(d, dict):
        raise ValueError(f"{path}: a persona is a JSON object")
    return d


def find(path: str | None = None) -> str:
    """The persona file to use: `path`, else $DUCK_PERSONA, else the user's, else the example."""
    for p in (path, os.environ.get("DUCK_PERSONA"), os.path.expanduser(USER_PATH)):
        if p and os.path.exists(os.path.expanduser(p)):
            return os.path.expanduser(p)
    return EXAMPLE


def load(path: str | None = None) -> Persona:
    """The persona, laid over the example's fields. Unknown keys are refused (a typo would
    otherwise fall back to the example without a word)."""
    chosen = find(path)
    d = _read(EXAMPLE)
    if chosen != EXAMPLE:
        own = _read(chosen)
        known = {f.name for f in fields(Persona)}
        unknown = sorted(k for k in own if k not in known and not k.startswith("_"))
        if unknown:
            raise ValueError(f"{chosen}: unknown persona keys {unknown}")
        d.update(own)
    d = {k: v for k, v in d.items() if not k.startswith("_")}
    d = {k: _text(v) for k, v in d.items()}
    d["name_aliases"] = tuple(d.get("name_aliases") or ())
    d["source"] = chosen
    return Persona(**d)
