# -*- coding: utf-8 -*-
"""The small brain (offline answers): python -m unittest tools/lifed/test_smallbrain.py"""
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from smallbrain import Mind, all_says, match, respond  # noqa: E402
from smallbrain_pinyin import PINYIN  # noqa: E402

CALLS = ("greet", "inquire", "coo", "chirp", "peck", "none")
GESTURES = ("nod", "shake", "tilt", "look_up", "droop", "nuzzle", "none")
BODIES = ("bounce", "lean_in", "shrink", "none")


class SmallBrainTest(unittest.TestCase):
    def test_every_line_has_its_pinyin(self):
        missing = [s for s in all_says() if s not in PINYIN]
        self.assertEqual(missing, [], "regenerate smallbrain_pinyin.py (see smallbrain.py)")

    def test_it_says_its_own_name(self):
        for seed in range(6):
            e = respond("你是谁", Mind(), random.Random(seed), name="豆豆", name_pinyin="dou4 dou5")
            self.assertIn("豆豆", e["say"] + e["meaning"])
            self.assertNotIn("{name}", e["say"] + e["meaning"] + e["pinyin"])
            if "豆豆" in e["say"]:
                self.assertIn("dou4 dou5", e["pinyin"])

    def test_intents(self):
        cases = {
            "你累不累呀": "tired", "热不热": "hot", "你饿不饿": "hungry", "现在几点了": "time",
            "你好吗": "how", "你怎么了呀": "how", "今天天气怎么样": "weather", "你好呀小白": "hello",
            "你真棒": "praise", "你好笨": "scold", "抱抱": "hug", "我好难过": "comfort", "给我唱首歌": "sing",
            "你是谁": "who", "谢谢你": "thanks", "对不起": "sorry", "我喜欢你": "love", "陪我玩": "dance",
            "你在干嘛": "how", "讲个笑话": "joke", "先不要动哦": None,
        }
        for text, want in cases.items():
            self.assertEqual(match(text), want, text)

    def test_answers_are_well_formed(self):
        rng = random.Random(1)
        for text in ("你好", "几点了", "你累吗", "随便说点什么", "这是什么？", "你真可爱", "唱歌"):
            for mind in (Mind(), Mind(asleep=True), Mind(energy=0.1, head_hot=True, battery_pct=8, valence=-0.6),
                         Mind(hour=21, minute=35)):
                e = respond(text, mind, rng)
                self.assertTrue(e["pinyin"], (text, e))
                self.assertIn(e["pre"], CALLS)
                self.assertIn(e["gesture"], GESTURES)
                self.assertIn(e["body"], BODIES)
                self.assertEqual(e["action"], "none", "the small brain never moves the body")
                self.assertTrue(e["meaning"])

    def test_the_state_colours_the_answer(self):
        rng = random.Random(1)
        self.assertEqual(respond("你累不累", Mind(energy=0.1), rng)["emotion"], "困")
        self.assertEqual(respond("你累不累", Mind(energy=0.9), rng)["emotion"], "开心")
        self.assertIn("没电", respond("饿不饿", Mind(battery_pct=10), rng)["meaning"])
        self.assertEqual(respond("你真棒", Mind(asleep=True), rng)["emotion"], "困", "asleep, it mumbles")

    def test_the_time_only_if_the_clock_is_set(self):
        rng = random.Random(1)
        self.assertIn("不知道", respond("几点了", Mind(hour=None), rng)["meaning"])
        e = respond("几点了", Mind(hour=21, minute=35), rng)
        self.assertEqual(e["say"], "九点半！")
        self.assertIn("晚上九点半", e["meaning"])
        self.assertIn("该睡觉", respond("几点了", Mind(hour=0, minute=5), rng)["meaning"])

    def test_not_the_same_words_twice_running(self):
        rng = random.Random(3)
        last = ""
        for _ in range(20):
            e = respond("随便聊聊", Mind(), rng, last_say=last)
            self.assertNotEqual(e["say"], last)
            last = e["say"]


if __name__ == "__main__":
    unittest.main()
