# -*- coding: utf-8 -*-
"""python -m unittest tools/lifed/test_judgments.py"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from judgments import decide_utterance  # noqa: E402


def ans(for_duck=0.9, intent="chat", p_intent=0.8, reply="think", p_reply=0.8):
    return {"for_duck": {"noul": for_duck},
            "intent": {"choice": intent, "probabilities": {intent: p_intent}},
            "reply": {"choice": reply, "probabilities": {reply: p_reply}}}


class UtteranceTest(unittest.TestCase):
    def test_without_jev_the_brain_answers(self):
        self.assertEqual(decide_utterance(None, named=False).action, "brain")
        self.assertEqual(decide_utterance(ans(), named=False, mode="off").action, "brain")

    def test_garbage_without_the_name_is_dropped(self):
        d = decide_utterance(ans(for_duck=0.1, intent="unclear", p_intent=0.9), named=False)
        self.assertEqual(d.action, "ignore")
        d = decide_utterance(ans(for_duck=0.6, intent="unclear", p_intent=0.9), named=False)
        self.assertEqual(d.action, "ignore", "an unclear transcript in a conversation")

    def test_said_with_the_name_it_is_never_dropped(self):
        d = decide_utterance(ans(for_duck=0.1, intent="unclear", p_intent=0.9), named=True)
        self.assertEqual(d.action, "brain")

    def test_commands_the_keywords_missed(self):
        d = decide_utterance(ans(intent="rest", p_intent=0.8), named=False)
        self.assertEqual((d.action, d.intent), ("command", "rest"))
        self.assertEqual(decide_utterance(ans(intent="rest", p_intent=0.6), named=False).action, "brain")

    def test_a_clear_command_counts_even_when_for_duck_runs_low(self):
        # the real API, 2026-10-17: "拜拜我去忙了" for_duck 0.34, intent bye 1.00
        d = decide_utterance(ans(for_duck=0.15, intent="bye", p_intent=1.0), named=False)
        self.assertEqual((d.action, d.intent), ("command", "bye"))
        d = decide_utterance(ans(for_duck=0.09, intent="chat", p_intent=0.52, reply="none"), named=False)
        self.assertEqual(d.action, "ignore", "妈我晚上不回来吃饭: for someone else")
        d = decide_utterance(ans(for_duck=0.06, intent="bye", p_intent=0.9, reply="none"), named=False)
        self.assertEqual(d.action, "ignore", "... even when it reads like a goodbye (the robot, 10-17)")
        d = decide_utterance(ans(for_duck=0.06, intent="bye", p_intent=0.9, reply="none"), named=True)
        self.assertEqual(d.action, "command", "with its name it is for the duck")

    def test_getting_up_needs_a_sure_answer(self):
        self.assertEqual(decide_utterance(ans(intent="wake", p_intent=0.8), named=True).action, "brain")
        d = decide_utterance(ans(intent="wake", p_intent=0.9), named=True)
        self.assertEqual((d.action, d.intent), ("command", "wake"))

    def test_how_to_answer_only_when_on(self):
        a = ans(reply="simple", p_reply=0.9)
        self.assertEqual(decide_utterance(a, named=True, mode="shadow").action, "brain")
        self.assertEqual(decide_utterance(a, named=True, mode="on").action, "small")
        self.assertEqual(decide_utterance(ans(reply="none", p_reply=0.9), named=True, mode="on").action, "nod")
        self.assertEqual(decide_utterance(ans(reply="none", p_reply=0.4), named=True, mode="on").action, "brain")

    def test_the_summary_is_for_the_page(self):
        s = decide_utterance(ans(for_duck=0.93, intent="chat", p_intent=0.81), named=True).summary
        self.assertEqual(s, {"for_duck": 0.93, "intent": "chat", "intent_p": 0.81, "reply": "think"})



class RecentTest(unittest.TestCase):
    def test_right_after_the_duck_spoke_a_misheard_answer_goes_to_the_brain(self):
        # 2026-10-17, after a mission: "你真棒" heard as "嗯，周棒" (0.42, unclear), "50到" (0.29, unclear)
        for p_for in (0.42, 0.29):
            a = ans(for_duck=p_for, intent="unclear", p_intent=0.95, reply="none")
            self.assertEqual(decide_utterance(a, named=False, recent=True).action, "brain")
            self.assertEqual(decide_utterance(a, named=False, recent=False).action, "ignore", "out of the blue: noise")

    def test_even_then_a_sure_not_for_the_duck_is_dropped(self):
        a = ans(for_duck=0.05, intent="chat", p_intent=0.6, reply="none")
        self.assertEqual(decide_utterance(a, named=False, recent=True).action, "ignore")


if __name__ == "__main__":
    unittest.main()
