# -*- coding: utf-8 -*-
"""Jev judgment requests, without calling the API: python -m unittest tools/duckbrain/test_judge.py"""
import base64
import os
import sys
import types
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import judge  # noqa: E402

JPEG = base64.b64encode(b"\xff\xd8\xff\xe0" + b"\x00" * 64).decode()


class BuildTest(unittest.TestCase):
    def test_utterance_is_text_only_with_an_unclear_way_out(self):
        s, q, assets, headers, model = judge.build("utterance", {"text": "你去歇着吧", "named": False,
                                                                "in_conversation": True})
        self.assertEqual(assets, {})
        self.assertEqual(headers, {})
        self.assertEqual(model, judge.TEXT_MODEL)
        self.assertEqual(s["text"], "你去歇着吧")
        self.assertEqual({k: v["type"] for k, v in q.items()}, {"for_duck": "noul", "intent": "choice", "reply": "choice"})
        self.assertIn("unclear", q["intent"]["criteria"])
        self.assertIn("none", q["reply"]["criteria"])

    def test_images_become_referenced_assets_with_the_header(self):
        s, q, assets, headers, model = judge.build("person", {"tof_distance_m": 0.5}, {"view": JPEG})
        (ref, asset), = assets.items()
        self.assertEqual(s["view"], ref, "the state points at the asset")
        self.assertEqual(asset, {"type": "image", "format": "jpg", "data_base64": JPEG})
        self.assertEqual(headers, judge.IMAGE_HEADER)
        self.assertEqual(model, judge.IMAGE_MODEL)
        self.assertEqual(q["face"]["type"], "box")
        self.assertIn("`view`", q["person"]["instructions"])

    def test_bad_requests_are_refused_before_any_call(self):
        with self.assertRaises(judge.JudgeError):
            judge.build("person", {}, {})                       # no image
        with self.assertRaises(judge.JudgeError):
            judge.build("person", {}, {"view": base64.b64encode(b"GIF89a").decode()})
        with self.assertRaises(judge.JudgeError):
            judge.build("horoscope", {})

    def test_no_key_no_call(self):
        old = os.environ.pop("TYPESAFE_API_KEY", None)
        try:
            with self.assertRaises(judge.JudgeError) as e:
                judge.judge("utterance", {"text": "你好"})
            self.assertEqual(str(e.exception), "no_api_key")
        finally:
            if old is not None:
                os.environ["TYPESAFE_API_KEY"] = old

    def test_answers_come_back_plain(self):
        class Fake:
            def system_one(self, s, q, **kw):
                self.kw = kw
                return types.SimpleNamespace(model="jev-test", answers={
                    "for_duck": {"type": "noul", "noul": 0.9},
                    "intent": {"type": "choice", "choice": "rest", "confidence": 0.8, "probabilities": {"rest": 0.9}}})
        fake = Fake()
        old_client, old_log = judge._client, judge.LOG
        os.environ["TYPESAFE_API_KEY"] = "test"
        judge._client, judge.LOG = fake, os.devnull
        try:
            out = judge.judge("utterance", {"text": "你去歇着吧", "named": True})
        finally:
            judge._client, judge.LOG = old_client, old_log
            del os.environ["TYPESAFE_API_KEY"]
        self.assertEqual(out["answers"]["intent"]["choice"], "rest")
        self.assertEqual(out["model"], "jev-test")
        self.assertIsNone(fake.kw["extra_body"])


if __name__ == "__main__":
    unittest.main()
