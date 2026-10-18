# -*- coding: utf-8 -*-
"""lifed's ask path with Jev and the brain faked: python -m unittest tools/lifed/test_lifed_ask.py"""
import argparse
import asyncio
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lifed  # noqa: E402

BRAIN_REPLY = {"emotion": "开心", "say": "嘎嘎", "pinyin": "ga1 ga1", "pre": "none", "gesture": "nod",
               "body": "none", "action": "none", "meaning": "我在呢"}


def make(jev="shadow", answers=None, brain=BRAIN_REPLY, brain_delay=0.05):
    a = argparse.Namespace(robotd="/tmp/none.sock", tap="/tmp/none.tap", sock="/tmp/none.sock", trace="",
                           quiet=True, brain="http://127.0.0.1:1", tz_hours=8.0, vision="/tmp/none.sock", jev=jev,
                           voice_dir="/tmp/none", audio_device="null", card="",
                           state=os.path.join(tempfile.gettempdir(), "lifed-test-state.json"))
    L = lifed.Lifed(a)
    calls = {"brain": 0, "judge": 0}

    async def fake_brain(text, session):
        calls["brain"] += 1
        await asyncio.sleep(brain_delay)
        return dict(brain) if brain else None

    async def fake_judge(kind, state, images=None, timeout=lifed.JEV_TIMEOUT_S):
        calls["judge"] += 1
        if L.jev_mode == "off":
            return None
        if answers is None:
            L._jev_fallback("no key")
        return answers
    L._call_brain = fake_brain
    L.judge = fake_judge
    return L, calls


def jev(for_duck=0.9, intent="chat", p_intent=0.8, reply="think", p_reply=0.8):
    return {"for_duck": {"noul": for_duck}, "intent": {"choice": intent, "probabilities": {intent: p_intent}},
            "reply": {"choice": reply, "probabilities": {reply: p_reply}}}


def ask(L, text, named=True):
    asyncio.run(L.ask_brain(text, "c1", named))
    return [e["type"] for e in L.events], L.events


class AskTest(unittest.TestCase):
    def test_no_jev_the_brain_answers(self):
        L, calls = make(jev="off")
        types, events = ask(L, "你在干嘛")
        self.assertEqual(types, ["reply"])
        self.assertEqual(events[-1]["say"], "嘎嘎")

    def test_jev_down_is_a_counted_fallback_and_the_brain_answers(self):
        L, calls = make(answers=None)
        types, _ = ask(L, "你在干嘛")
        self.assertEqual(types, ["reply"])
        self.assertEqual(L.jev_stats["fallback"], 1)

    def test_garbage_is_not_answered(self):
        L, calls = make(answers=jev(for_duck=0.1, intent="unclear", p_intent=0.9))
        types, events = ask(L, "申请吧", named=False)
        self.assertEqual(types, ["judged", "ignored"])
        self.assertEqual(events[0]["summary"]["action"], "ignore")

    def test_a_missed_command_is_carried_out_instead_of_answered(self):
        L, calls = make(answers=jev(intent="rest", p_intent=0.85))
        types, events = ask(L, "你去歇着吧")
        self.assertEqual(types, ["judged", "command", "ignored"])
        self.assertEqual(events[1], {"type": "command", "name": "rest", "text": "你去歇着吧", "via": "jev"})

    def test_mode_on_greetings_get_the_small_brain_at_once(self):
        L, calls = make(jev="on", answers=jev(intent="chat", reply="simple", p_reply=0.9), brain_delay=5.0)
        types, events = ask(L, "你好呀")
        self.assertEqual(types, ["judged", "reply"])
        self.assertEqual(events[-1]["via"], "small", "no waiting for the brain")

    def test_brain_down_the_small_brain_answers(self):
        L, calls = make(answers=jev(), brain=None)
        types, events = ask(L, "你在干嘛")
        self.assertEqual(types, ["judged", "reply"])
        self.assertEqual(events[-1]["via"], "small")



class LookTest(unittest.TestCase):
    def run_look(self, L):
        asyncio.run(L.look({"text": "你看这是什么", "session": "c1"}))
        return L.events[-1]

    def test_offline_it_says_so_in_a_persons_words(self):
        L, _ = make()
        L.brain_down_until = lifed.time.monotonic() + 60
        ev = self.run_look(L)
        self.assertEqual((ev["type"], ev["voice"]), ("reply", "look_offline"))

    def test_no_photo_it_says_its_eyes_cannot(self):
        L, _ = make()
        L._vision_call = lambda *a, **k: {"ok": False, "reason": "busy"}
        ev = self.run_look(L)
        self.assertEqual(ev["voice"], "look_failed")

    def test_the_brain_answers_about_the_photo(self):
        L, _ = make()
        L._vision_call = lambda *a, **k: {"ok": True, "id": "123", "jpeg_b64": "AAAA"}
        sent = {}

        def fake_urlopen(req, timeout):
            import io
            import json
            sent.update(json.loads(req.data))
            return io.BytesIO(json.dumps({"emotion": "开心", "say": "杯子！", "pinyin": "bei1 zi5 ！",
                                          "meaning": "是一个红色的杯子。", "seen": True}).encode())
        real = lifed.urllib.request.urlopen
        lifed.urllib.request.urlopen = fake_urlopen
        try:
            ev = self.run_look(L)
        finally:
            lifed.urllib.request.urlopen = real
        self.assertEqual(sent["image"], "AAAA")
        self.assertEqual(sent["session"], "c1")
        self.assertEqual((ev["via"], ev["photo"], ev["meaning"]), ("eyes", "123", "是一个红色的杯子。"))



class MissionRunTest(unittest.TestCase):
    def test_a_dry_run_of_the_route_reports_each_step(self):
        import mission
        L, _ = make()
        r = mission.parse_route("往前走一米，再左转，再往前走半米，最后坐下")
        L.mission_cmd({"cmd": "propose", "text": "route", "steps": r.steps})
        L.mission_cmd({"cmd": "start"})
        t = lifed.time.monotonic()

        async def go():
            for i in range(800):
                if L.mission.state != "running":
                    break
                await L.mission_tick(t + i * 0.1, 0.1)
        asyncio.run(go())
        whats = [e["what"] for e in L.events if e["type"] == "mission"]
        self.assertEqual(whats, ["step_done"] * 4 + ["done"])
        snap = L.snapshot(log_lines=False)["mission"]
        self.assertEqual((snap["state"], snap["mode"], snap["sim"]["posture"]), ("done", "dry", "sitting"))
        self.assertAlmostEqual(snap["sim"]["x"], 1.0, delta=0.15)
        self.assertAlmostEqual(snap["sim"]["y"], 0.5, delta=0.15)



class RealMissionTest(unittest.TestCase):
    """missions real: the twist to robotd, and what stops it (no robot: notify / request recorded)."""

    def setUp(self):
        import mission
        self.mission = mission
        self.L, _ = make()
        self.L.set_missions_mode("real")
        self.sent = []

        async def notify(method, params):
            self.sent.append((method, dict(params)))

        async def request(method, params=None, timeout=3.0):
            self.sent.append((method, dict(params or {})))
            return {"result": {"accepted": True}}
        self.L.robot.notify, self.L.robot.request = notify, request
        self.L.body = lifed.Body(policy="stand", odom=(0.0, 0.0, 0.0), temps_c=[40.0] * 15)
        self.L.tof_at, self.L.tof_m = lifed.time.monotonic(), None
        r = mission.parse_route("往前走半米，坐下")
        self.L.mission_cmd({"cmd": "propose", "text": "x", "steps": r.steps})
        self.L.mission_cmd({"cmd": "start"})

    def tick(self, n=1):
        async def go():
            for _ in range(n):
                await self.L.mission_tick(lifed.time.monotonic(), 0.1)
        asyncio.run(go())

    def moves(self):
        return [p for m, p in self.sent if m == "robot.move"]

    def test_it_walks_at_the_robots_pace_with_the_tape_scale(self):
        self.assertEqual((self.L.mission.gait, self.L.mission.odom_scale),
                         (self.mission.ROBOT_GAIT, self.mission.ODOM_SCALE))
        self.tick()
        self.assertEqual(self.moves()[-1], {"vx": 0.27, "vy": 0.0, "vyaw": 0.0})
        self.L.body = lifed.Body(policy="walk", odom=(0.47, 0.0, 0.0), twist=(0.27, 0, 0), temps_c=[40.0] * 15)
        self.tick()
        self.assertEqual(self.moves()[-1]["vx"], 0.0, "0.47 m on the odometry is 0.51 m: there")

    def test_the_pad_stops_it(self):
        self.tick()
        self.L.body = lifed.Body(policy="walk", odom=(0.1, 0.0, 0.0), pad_active=True, temps_c=[40.0] * 15)
        self.tick()
        self.assertEqual((self.L.mission.state, self.L.mission.why), ("paused", "the pad"))
        self.assertEqual(self.moves()[-1], {"vx": 0.0, "vy": 0.0, "vyaw": 0.0})

    def test_a_connected_pad_left_alone_lets_it_walk(self):
        self.tick()
        self.L.body = lifed.Body(policy="walk", odom=(0.1, 0.0, 0.0), pad_present=True, twist=(0.27, 0, 0),
                                 temps_c=[40.0] * 15)
        self.tick()
        self.assertEqual(self.L.mission.state, "running")
        self.assertEqual(self.moves()[-1]["vx"], 0.27)

    def test_no_tof_no_walking(self):
        self.L.tof_at = -1e9
        self.tick()
        self.assertEqual((self.L.mission.state, self.L.mission.why), ("paused", "no ToF"))
        self.assertFalse(any(m["vx"] for m in self.moves()), "never set off")

    def test_something_close_ahead_stops_it(self):
        self.L.tof_m = 0.2
        self.tick()
        self.assertEqual((self.L.mission.state, self.L.mission.why), ("paused", "obstacle"))

    def test_a_fall_fails_it(self):
        self.tick()
        self.L.body = lifed.Body(policy="limp_fall", odom=(0.1, 0.0, 0.0), temps_c=[40.0] * 15)
        self.tick()
        self.assertEqual((self.L.mission.state, self.L.mission.why), ("failed", "fell"))

    def test_switching_back_to_dry_calls_it_off(self):
        self.tick()
        self.L.set_missions_mode("dry")
        self.assertEqual(self.L.mission.state, "cancelled")


if __name__ == "__main__":
    unittest.main()
