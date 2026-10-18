# -*- coding: utf-8 -*-
"""python -m pytest tools/lifed/test_attention.py — where Xiaobai looks (attention.py)."""
import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from attention import HEAD_YAW_LIMIT, Attention  # noqa: E402

R = math.radians


def run(att, t0, seconds, dt=0.1, state="IDLE", heading=0.0, rng=None, **kw):
    """Tick select + decay; returns the focuses and the time after."""
    rng = rng or random.Random(1)
    out, t = [], t0
    gaze = 0.0
    while t < t0 + seconds - 1e-9:
        att.decay(t, dt)
        f = att.select(t, state, heading, gaze, rng, **kw)
        gaze = f.yaw
        out.append(f)
        t += dt
    return out, t


class CallTest(unittest.TestCase):
    def test_its_name_from_the_left_turns_the_head_there_at_once(self):
        att = Attention()
        att.observe(0.0, "call", R(35), sigma=R(10), mirror_body=R(145))
        f = att.select(0.0, "IDLE", 0.0, 0.0, random.Random(1))
        self.assertEqual((f.kind, f.mode), ("call", "saccade"))
        self.assertAlmostEqual(f.yaw, R(35), delta=1e-6)

    def test_no_one_there_it_looks_at_the_other_answer_behind(self):
        att = Attention()
        tid = att.observe(0.0, "call", R(35), sigma=R(10), mirror_body=R(145))
        att.not_found(0.6, tid)
        f = att.select(0.7, "IDLE", 0.0, R(35), random.Random(1))
        self.assertEqual(f.yaw, HEAD_YAW_LIMIT, "145 deg is past the head's reach: as far as it goes")

    def test_a_face_where_the_call_was_is_the_person_and_settles_it(self):
        att = Attention()
        tid = att.observe(0.0, "call", R(30), sigma=R(12), mirror_body=R(150))
        tid2 = att.observe(0.5, "person", R(26), sigma=R(3), facing=True)
        self.assertEqual(tid, tid2)
        tg = att.targets[tid]
        self.assertEqual(tg.kind, "person")
        self.assertIsNone(tg.mirror)
        self.assertLess(tg.sigma, R(4))
        self.assertAlmostEqual(tg.az, R(26), delta=R(1.5))

    def test_a_call_interrupts_any_focus(self):
        att = Attention()
        att.observe(0.0, "person", R(-20), sigma=R(3), facing=True)
        run(att, 0.0, 0.3)
        att.observe(0.3, "call", R(40), sigma=R(10))
        f = att.select(0.4, "IDLE", 0.0, R(-20), random.Random(1))
        self.assertEqual(f.kind, "call")

    def test_asleep_or_walking_the_eyes_do_nothing(self):
        att = Attention()
        att.observe(0.0, "call", R(30))
        self.assertEqual(att.select(0.0, "SLEEPING", 0.0, 0.0, random.Random(1)).mode, "none")
        self.assertEqual(att.select(0.0, "IDLE", 0.0, 0.0, random.Random(1), walking=True).mode, "none")
        self.assertIn(att.targets[1].kind, ("call",), "the table is kept")


class FocusTest(unittest.TestCase):
    def test_a_focus_is_held_a_moment_before_a_like_one_takes_it(self):
        att = Attention()
        a = att.observe(0.0, "person", R(-30), sigma=R(3))
        run(att, 0.0, 0.2)
        att.observe(0.2, "person", R(30), sigma=R(3), facing=True)
        f = att.select(0.3, "IDLE", 0.0, R(-30), random.Random(1))
        self.assertEqual(f.tid, a, "dwell: not yet")

    def test_inhibition_of_return_a_sound_looked_at_is_let_go(self):
        att = Attention()
        att.observe(0.0, "sound", R(40), sigma=R(10))
        focuses, _ = run(att, 0.0, 6.0)
        self.assertEqual(focuses[0].kind, "sound")
        self.assertEqual(focuses[-1].mode, "wander", "nothing new there: it moves on")

    def test_someone_talking_in_a_conversation_is_watched_and_the_eyes_leave_now_and_then(self):
        att = Attention()
        for k in range(100):
            att.observe(k * 0.1, "person", R(10), sigma=R(3), facing=True, talking=(k % 30 < 20))
        att2 = Attention()
        focuses = []
        t = 0.0
        rng = random.Random(3)
        for k in range(150):
            if k % 3 == 0:
                att2.observe(t, "person", R(10), sigma=R(3), facing=True, talking=True)
            att2.decay(t, 0.1)
            focuses.append(att2.select(t, "ATTENDING", 0.0, 0.0, rng, conversing=True))
            t += 0.1
        modes = [f.mode for f in focuses]
        self.assertIn("look_away", modes)
        self.assertGreater(modes.count("pursue") + modes.count("saccade"), modes.count("look_away") * 3)
        away = [f for f in focuses if f.mode == "look_away"][0]
        self.assertGreater(abs(away.yaw - R(10)), R(10))

    def test_thinking_the_eyes_go_up_and_aside(self):
        att = Attention()
        att.observe(0.0, "person", R(0), sigma=R(3), facing=True, talking=True)
        run(att, 0.0, 1.5, state="ATTENDING", conversing=True)
        f = att.select(1.6, "ATTENDING", 0.0, 0.0, random.Random(2), conversing=True, thinking=True)
        self.assertEqual(f.mode, "look_away")
        self.assertGreater(f.el, 0.05)


class WorldTest(unittest.TestCase):
    def test_turning_the_body_keeps_the_target_where_it_is(self):
        att = Attention()
        att.observe(0.0, "person", R(40), heading=0.0, sigma=R(3))
        f = att.select(0.1, "IDLE", R(30), 0.0, random.Random(1))        # the body turned 30 deg left
        self.assertAlmostEqual(f.yaw, R(10), delta=1e-6)

    def test_held_far_round_it_asks_to_turn_the_body_when_it_may(self):
        att = Attention()
        for k in range(40):
            att.observe(k * 0.1, "person", R(120), sigma=R(5), talking=True)
        focuses, _ = run(att, 0.0, 3.0, can_turn=True)
        self.assertIsNone(focuses[5].want_turn, "not straight away")
        self.assertAlmostEqual(focuses[-1].want_turn, R(120), delta=R(2))
        self.assertEqual(focuses[-1].yaw, HEAD_YAW_LIMIT)
        att2 = Attention()
        att2.observe(0.0, "person", R(120), sigma=R(5), talking=True)
        focuses, _ = run(att2, 0.0, 3.0, can_turn=False)
        self.assertIsNone(focuses[-1].want_turn, "sitting / a mission / the pad: no turning")

    def test_a_person_gone_a_minute_is_remembered_as_a_place_and_looked_at_now_and_then(self):
        att = Attention()
        att.observe(0.0, "person", R(-45), sigma=R(3))
        run(att, 0.0, 65.0, dt=0.5)
        kinds = [tg.kind for tg in att.targets.values()]
        self.assertEqual(kinds, ["place"])
        focuses, _ = run(att, 65.0, 60.0, dt=0.5, rng=random.Random(5))
        self.assertTrue(any(abs(f.yaw - R(-45)) < R(3) for f in focuses), "looks back where they were")

    def test_wandering_covers_the_reachable_directions(self):
        att = Attention()
        focuses, _ = run(att, 0.0, 60.0, dt=0.5, rng=random.Random(7))
        yaws = sorted({round(f.yaw, 1) for f in focuses})
        self.assertLess(yaws[0], -0.4)
        self.assertGreater(yaws[-1], 0.4)
        self.assertTrue(all(abs(y) <= HEAD_YAW_LIMIT + 1e-9 for y in yaws))


class FusionTest(unittest.TestCase):
    def test_a_voice_and_the_tof_and_a_face_are_one_person(self):
        att = Attention()
        a = att.observe(0.0, "voice", R(15), sigma=R(12), talking=True)
        b = att.observe(0.2, "near", R(12), sigma=R(6), dist=0.6)
        c = att.observe(0.4, "person", R(13), sigma=R(3))
        self.assertEqual(len({a, b, c}), 1)
        tg = att.targets[a]
        self.assertEqual((tg.kind, tg.dist), ("person", 0.6))
        self.assertGreater(tg.talking_until, 0.4)

    def test_two_people_apart_stay_two(self):
        att = Attention()
        att.observe(0.0, "person", R(-30), sigma=R(3))
        att.observe(0.0, "person", R(30), sigma=R(3))
        self.assertEqual(len(att.targets), 2)

    def test_an_unsure_target_drifts_less_sure(self):
        att = Attention()
        tid = att.observe(0.0, "person", R(0), sigma=R(3))
        run(att, 0.0, 5.0)
        self.assertGreater(att.targets[tid].sigma, R(10))


if __name__ == "__main__":
    unittest.main()
