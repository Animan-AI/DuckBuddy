# -*- coding: utf-8 -*-
"""Scenario tests for the life layer, in simulated time: python -m unittest tools/lifed/test_core.py"""
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core import Body, Config, Life, intent_of  # noqa: E402

DT = 0.1


def standing(**kw):
    return Body(policy="stand", temps_c=[40.0] * 15, battery_pct=80.0, **kw)


def run(life, body, secs, events=None):
    acts = []
    for i in range(int(round(secs / DT))):
        acts += life.step(DT, body, (events or []) if i == 0 else [])
    return acts


def kinds(acts, kind):
    return [a for a in acts if a.kind == kind]


def scans(acts):
    """The eyes asked for something other than their own watch (a QR scan, a stop)."""
    return [a for a in acts if a.kind == "vision" and a.args.get("cmd") != "watch"]


class LifeTest(unittest.TestCase):
    def setUp(self):
        self.life = Life(Config(deep_ok=True), random.Random(1))

    def test_off_until_a_policy_drives_then_it_wakes_and_idles(self):
        acts = run(self.life, Body(policy="held"), 5)
        self.assertEqual(self.life.state, "OFF")
        self.assertEqual(acts, [], "a robot nobody started is left alone")
        acts = run(self.life, Body(policy="held", fallen=True), 5)
        self.assertEqual((self.life.state, acts), ("OFF", []), "limp on its side is off, not fallen")
        acts = run(self.life, standing(), 6)
        self.assertEqual(self.life.state, "IDLE")
        self.assertTrue(any(a.args.get("tag") == "greet" for a in kinds(acts, "sound")))
        acts = run(self.life, standing(), 60)
        self.assertTrue(kinds(acts, "gaze") or kinds(acts, "gesture"), "an idle duck moves now and then")

    def test_it_never_asks_for_power_or_walking(self):
        acts = run(self.life, standing(), 600, [{"type": "say", "text": "你好"}])
        self.assertTrue({a.kind for a in acts} <= {"gaze", "gaze_stop", "gesture", "sound", "rest", "wake", "vision"})

    def test_a_call_turns_the_head_and_attends(self):
        run(self.life, standing(), 6)
        acts = run(self.life, standing(), 0.2, [{"type": "call", "yaw": 0.5}])
        self.assertEqual(self.life.state, "ATTENDING")
        self.assertEqual(kinds(acts, "gaze")[0].args["yaw"], 0.5)
        run(self.life, standing(), 25)
        self.assertEqual(self.life.state, "ATTENDING", "a call opens a conversation: it keeps listening")
        run(self.life, standing(), 10)
        self.assertEqual(self.life.state, "IDLE", "attention lapses when nothing more happens")

    def test_sleep_lies_down_deep_and_wake_gets_up(self):
        run(self.life, standing(), 6)
        acts = run(self.life, standing(), 0.2, [{"type": "say", "text": "晚安"}])
        self.assertEqual(self.life.state, "SLEEPING")
        self.assertEqual(kinds(acts, "rest")[0].args, {"on": True, "deep": True})
        lying = Body(policy="lie", rest={"phase": "resting", "deep": True}, temps_c=[40.0] * 15)
        acts = run(self.life, lying, 30)
        self.assertFalse(kinds(acts, "gesture") + kinds(acts, "gaze"), "asleep, the head is still")
        acts = run(self.life, lying, 0.2, [{"type": "command", "name": "wake"}])
        self.assertEqual(len(kinds(acts, "wake")), 1)
        self.assertEqual(self.life.state, "WAKING")

    def test_tired_rests_seated_and_a_hot_head_lies_down(self):
        run(self.life, standing(), 6)
        hot = standing()
        hot.temps_c = [66.0] * 15                    # energy ~0.12
        acts = run(self.life, hot, 0.5)
        self.assertEqual(self.life.state, "RESTING")
        self.assertEqual(kinds(acts, "rest")[0].args["deep"], True, "the head is 66 C: lie down")
        life = Life(Config(rest_head_c=99.0, deep_ok=True), random.Random(1))
        run(life, standing(), 6)
        acts = run(life, hot, 0.5)
        self.assertEqual(kinds(acts, "rest")[0].args["deep"], False)

    def test_a_fall_cries_out_and_waits_to_be_up(self):
        run(self.life, standing(), 6)
        acts = run(self.life, Body(policy="limp_fall", fallen=True, temps_c=[40.0] * 15), 1)
        self.assertEqual(self.life.state, "FALLEN")
        self.assertTrue(any(a.args.get("tag") == "alarm" for a in kinds(acts, "sound")))
        run(self.life, standing(), 3)
        self.assertEqual(self.life.state, "IDLE")

    def test_the_pad_leads_voice_waits_and_the_pad_interrupts(self):
        run(self.life, standing(), 6)
        driving = standing(pad_present=True, pad_active=True)
        acts = run(self.life, driving, 30, [{"type": "say", "text": "休息一下"}])
        self.assertEqual(self.life.state, "IDLE", "a voice ask waits while the pad is in use")
        self.assertFalse(kinds(acts, "rest") + kinds(acts, "gesture"), "nothing moves under the pad")
        # pad idle: voice is carried out
        idle_pad = standing(pad_present=True, pad_active=False)
        acts = run(self.life, idle_pad, 0.2, [{"type": "say", "text": "休息一下"}])
        self.assertEqual(self.life.state, "RESTING")
        self.assertTrue(self.life.own_rest)
        # the pad moves: the rest this layer asked for ends at once
        acts = run(self.life, driving, 0.2)
        self.assertEqual(len(kinds(acts, "wake")), 1)
        self.assertFalse(self.life.own_rest)

    def test_a_hot_duck_under_the_pad_says_so_and_rests_once_let_go(self):
        run(self.life, standing(), 6)
        hot = standing(pad_present=True, pad_active=True)
        hot.temps_c = [66.0] * 15
        acts = run(self.life, hot, 5)
        self.assertFalse(kinds(acts, "rest"), "not sat down under the user's hands")
        self.assertTrue(kinds(acts, "sound"))
        hot.pad_active = False
        acts = run(self.life, hot, 0.5)
        self.assertEqual(self.life.state, "RESTING")
        run(self.life, standing(pad_present=True, pad_active=True), 0.5)
        self.assertEqual(self.life.state, "RESTING", "a rest the body needed is not the pad's to cancel")

    def test_quiet_mutes_unprompted_sounds_only(self):
        life = Life(Config(quiet=True, lonely_social=0.0), random.Random(1))
        run(life, standing(), 6)
        acts = run(life, standing(), 60)
        self.assertFalse(kinds(acts, "sound"))
        acts = run(life, standing(), 0.2, [{"type": "touch"}])
        self.assertTrue(kinds(acts, "sound"), "a touch is still answered")

    def test_without_lying_sleep_and_a_hot_head_rest_seated(self):
        life = Life(Config(deep_ok=False), random.Random(1))
        run(life, standing(), 6)
        acts = run(life, standing(), 0.2, [{"type": "command", "name": "sleep"}])
        self.assertEqual(kinds(acts, "rest")[0].args["deep"], False)

    def test_woken_from_the_pad_it_waits_for_the_hands_to_leave(self):
        acts = run(self.life, standing(pad_present=True, pad_active=True), 5)
        self.assertEqual(self.life.state, "WAKING")
        self.assertFalse(kinds(acts, "gesture"))
        acts = run(self.life, standing(pad_present=True), 6)
        self.assertTrue(any(a.args.get("name") == "wake_up" for a in kinds(acts, "gesture")))
        self.assertEqual(self.life.state, "IDLE")

    def test_a_hot_head_keeps_still_and_tired_means_the_legs(self):
        life = Life(Config(deep_ok=False), random.Random(1))     # no deep rest: a hot head cannot rest
        run(life, standing(), 6)
        warm = standing()
        warm.temps_c = [45.0] * 5 + [69.0] * 4 + [50.0] + [45.0] * 5   # head 69, legs 45
        acts = run(life, warm, 120, [{"type": "call", "yaw": 0.5}])
        self.assertFalse(kinds(acts, "gaze") + kinds(acts, "gesture"), "a hot head stays still")
        self.assertTrue(kinds(acts, "sound"), "and still answers")
        self.assertFalse(kinds(acts, "rest"), "the legs are cool: not tired")
        cool = standing()
        cool.temps_c = [45.0] * 5 + [63.0] * 4 + [50.0] + [45.0] * 5   # head 63: calmer, not still
        acts = run(Life(Config(), random.Random(1)), cool, 60)
        self.assertTrue(kinds(acts, "gaze") + kinds(acts, "gesture"))

    def test_a_rest_robotd_does_not_take_is_not_asked_for_again_and_again(self):
        run(self.life, standing(), 6)
        hot = standing()
        hot.temps_c = [66.0] * 15
        acts = run(self.life, hot, 60)
        self.assertLessEqual(len(kinds(acts, "rest")), 2)

    def test_overheard_words_are_not_commands(self):
        run(self.life, standing(), 6)
        run(self.life, standing(), 0.2, [{"type": "say", "text": "晚安", "addressed": False}])
        self.assertNotEqual(self.life.state, "SLEEPING", "said to someone else")
        acts = run(self.life, standing(), 0.2, [{"type": "call", "yaw": 0.0},
                                                 {"type": "say", "text": "晚安", "addressed": True}])
        self.assertEqual(self.life.state, "SLEEPING")

    def test_intents(self):
        self.assertEqual(intent_of("晚安啦"), "sleep")
        self.assertEqual(intent_of("醒醒"), "wake")
        self.assertEqual(intent_of("你坐下歇会儿吧"), "rest")
        self.assertIsNone(intent_of("今天天气不错"))


class ConversationTest(unittest.TestCase):
    """Talking to it: what is said goes to the brain, and the answer comes back as a duck's."""

    def setUp(self):
        self.life = Life(Config(), random.Random(2))
        run(self.life, standing(), 8)             # awake and idle

    def test_it_asks_the_brain_and_hums_while_it_thinks(self):
        # (a command word — 你好, 睡觉 — is taken here, without the brain)
        acts = run(self.life, standing(), 0.5, [{"type": "say", "text": "今天玩什么", "addressed": True}])
        self.assertEqual([a.args["text"] for a in kinds(acts, "ask")], ["今天玩什么"])
        self.assertTrue(any(a.args.get("name") == "nod" for a in kinds(acts, "gesture")), "a nod: heard you")
        acts = run(self.life, standing(), 1.5)
        self.assertEqual([a.args["pinyin"] for a in kinds(acts, "speak")], ["n1 ～"], "thinking, aloud")
        # not addressed: no question to the brain
        acts = run(self.life, standing(), 3, [{"type": "say", "text": "今天下雨", "addressed": False}])
        self.assertEqual(kinds(acts, "ask"), [])

    def test_the_answer_is_a_call_then_duck_speak_and_a_gesture_and_it_does_not_hear_itself(self):
        run(self.life, standing(), 0.3, [{"type": "say", "text": "今天玩什么", "addressed": True}])
        reply = {"type": "reply", "emotion": "开心", "say": "好呀！", "pinyin": "hao3 ya5 ！", "pre": "greet",
                 "gesture": "nod", "body": "bounce", "action": "none", "meaning": "你好呀"}
        acts = run(self.life, standing(), 0.3, [reply])
        self.assertEqual([a.args["tag"] for a in kinds(acts, "sound")], ["greet"])
        speak = kinds(acts, "speak")
        self.assertEqual(len(speak), 1)
        self.assertEqual((speak[0].args["pinyin"], speak[0].args["mood"]), ("hao3 ya5 ！", "开心"))
        self.assertGreater(speak[0].args["delay_s"], 0.0, "the call first, then the words")
        self.assertTrue(kinds(acts, "gesture"))
        self.assertFalse(self.life.snapshot(standing())["asking"])
        # its own quacks, transcribed, while it is still speaking: not a question
        acts = run(self.life, standing(), 0.3, [{"type": "say", "text": "嘎嘎", "addressed": True}])
        self.assertEqual(kinds(acts, "ask"), [])

    def test_its_own_call_is_not_someone_talking(self):
        acts = run(self.life, standing(), 0.3, [{"type": "call", "text": "小白"}])
        self.assertTrue(kinds(acts, "sound"), "it answers a call with a call")
        acts = run(self.life, standing(), 0.3, [{"type": "say", "text": "啊", "addressed": True}])
        self.assertEqual(kinds(acts, "ask"), [], "its own call, heard back")
        acts = run(self.life, standing(), 2.0)
        acts = run(self.life, standing(), 0.3, [{"type": "say", "text": "你在干嘛", "addressed": True}])
        self.assertEqual(len(kinds(acts, "ask")), 1)

    def test_the_brain_sits_and_stands_it_only_from_where_that_makes_sense(self):
        sit = {"type": "reply", "emotion": "开心", "say": "好", "pinyin": "hao3", "pre": "none", "gesture": "nod"}
        acts = run(self.life, standing(), 0.3, [{**sit, "action": "sit"}])
        self.assertEqual([a.args["skill"] for a in kinds(acts, "skill")], ["sit_toggle"])
        seated = Body(policy="sit", temps_c=[40.0] * 15, battery_pct=80.0)
        acts = run(self.life, seated, 0.3, [{**sit, "action": "sit"}])
        self.assertEqual(kinds(acts, "skill"), [], "already sitting")
        acts = run(self.life, seated, 0.3, [{**sit, "action": "stand"}])
        self.assertEqual([a.args["skill"] for a in kinds(acts, "skill")], ["sit_toggle"])
        walking = Body(policy="walk", twist=(0.2, 0.0, 0.0), temps_c=[40.0] * 15, battery_pct=80.0)
        acts = run(self.life, walking, 0.3, [{**sit, "action": "sit"}])
        self.assertEqual(kinds(acts, "skill"), [], "not mid-stride")

    def test_the_brain_cannot_wake_a_duck_that_is_not_asleep(self):
        acts = run(self.life, standing(), 0.3, [{"type": "reply", "emotion": "惊讶", "say": "哇", "pinyin": "wa5",
                                                 "pre": "none", "gesture": "look_up", "action": "wake"}])
        self.assertEqual(kinds(acts, "wake"), [])
        self.assertEqual(self.life.state in ("IDLE", "ATTENDING"), True)
        acts = run(self.life, standing(), 0.3, [{"type": "reply", "emotion": "困", "say": "嗯", "pinyin": "n2",
                                                 "pre": "none", "gesture": "droop", "action": "sleep"}])
        self.assertEqual(self.life.state, "SLEEPING")
        self.assertTrue(kinds(acts, "rest"))
        # asleep, the brain cannot get it up either: only the wake words power the legs (2026-10-16:
        # "怎么了呀" came back as wake and the duck stood up on a half-rebuilt leg)
        for act in ("wake", "stand"):
            acts = run(self.life, standing(), 0.3, [{"type": "reply", "emotion": "困", "say": "嗯", "pinyin": "n2",
                                                     "pre": "none", "gesture": "droop", "action": act}])
            self.assertEqual(kinds(acts, "wake"), [], act)
            self.assertEqual(self.life.state, "SLEEPING", act)
        acts = run(self.life, standing(), 0.3, [{"type": "say", "text": "醒醒", "addressed": True}])
        self.assertTrue(kinds(acts, "wake"), "the wake words still do")


class CommandWordsTest(unittest.TestCase):
    """休息 lets it all go like sleep; 坐下 / 站起来 with the power on; and the commands that only
    power things down work without its name when said short (the user, 2026-10-16)."""

    def setUp(self):
        self.life = Life(Config(deep_ok=True), random.Random(1))
        run(self.life, standing(), 6)

    def say(self, text, addressed, body=None, secs=0.3):
        return run(self.life, body or standing(), secs, [{"type": "say", "text": text, "addressed": addressed, "named": addressed}])

    def test_rest_is_the_deep_rest(self):
        acts = self.say("小白你休息吧", True)
        self.assertEqual([a.args for a in kinds(acts, "rest")], [{"on": True, "deep": True}])
        self.assertEqual(self.life.state, "RESTING")

    def test_after_a_goodbye_short_power_down_words_need_no_name(self):
        self.assertEqual(kinds(self.say("你睡觉吧", False), "rest"), [], "out of the blue, it needs its name")
        run(self.life, standing(), 1.5, [{"type": "call", "yaw": 0.0}])
        self.say("拜拜了，再见", False, secs=17.0)      # the user's 2026-10-16 evening, 17 s apart
        acts = self.say("你睡觉吧", False)
        self.assertEqual(self.life.state, "SLEEPING")
        self.assertEqual([a.args.get("deep") for a in kinds(acts, "rest")], [True])
        self.assertFalse(self.life.in_convo(), "a command to sleep is no conversation")

    def test_a_long_sentence_or_getting_up_still_needs_the_name(self):
        acts = self.say("我昨天晚上很晚才去睡觉的", False)
        self.assertEqual(kinds(acts, "rest"), [])
        self.assertIn(self.life.state, ("IDLE", "ATTENDING"))
        self.say("睡觉", True)
        acts = self.say("起来", False)
        self.assertEqual(kinds(acts, "wake"), [], "unnamed, nothing gets it up")
        acts = self.say("小白起来", True)
        self.assertTrue(kinds(acts, "wake"))

    def test_sit_down_and_stand_up_with_the_power_on(self):
        acts = self.say("坐下", True)
        self.assertEqual([a.args for a in kinds(acts, "skill")], [{"skill": "sit_toggle"}])
        sitting = Body(policy="sit", temps_c=[40.0] * 15, battery_pct=80.0)
        run(self.life, sitting, 2.0)
        acts = self.say("小白站起来", True, body=sitting)
        self.assertEqual([a.args for a in kinds(acts, "skill")], [{"skill": "sit_toggle"}])
        self.assertEqual(kinds(acts, "wake"), [])


class NetTest(unittest.TestCase):
    """"小白，连网": it asks the eyes to scan for a Wi-Fi QR code and says how it went."""

    def setUp(self):
        self.life = Life(Config(deep_ok=True), random.Random(1))
        run(self.life, standing(), 6)

    def test_it_scans_and_says_how_it_went(self):
        acts = run(self.life, standing(), 0.3, [{"type": "say", "text": "小白连网", "addressed": True, "named": True}])
        self.assertEqual([a.args for a in scans(acts)], [{"cmd": "scan_qr", "timeout_s": 30.0}])
        self.assertTrue(self.life.snapshot(standing())["scanning"])
        acts = run(self.life, standing(), 0.3, [{"type": "qr", "result": "joined", "ssid": "我的手机"}])
        self.assertEqual([a.args["clip"] for a in kinds(acts, "voice")], ["joined"], "said in a person's words")
        self.assertIn("lian2 shang4 la5 ！", [a.args.get("pinyin") for a in kinds(acts, "voice")], "duck-speak if no clip")
        snap = self.life.snapshot(standing())
        self.assertFalse(snap["scanning"])
        self.assertIn("连上 我的手机 啦！", [d.get("meaning") for d in snap["dialog"]])

    def test_the_network_it_is_already_on(self):
        run(self.life, standing(), 0.3, [{"type": "say", "text": "小白连网", "addressed": True, "named": True}])
        acts = run(self.life, standing(), 0.3, [{"type": "qr", "result": "already", "ssid": "Home-7k2Q"}])
        self.assertEqual([a.args["clip"] for a in kinds(acts, "voice")], ["already"])
        self.assertIn("我已经连着 Home-7k2Q 啦。", [d.get("meaning") for d in self.life.snapshot(standing())["dialog"]])

    def test_it_says_what_to_do_and_keeps_scanning_while_joining(self):
        acts = run(self.life, standing(), 0.3, [{"type": "say", "text": "小白连网", "addressed": True, "named": True}])
        self.assertEqual([a.args["clip"] for a in kinds(acts, "voice")], ["scan_start"])
        run(self.life, standing(), 9.0)
        acts = run(self.life, standing(), 0.3, [{"type": "qr", "result": "joining", "ssid": "我的手机"}])
        self.assertEqual([a.args["clip"] for a in kinds(acts, "voice")], ["joining"])
        self.assertTrue(self.life.snapshot(standing())["scanning"], "still at it")
        run(self.life, standing(), 5.0)
        acts = run(self.life, standing(), 0.3, [{"type": "qr", "result": "not_found", "ssid": "我的手机"}])
        self.assertEqual([a.args["clip"] for a in kinds(acts, "voice")], ["not_found"])
        self.assertFalse(self.life.snapshot(standing())["scanning"])

    def test_its_own_clip_coming_back_is_not_a_new_scan(self):
        # 2026-10-17: "...再叫我连网" heard through its own microphone started scan after scan
        run(self.life, standing(), 0.3, [{"type": "say", "text": "小白连网", "addressed": True, "named": True}])
        run(self.life, standing(), 9.0)
        run(self.life, standing(), 0.3, [{"type": "qr", "result": "timeout"}])
        run(self.life, standing(), 5.0)
        acts = run(self.life, standing(), 0.3, [{"type": "say", "text": "想再试就叫我连网", "addressed": True}])
        self.assertEqual(scans(acts), [], "its own words")
        run(self.life, standing(), 8.0)
        acts = run(self.life, standing(), 0.3, [{"type": "say", "text": "小白连网", "addressed": True, "named": True}])
        self.assertEqual(len(scans(acts)), 1, "later, from someone, it scans again")

    def test_cancel_stops_the_scan_and_says_so(self):
        run(self.life, standing(), 0.3, [{"type": "say", "text": "小白连网", "addressed": True, "named": True}])
        run(self.life, standing(), 14.0)    # after its prompt (which itself ends in "就说取消")
        acts = run(self.life, standing(), 0.3, [{"type": "say", "text": "取消", "addressed": True}])
        self.assertEqual([a.args for a in scans(acts)], [{"cmd": "stop"}])
        self.assertEqual([a.args["clip"] for a in kinds(acts, "voice")], ["cancelled"])
        self.assertFalse(self.life.snapshot(standing())["scanning"])
        acts = run(self.life, standing(), 3.0, [{"type": "qr", "result": "cancelled"}])
        self.assertEqual(kinds(acts, "voice"), [], "said already")

    def test_cancel_right_after_the_prompt_is_heard(self):
        # 2026-10-17: "不想连了，取消了吧" 2 s after the prompt ended was taken for the prompt's echo
        run(self.life, standing(), 0.3, [{"type": "say", "text": "小白连网", "addressed": True, "named": True}])
        run(self.life, standing(), 12.4)
        acts = run(self.life, standing(), 0.3, [{"type": "say", "text": "不想连了，取消了吧", "addressed": True}])
        self.assertEqual([a.args for a in scans(acts)], [{"cmd": "stop"}])

    def test_the_echo_of_a_prompt_is_still_dropped(self):
        run(self.life, standing(), 0.3, [{"type": "say", "text": "小白连网", "addressed": True, "named": True}])
        run(self.life, standing(), 11.0)
        acts = run(self.life, standing(), 0.3, [{"type": "say", "text": "放在我脸前面二三十厘米，保持一两秒", "addressed": True}])
        self.assertEqual(scans(acts), [])
        self.assertEqual(kinds(acts, "ask"), [])

    def test_after_joining_it_reads_out_its_address(self):
        run(self.life, standing(), 0.3, [{"type": "say", "text": "小白连网", "addressed": True, "named": True}])
        run(self.life, standing(), 12.0)
        acts = run(self.life, standing(), 0.3, [{"type": "qr", "result": "joined", "ssid": "我的手机"}])
        self.assertEqual([(a.args["clip"], a.args["then"]) for a in kinds(acts, "voice")], [("joined", "address")])

    def test_asked_its_address_it_reads_it_out(self):
        acts = run(self.life, standing(), 0.3, [{"type": "say", "text": "小白你的网址是多少", "addressed": True,
                                                  "named": True}])
        self.assertEqual([a.args["clip"] for a in kinds(acts, "voice")], ["address"])
        self.assertEqual(scans(acts), [], "no scan")
        self.assertEqual(kinds(acts, "ask"), [], "no brain")

    def test_reading_its_address_it_does_not_hear_itself_ask_again(self):
        # 2026-10-17: "网页地址是" came back through the mic and started it over, three times; and the
        # whole address, after, closed the conversation with the "88" in "<robot-ip>"
        run(self.life, standing(), 0.3, [{"type": "say", "text": "小白你的网址是多少", "addressed": True, "named": True}])
        for text in ("网页地址是。", "我的网页地址是。"):
            run(self.life, standing(), 2.5)
            acts = run(self.life, standing(), 0.3, [{"type": "say", "text": text, "addressed": True}])
            self.assertEqual(kinds(acts, "voice"), [], text)
        run(self.life, standing(), 10.0)
        acts = run(self.life, standing(), 0.3, [{"type": "say", "text": "我的网页地址是<robot-ip>，冒号8088，在手机浏览器里打开就行。",
                                                  "addressed": True}])
        self.assertEqual(kinds(acts, "voice"), [])
        self.assertTrue(self.life.in_convo(), "its own 88 is no goodbye")

    def test_no_command_from_its_own_voice(self):
        self.life._voice_heard("joined", 0.0)
        acts = run(self.life, standing(), 0.5, [{"type": "say", "text": "去睡觉吧", "addressed": True}])
        self.assertEqual(kinds(acts, "rest"), [], "unnamed, while a clip plays: the clip")
        acts = run(self.life, standing(), 0.5, [{"type": "say", "text": "小白去睡觉吧", "addressed": True, "named": True}])
        self.assertTrue(kinds(acts, "rest"), "with its name it is someone")

    def test_louder_and_quieter_and_the_page(self):
        for text, args in (("小白大声点", {"step": 15}), ("小白小声一点", {"step": -15}),
                           ("小白音量调到一半", {"set": 50}), ("小白最大声", {"set": 100})):
            run(self.life, standing(), 3.0)
            acts = run(self.life, standing(), 0.3, [{"type": "say", "text": text, "addressed": True, "named": True}])
            self.assertEqual([a.args for a in kinds(acts, "volume")], [args], text)
            self.assertEqual([a.args["tag"] for a in kinds(acts, "sound")], ["chirp"], "heard at the new loudness")
        run(self.life, standing(), 3.0)
        acts = run(self.life, standing(), 0.3, [{"type": "volume", "set": 35}])
        self.assertEqual([a.args for a in kinds(acts, "volume")], [{"set": 35}])

    def test_the_ways_people_ask_for_it(self):
        from core import intent_of
        # 2026-10-17, from the robot's log
        for text, want in (("声音再大一点。", "louder"), ("声音再小一点。", "quieter"), ("大声点。", "louder"),
                           ("声音太小了，声音大一点。", "louder"), ("声音太大了，小声点", "quieter"),
                           ("太吵了", "quieter"), ("我听不清", "louder"), ("音量调高", "louder"),
                           ("小声一些", "quieter"), ("再大声点", "louder"), ("最大声", "vol_max"),
                           ("你的声音好大吗？", None), ("你真牛。", None), ("小一点的那个", None)):
            self.assertEqual(intent_of(text), want, text)

    def test_asked_to_look_it_looks_and_waits_for_the_answer(self):
        # voiced takes the name out of the text ("小白你看这是什么" -> a call and "你看这是什么")
        acts = run(self.life, standing(), 0.3, [{"type": "say", "text": "你看这是什么", "addressed": True,
                                                  "named": True}])
        self.assertEqual([a.args["text"] for a in kinds(acts, "look")], ["你看这是什么"])
        self.assertEqual(kinds(acts, "ask"), [], "the look asks the brain itself, with the photo")
        self.assertTrue(self.life.snapshot(standing())["asking"])
        acts = run(self.life, standing(), 0.3, [{"type": "reply", "emotion": "开心", "say": "杯子！", "pinyin": "bei1 zi5",
                                                  "pre": "chirp", "gesture": "nod", "action": "none",
                                                  "meaning": "是一个红杯子。", "via": "eyes", "photo": "123"}])
        d = self.life.snapshot(standing())["dialog"][-1]
        self.assertEqual((d["meaning"], d["photo"]), ("是一个红杯子。", "123"))

    def test_no_look_while_walking(self):
        walking = Body(policy="walk", temps_c=[40.0] * 15, battery_pct=80.0, twist=(0.2, 0.0, 0.0))
        acts = run(self.life, walking, 0.3, [{"type": "say", "text": "小白你看看这个", "addressed": True, "named": True}])
        self.assertEqual(kinds(acts, "look"), [])

    def test_its_name_in_its_own_duck_speak_is_no_call(self):
        run(self.life, standing(), 0.3, [{"type": "reply", "emotion": "开心", "say": "小白在这！", "pinyin": "xiao3 bai2 zai4 zhe4",
                                          "pre": "none", "gesture": "none", "action": "none"}])
        acts = run(self.life, standing(), 0.3, [{"type": "call", "text": "小白"}])
        self.assertEqual([a.args for a in kinds(acts, "speak") if a.args.get("pinyin") == ""], [], "not hushed")

    def test_cancel_words_are_only_for_a_scan(self):
        acts = run(self.life, standing(), 0.3, [{"type": "say", "text": "算了", "addressed": True, "named": True}])
        self.assertEqual(scans(acts), [])
        self.assertEqual(kinds(acts, "voice"), [])

    def test_no_clip_says_its_name(self):
        import prompts
        for key, text in prompts.PROMPTS.items():
            self.assertNotIn("小白", text, key)

    def test_a_question_about_the_internet_is_no_scan(self):
        acts = run(self.life, standing(), 0.3, [{"type": "say", "text": "小白你能上网吗", "addressed": True, "named": True}])
        self.assertEqual(scans(acts), [])

    def test_not_while_walking(self):
        walking = Body(policy="walk", temps_c=[40.0] * 15, battery_pct=80.0, twist=(0.2, 0.0, 0.0))
        acts = run(self.life, walking, 0.3, [{"type": "say", "text": "小白连网", "addressed": True, "named": True}])
        self.assertEqual(scans(acts), [])

    def test_the_dialog_keeps_both_sides(self):
        run(self.life, standing(), 1.5, [{"type": "call", "yaw": 0.0}])
        run(self.life, standing(), 0.3, [{"type": "say", "text": "你好呀", "addressed": True, "named": False}])
        run(self.life, standing(), 0.3, [{"type": "reply", "emotion": "开心", "say": "嘎嘎", "pinyin": "ga1 ga1",
                                          "pre": "none", "gesture": "nod", "body": "none", "action": "none",
                                          "meaning": "你好！", "via": "small"}])
        d = self.life.snapshot(standing())["dialog"]
        self.assertEqual([(x["who"], x["text"]) for x in d[-2:]], [("you", "你好呀"), ("duck", "嘎嘎")])
        self.assertEqual(d[-1]["via"], "small")


class PersonTest(unittest.TestCase):
    """Someone coming close (visiond's ToF watch): greeted when awake, only noted otherwise."""

    def test_greeted_when_awake_not_when_off_or_resting(self):
        life = Life(Config(deep_ok=True), random.Random(1))
        acts = run(life, Body(policy="held"), 1.0, [{"type": "person", "yaw": 0.0, "dist": 0.5, "via": "tof"}])
        self.assertEqual(kinds(acts, "sound"), [], "off: nothing")
        run(life, standing(), 6)
        acts = run(life, standing(), 0.3, [{"type": "person", "yaw": 0.0, "dist": 0.5, "via": "tof"}])
        self.assertTrue(any(a.args.get("tag") == "greet" for a in kinds(acts, "sound")))
        self.assertEqual(life.state, "ATTENDING")
        resting = Body(policy="sit", temps_c=[40.0] * 15, battery_pct=80.0, rest={"phase": "resting", "deep": False})
        life2 = Life(Config(deep_ok=True), random.Random(1))
        run(life2, resting, 2.0)
        acts = run(life2, resting, 0.3, [{"type": "person", "yaw": 0.0, "dist": 0.5, "via": "tof"}])
        self.assertEqual([a for a in kinds(acts, "sound") if a.args.get("tag") == "greet"], [])


class JevEventsTest(unittest.TestCase):
    """What lifed does with Jev's verdicts (framework §4)."""

    def setUp(self):
        self.life = Life(Config(deep_ok=True), random.Random(1))
        run(self.life, standing(), 6)

    def ev(self, *events, secs=0.3):
        return run(self.life, standing(), secs, list(events))

    def test_the_ask_says_whether_the_name_was_said(self):
        acts = self.ev({"type": "say", "text": "你在干嘛", "addressed": True, "named": True})
        self.assertTrue(kinds(acts, "ask")[0].args["named"])

    def test_an_ignored_utterance_ends_the_wait_quietly(self):
        self.ev({"type": "call", "yaw": 0.0}, secs=1.5)
        self.ev({"type": "say", "text": "申请吧", "addressed": True, "named": False})
        self.assertTrue(self.life.snapshot(standing())["asking"])
        acts = self.ev({"type": "ignored", "text": "申请吧", "why": "not for the duck (0.12)"}, secs=3.0)
        self.assertFalse(self.life.snapshot(standing())["asking"])
        self.assertEqual(kinds(acts, "speak"), [], "no thinking hum, no answer")

    def test_the_judgment_shows_next_to_what_was_said(self):
        self.ev({"type": "say", "text": "你在干嘛", "addressed": True, "named": True})
        self.ev({"type": "judged", "text": "你在干嘛", "summary": {"for_duck": 0.93, "intent": "chat"}})
        you = [d for d in self.life.snapshot(standing())["dialog"] if d["who"] == "you"][-1]
        self.assertEqual(you["jev"]["for_duck"], 0.93)

    def test_near_is_a_glance_and_jev_decides_the_rest(self):
        acts = self.ev({"type": "near", "dist": 0.5, "via": "tof"})
        self.assertTrue(kinds(acts, "gaze"))
        self.assertEqual(kinds(acts, "sound"), [], "no hello yet")
        acts = self.ev({"type": "person", "via": "jev", "present": False, "activity": "unclear", "dist": 0.5, "yaw": 0.0})
        self.assertEqual(kinds(acts, "sound"), [], "a wall: nothing")
        acts = self.ev({"type": "person", "via": "jev", "present": True, "activity": "passing", "dist": 0.8, "yaw": 0.3})
        self.assertEqual(kinds(acts, "sound"), [], "someone passing: a look")
        self.assertEqual(kinds(acts, "gaze")[0].args["yaw"], 0.3)
        acts = self.ev({"type": "person", "via": "jev", "present": True, "activity": "waving", "dist": 0.8, "yaw": 0.0})
        self.assertTrue(any(a.args.get("tag") == "greet" for a in kinds(acts, "sound")))

    def test_a_thumbs_up_is_a_happy_chirp(self):
        acts = self.ev({"type": "person", "via": "jev", "present": True, "activity": "thumbs_up", "dist": 0.8, "yaw": 0.0})
        self.assertTrue(any(a.args.get("tag") == "chirp" for a in kinds(acts, "sound")))

    def test_a_command_jev_caught(self):
        acts = self.ev({"type": "command", "name": "rest", "text": "你去歇着吧", "via": "jev"})
        self.assertEqual([a.args.get("deep") for a in kinds(acts, "rest")], [True])
        you = [d for d in self.life.snapshot(standing())["dialog"] if d["who"] == "you"][-1]
        self.assertEqual(you["command"], "rest")


class RestartTest(unittest.TestCase):
    def test_restarted_on_a_resting_duck_it_knows_it_is_asleep(self):
        life = Life(Config(deep_ok=True), random.Random(1))
        deep = Body(policy="sit", temps_c=[40.0] * 15, battery_pct=80.0, rest={"phase": "resting", "deep": True})
        self.assertEqual(run(life, deep, 2.0), [])
        self.assertEqual(life.state, "SLEEPING")
        life = Life(Config(deep_ok=True), random.Random(1))
        run(life, Body(policy="sit", temps_c=[40.0] * 15, battery_pct=80.0, rest={"phase": "resting", "deep": False}), 2.0)
        self.assertEqual(life.state, "RESTING")


class ConversationModeTest(unittest.TestCase):
    """The name opens a conversation; inside it what is said goes to the brain without the name,
    until 30 s pass after the duck's last answer, or a goodbye (the user, 2026-10-16)."""

    def setUp(self):
        self.life = Life(Config(deep_ok=True, convo_s=30.0), random.Random(1))
        run(self.life, standing(), 6)

    def call(self):
        """Its name alone: it answers "嗯？" and does not hear the next 1.3 s (its own voice); the next
        sentence reaches lifed 1-2 s later on the robot (VAD tail + ASR)."""
        return run(self.life, standing(), 1.5, [{"type": "call", "yaw": 0.0}])

    def say(self, text, named=False, secs=0.2):
        return run(self.life, standing(), secs, [{"type": "say", "text": text, "addressed": named, "named": named}])

    def answer(self, secs=0.2):
        return run(self.life, standing(), secs, [{"type": "reply", "emotion": "开心", "say": "嘎嘎", "pinyin": "ga1 ga1",
                                                   "pre": "chirp", "gesture": "nod", "body": "none", "action": "none"}])

    def test_without_its_name_nothing_goes_to_the_brain(self):
        acts = self.say("今天天气怎么样")
        self.assertEqual(kinds(acts, "ask"), [])

    def test_after_its_name_the_talk_continues_without_it(self):
        self.call()
        acts = self.say("今天天气怎么样")
        self.assertEqual(len(kinds(acts, "ask")), 1)
        session = kinds(acts, "ask")[0].args["session"]
        self.answer(secs=20)                 # it answers; you think for a while
        acts = self.say("那我们出去玩吧")
        self.assertEqual(len(kinds(acts, "ask")), 1, "20 s after its answer the conversation is still on")
        self.assertEqual(kinds(acts, "ask")[0].args["session"], session, "one brain session per conversation")

    def test_it_closes_30_s_after_its_last_answer(self):
        self.call()
        self.say("你好呀小白", named=True)
        acts = self.answer(secs=35)
        self.assertFalse(self.life.in_convo())
        self.assertTrue(any(a.args.get("tag") == "coo" for a in kinds(acts, "sound")), "a soft coo as it lets go")
        acts = self.say("还在吗")
        self.assertEqual(kinds(acts, "ask"), [], "closed: it needs its name again")
        acts = self.say("小白还在吗", named=True)
        self.assertEqual(len(kinds(acts, "ask")), 1)

    def test_the_brain_can_end_it_after_its_goodbye(self):
        self.call()
        self.say("那我去忙啦")
        acts = run(self.life, standing(), 0.2, [{"type": "reply", "emotion": "难过", "say": "好吧", "pinyin": "hao3 ba5",
                                                 "pre": "coo", "gesture": "droop", "body": "none", "action": "none",
                                                 "end": True}])
        self.assertTrue(self.life.in_convo(), "it finishes saying bye first")
        acts = run(self.life, standing(), 5.0)
        self.assertFalse(self.life.in_convo())
        self.assertEqual([a for a in kinds(acts, "sound") if a.args.get("tag") == "coo"], [], "no second goodbye")
        self.assertEqual(kinds(self.say("还有一件事"), "ask"), [], "closed: the name again")

    def test_a_goodbye_closes_it_at_once(self):
        self.call()
        self.say("今天好开心", secs=3.0)
        acts = self.say("拜拜")
        self.assertFalse(self.life.in_convo())
        self.assertEqual(kinds(acts, "ask"), [])
        self.assertTrue(any(a.args.get("tag") == "coo" for a in kinds(acts, "sound")))
        self.assertEqual(kinds(self.say("你好笨"), "ask"), [])

    def test_its_own_voice_is_not_a_question(self):
        self.call()
        self.say("你开心吗")
        self.answer(secs=0.3)
        acts = self.say("啊")            # its own duck-speak, back through the mic
        self.assertEqual(kinds(acts, "ask"), [])

    def test_calling_it_while_it_talks_stops_it(self):
        self.call()
        self.say("唱首歌吧")
        self.answer(secs=0.3)
        acts = self.call()
        self.assertIn("", [a.args.get("pinyin") for a in kinds(acts, "speak")], "an empty speak silences it")
        acts = self.say("别唱了")
        self.assertEqual(len(kinds(acts, "ask")), 1, "and it listens straight away")

    def test_asleep_it_answers_but_does_not_get_up(self):
        self.say("晚安", named=True)
        self.assertEqual(self.life.state, "SLEEPING")
        sleeping = Body(policy="sit", temps_c=[40.0] * 15, battery_pct=80.0,
                        rest={"phase": "resting", "deep": True})
        run(self.life, sleeping, 2.0)
        acts = run(self.life, sleeping, 0.2, [{"type": "say", "text": "小白你睡着了吗", "addressed": True, "named": True}])
        self.assertEqual(len(kinds(acts, "ask")), 1)
        self.assertEqual(kinds(acts, "wake"), [])
        self.assertEqual(self.life.state, "SLEEPING")




class MissionTest(unittest.TestCase):
    """A spoken route: read back, "开始", reports, "停" / "继续" / "算了" (lifed runs it: mission.py)."""

    def setUp(self):
        self.life = Life(Config(missions="dry"), random.Random(1))
        run(self.life, standing(), 6)

    def say(self, text, named=False, addressed=True):
        run(self.life, standing(), 15.0)            # past any clip
        return run(self.life, standing(), 0.3, [{"type": "say", "text": text, "addressed": addressed, "named": named}])

    def test_a_route_is_read_back_and_waits_for_go(self):
        acts = self.say("往前走一米，再左转，再往前走半米，最后坐下", named=True)
        m = [a.args for a in kinds(acts, "mission")]
        self.assertEqual([x["cmd"] for x in m], ["propose"])
        self.assertEqual([s.words() for s in m[0]["steps"]], ["往前走 1 米", "左转 90°", "往前走 0.5 米", "坐下"])
        self.assertEqual([a.args["clip"] for a in kinds(acts, "voice")], ["mission_ready_dry"])
        self.assertEqual(kinds(acts, "skill"), [], "the 坐下 in it is the route's, not now")
        acts = self.say("开始")
        self.assertEqual([a.args["cmd"] for a in kinds(acts, "mission")], ["start"])

    def test_go_said_right_after_the_prompt_counts_and_its_echo_does_not(self):
        # 2026-10-17: "开始" a second after "……说开始就出发" was dropped as the clip's echo
        self.say("往前走一米，再左转，再往前走半米，最后坐下", named=True)
        until = self.life.voice_until
        run(self.life, standing(), until - self.life.t + 0.1)
        acts = run(self.life, standing(), 0.1, [{"type": "say", "text": "说开始就出发", "addressed": True}])
        self.assertEqual(kinds(acts, "mission"), [], "its own words, just after the clip")
        acts = run(self.life, standing(), 0.1, [{"type": "say", "text": "出发", "addressed": True}])
        self.assertEqual(kinds(acts, "mission"), [], "the clip's last word, too soon to be an answer")
        run(self.life, standing(), 0.5)
        acts = run(self.life, standing(), 0.1, [{"type": "say", "text": "开始", "addressed": True}])
        self.assertEqual([a.args["cmd"] for a in kinds(acts, "mission")], ["start"])

    def test_halt_needs_no_name_while_it_runs_and_continue_goes_on(self):
        self.say("往前走一米", named=True)
        self.say("开始")
        acts = self.say("停", addressed=False)
        self.assertEqual([a.args for a in kinds(acts, "mission")], [{"cmd": "pause", "why": "told"}])
        acts = self.say("继续")
        self.assertEqual([a.args["cmd"] for a in kinds(acts, "mission")], ["resume"])
        acts = self.say("算了")
        self.assertEqual([a.args["cmd"] for a in kinds(acts, "mission")], ["cancel"])
        acts = self.say("开始")
        self.assertEqual(kinds(acts, "mission"), [], "nothing left to start")

    def test_unclear_routes_are_asked_about(self):
        for text, clip in (("往左走半米", "ask_turn_or_side"), ("往前走", "ask_how_far"), ("向左侧移半米", "no_sidestep"),
                           ("往前走五米", "too_far")):
            acts = self.say(text, named=True)
            self.assertEqual([a.args["clip"] for a in kinds(acts, "voice")], [clip], text)
            self.assertEqual(kinds(acts, "mission"), [], text)

    def test_reports(self):
        self.say("往前走一米", named=True)
        self.say("开始")
        run(self.life, standing(), 0.3, [{"type": "mission", "what": "step_done", "i": 0, "words": "往前走 1 米", "note": "0.98 m"}])
        self.assertIn("✓ 往前走 1 米（0.98 m）", [d.get("meaning") for d in self.life.snapshot(standing())["dialog"]])
        acts = run(self.life, standing(), 0.3, [{"type": "mission", "what": "done", "words": "往前走 1 米"}])
        self.assertEqual([a.args["clip"] for a in kinds(acts, "voice")], ["mission_done"])
        acts = self.say("开始")
        self.assertEqual(kinds(acts, "mission"), [])

    def test_an_obstacle_report(self):
        self.say("往前走一米", named=True)
        self.say("开始")
        acts = run(self.life, standing(), 0.3, [{"type": "mission", "what": "paused", "why": "obstacle"}])
        self.assertEqual([a.args["clip"] for a in kinds(acts, "voice")], ["mission_obstacle"])
        acts = self.say("继续")
        self.assertEqual([a.args["cmd"] for a in kinds(acts, "mission")], ["resume"])

    def test_on_the_robot_it_must_be_standing_to_set_off(self):
        life = Life(Config(missions="real"), random.Random(1))
        sitting = Body(policy="sit", temps_c=[40.0] * 15, battery_pct=80.0)
        run(life, sitting, 6)
        acts = run(life, sitting, 0.3, [{"type": "say", "text": "往前走一米", "addressed": True, "named": True}])
        self.assertEqual([a.args["clip"] for a in kinds(acts, "voice")], ["mission_need_stand"])
        self.assertEqual(kinds(acts, "mission"), [])
        run(life, standing(), 15)
        acts = run(life, standing(), 0.3, [{"type": "say", "text": "往前走一米", "addressed": True, "named": True}])
        self.assertEqual([a.args["clip"] for a in kinds(acts, "voice")], ["mission_ready"])

    def test_off_no_routes(self):
        life = Life(Config(missions="off"), random.Random(1))
        run(life, standing(), 6)
        acts = run(life, standing(), 0.3, [{"type": "say", "text": "往前走一米", "addressed": True, "named": True}])
        self.assertEqual(kinds(acts, "mission"), [])




class HeardDirectionTest(unittest.TestCase):
    """A call's direction (voiced, the two head mics) turns the head - in the body frame."""

    def setUp(self):
        self.life = Life(Config(), random.Random(1))
        run(self.life, standing(), 6)

    def gaze_after_call(self, ev, body=None):
        acts = run(self.life, body or standing(), 0.3, [dict({"type": "call", "text": "小白"}, **ev)])
        return [a.args["yaw"] for a in kinds(acts, "gaze")]

    def test_a_call_from_the_left_turns_the_head_there(self):
        self.assertAlmostEqual(self.gaze_after_call({"yaw": 0.6, "dir_ok": True})[0], 0.6, places=3)

    def test_the_head_already_turned_is_added(self):
        turned = Body(policy="stand", temps_c=[40.0] * 15, battery_pct=80.0, head_cmd=(0.0, 0.0, 0.5, 0.0))
        run(self.life, turned, 1.0)
        self.assertAlmostEqual(self.gaze_after_call({"yaw": -0.5, "dir_ok": True}, turned)[0], 0.0, places=3,
                               msg="heard 0.5 rad right of a head turned 0.5 left: straight ahead")

    def test_an_unsure_direction_looks_ahead(self):
        self.assertAlmostEqual(self.gaze_after_call({"yaw": 0.9, "dir_ok": False})[0], 0.0, places=3)


class EyesTest(unittest.TestCase):
    """Vision-guided attention (attention.py): visiond's watch kept up, faces and motion as targets, the gaze
    on the focus - aimed once, re-aimed only when it moves."""

    def setUp(self):
        self.life = Life(Config(), random.Random(1))
        run(self.life, standing(), 6)

    @staticmethod
    def head_at(yaw, pitch_up=0.0):
        return Body(policy="stand", temps_c=[40.0] * 15, battery_pct=80.0, head_cmd=(0.0, -pitch_up, yaw, 0.0))

    def watches(self, acts):
        return [a.args for a in kinds(acts, "vision") if a.args.get("cmd") == "watch"]

    def test_awake_the_eyes_watch_slowly_and_keep_watching(self):
        w = self.watches(run(self.life, standing(), 10))
        self.assertTrue(2 <= len(w) <= 4, w)
        self.assertTrue(all(x["hz"] == 2.0 for x in w))

    def test_no_watch_walking_or_asleep(self):
        walking = Body(policy="walk", temps_c=[40.0] * 15, battery_pct=80.0, twist=(0.2, 0.0, 0.0))
        run(self.life, walking, 1)
        self.assertEqual(self.watches(run(self.life, walking, 6)), [])

    def test_a_call_looks_there_a_little_up_and_watches_fast(self):
        acts = run(self.life, standing(), 0.5, [{"type": "call", "text": "小白", "yaw": 0.6, "dir_ok": True}])
        last = kinds(acts, "gaze")[-1].args
        self.assertAlmostEqual(last["yaw"], 0.6, places=2)
        self.assertGreater(last["pitch"], 0.2)
        self.assertIn(5.0, [x["hz"] for x in self.watches(acts)])

    def test_the_face_found_there_is_what_it_looks_at(self):
        run(self.life, standing(), 0.3, [{"type": "call", "text": "小白", "yaw": 0.6, "dir_ok": True}])
        turned = self.head_at(0.6, 0.25)
        run(self.life, turned, 1.0)
        acts = run(self.life, turned, 0.5, [{"type": "face", "yaw_cam": -0.1, "pitch_cam": 0.1, "size": 0.08}])
        last = kinds(acts, "gaze")[-1].args
        self.assertAlmostEqual(last["yaw"], 0.5, delta=0.02)
        self.assertAlmostEqual(last["pitch"], 0.35, delta=0.02)
        self.assertEqual(self.life.att.focus.kind, "person")

    def test_a_face_held_still_is_not_chased(self):
        turned = self.head_at(0.3, 0.2)
        run(self.life, turned, 1.0)
        gazes = []
        for _ in range(15):                            # 3 s of the same face, 5 a second
            acts = run(self.life, turned, 0.2, [{"type": "face", "yaw_cam": 0.02, "pitch_cam": 0.0, "size": 0.1}])
            gazes += kinds(acts, "gaze")
        self.assertTrue(1 <= len(gazes) <= 3, [g.args for g in gazes])

    def test_someone_close_looking_at_it_gets_its_attention(self):
        acts = run(self.life, standing(), 0.3, [{"type": "face", "yaw_cam": 0.2, "pitch_cam": 0.1, "size": 0.12}])
        self.assertEqual(self.life.state, "ATTENDING")
        self.assertAlmostEqual(kinds(acts, "gaze")[-1].args["yaw"], 0.2, places=2)

    def test_motion_counts_only_from_a_still_head(self):
        run(self.life, self.head_at(0.0), 0.2)
        run(self.life, self.head_at(0.4), 0.1)            # the head just turned
        run(self.life, self.head_at(0.4), 0.1, [{"type": "motion", "yaw_cam": -0.3, "pitch_cam": 0.0, "size": 0.02}])
        self.assertFalse([t for t in self.life.att.targets.values() if t.kind == "motion"])
        still = self.head_at(0.4)
        run(self.life, still, 1.0)
        acts = run(self.life, still, 0.3, [{"type": "motion", "yaw_cam": -0.3, "pitch_cam": 0.0, "size": 0.02}])
        self.assertTrue([t for t in self.life.att.targets.values() if t.kind == "motion"])
        self.assertAlmostEqual(kinds(acts, "gaze")[-1].args["yaw"], 0.1, places=2)

    def test_what_is_shown_is_looked_at(self):
        turned = self.head_at(0.3, 0.2)
        run(self.life, turned, 1.0)
        run(self.life, turned, 0.3, [{"type": "say", "text": "小白你看这是什么", "addressed": True, "named": True}])
        self.assertEqual(self.life.att.focus.kind, "object")
        self.assertAlmostEqual(self.life.att.focus.yaw, 0.3, places=2)


class FallenTest(unittest.TestCase):
    def test_a_re_pose_after_a_deep_rest_is_no_fall(self):
        life = Life(Config(), random.Random(1))
        run(life, standing(), 3)
        repose = Body(policy="limp_pose", temps_c=[40.0] * 15, battery_pct=80.0, gravity=(-0.6, 0.0, -0.8))
        run(life, repose, 0.5)
        self.assertNotEqual(life.state, "FALLEN")

    def test_tipped_over_it_is_a_fall_and_sitting_up_again_ends_it(self):
        life = Life(Config(), random.Random(1))
        run(life, standing(), 3)
        tipped = Body(policy="limp_pose", temps_c=[40.0] * 15, battery_pct=80.0, gravity=(0.0, -0.95, -0.2))
        run(life, tipped, 0.5)
        self.assertEqual(life.state, "FALLEN")
        sitting = Body(policy="sit", temps_c=[40.0] * 15, battery_pct=80.0)
        run(life, sitting, 3.0)
        self.assertEqual(life.state, "IDLE", "upright again, sitting")


if __name__ == "__main__":
    unittest.main()
