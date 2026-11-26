"""Regression for saving wide zero drafts from unloaded HLS servos."""

import importlib.util
import tempfile
import unittest
from contextlib import nullcontext
from pathlib import Path
from unittest.mock import patch

import agent


ALIGN_SERVER = Path(__file__).resolve().parents[1] / "joint_align" / "server.py"
spec = importlib.util.spec_from_file_location("joint_align_server", ALIGN_SERVER)
joint_align = importlib.util.module_from_spec(spec)
spec.loader.exec_module(joint_align)

IDS = list(range(1, 16))
SIGNS = [1] * 15
ZEROS = [0] * 15
WIDE_OFFSETS = {4: -2316, 5: 2093, 8: 2111, 12: 2075, 14: 2275}
DRAFT = [WIDE_OFFSETS.get(sid, 0) for sid in IDS]


class UnloadedBus(agent.Bus):
    def __init__(self, model, bad_id=None):
        self.model = model
        self.bad_id = bad_id
        self.centres = dict.fromkeys(IDS, 17)
        self.locked = dict.fromkeys(IDS, 1)
        self.reset_ids = set()

    def r8(self, sid, address):
        return self.locked[sid] if address == agent.LOCK else 0

    def r16(self, sid, address):
        return self.model if address == 3 else agent.enc(self.centres[sid])

    def w8(self, sid, address, value):
        if address != agent.LOCK:
            raise AssertionError("saving calibration may not power or move a servo")
        self.locked[sid] = value
        return 0

    def w16(self, sid, address, value):
        if address != 31 or self.locked[sid]:
            raise AssertionError("expected an unlocked EEPROM centre write")
        self.centres[sid] = agent.smag(value)
        return 0

    def pos(self, sid):
        value = 2048 + WIDE_OFFSETS.get(sid, 0) + 17 - self.centres[sid]
        return value % 4096 if sid in self.reset_ids else value

    def reset_turns(self, sid):
        self.reset_ids.add(sid)

    def calibrate_position(self, sid, position):
        self.centres[sid] += self.pos(sid) - position
        if sid == self.bad_id:
            self.centres[sid] += 100


class ZeroCalibrationTests(unittest.TestCase):
    def save_draft(self, path, bus):
        calib = joint_align.Calib("test", str(path), "", "")
        calib.loaded = (SIGNS[:], ZEROS[:])
        calib.signs, calib.offsets, calib.ids = SIGNS[:], DRAFT[:], IDS
        with (
            patch.object(agent.Ctx, "cfg_path", str(path)),
            patch.object(agent.Ctx, "config", {}),
            patch.object(agent, "robotd_pids", return_value=[]),
            patch.object(agent, "bus_session", return_value=nullcontext(bus)),
            patch.object(agent.os, "sync", create=True),
            patch.object(agent.time, "sleep"),
        ):
            return calib.save(save_zeros=agent.op_center_calibration)

    def test_wide_draft_centres_both_models(self):
        for model in (agent.HL2915_MODEL, agent.HD1910_MODEL):
            with self.subTest(model=model), tempfile.TemporaryDirectory() as temp:
                path = Path(temp) / "duck.toml"
                path.write_text(f"[bus]\njoint_ids = {IDS}\njoint_signs = {SIGNS}\njoint_offsets = {ZEROS}\n")
                bus = UnloadedBus(model)
                ok, message = self.save_draft(path, bus)
                self.assertTrue(ok, message)
                self.assertEqual(bus.reset_ids, set(WIDE_OFFSETS))
                self.assertTrue(all(bus.pos(sid) == 2048 for sid in WIDE_OFFSETS))
                self.assertEqual(agent.load_config(str(path))[0]["bus"]["joint_offsets"], ZEROS)

    def test_bad_readback_restores_centres_and_keeps_config(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "duck.toml"
            original = f"[bus]\njoint_ids = {IDS}\njoint_signs = {SIGNS}\njoint_offsets = {ZEROS}\n"
            path.write_text(original)
            bus = UnloadedBus(agent.HD1910_MODEL, bad_id=8)
            with self.assertRaisesRegex(agent.BusError, "未通过读回"):
                self.save_draft(path, bus)
            self.assertEqual(path.read_text(), original)
            self.assertEqual(set(bus.centres.values()), {17})
            self.assertEqual(set(bus.locked.values()), {1})
