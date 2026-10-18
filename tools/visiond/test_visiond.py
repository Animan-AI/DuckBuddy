# -*- coding: utf-8 -*-
"""python -m unittest tools/visiond/test_visiond.py"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from visiond import AirWatch, match_bytes, match_network, parse_wifi_qr, split_terse, ssid_on_air  # noqa: E402


class WifiQrTest(unittest.TestCase):
    def test_a_phone_hotspot_code(self):
        self.assertEqual(parse_wifi_qr("WIFI:T:WPA;S:小米手机;P:12345678;;"),
                         {"ssid": "小米手机", "password": "12345678", "security": "WPA", "hidden": False})

    def test_fields_in_any_order_with_escapes_and_hidden(self):
        net = parse_wifi_qr(r"WIFI:S:my\;net\:5G;H:true;P:p\\a\;ss;T:SAE;;")
        self.assertEqual(net, {"ssid": "my;net:5G", "password": "p\\a;ss", "security": "SAE", "hidden": True})

    def test_open_networks(self):
        self.assertEqual(parse_wifi_qr("WIFI:T:nopass;S:Cafe;;")["security"], "nopass")
        self.assertEqual(parse_wifi_qr("WIFI:S:Cafe;;")["security"], "nopass")

    def test_not_wifi(self):
        self.assertIsNone(parse_wifi_qr("https://example.com"))
        self.assertIsNone(parse_wifi_qr("WIFI:T:WPA;P:x;;"), "no network name")
        self.assertIsNone(parse_wifi_qr(""))


class NetworkMatchTest(unittest.TestCase):
    def test_a_name_longer_than_32_bytes_is_cut_on_the_air(self):
        # 2026-10-17: the user's hotspot, 33 bytes in UTF-8
        self.assertEqual(ssid_on_air("风一样的男子，简称风子"), "风一样的男子，简称风")
        self.assertEqual(match_network("风一样的男子，简称风子", ["Home-7k2Q", "风一样的男子，简称风"]), "风一样的男子，简称风")

    def test_exact_names_and_no_false_matches(self):
        self.assertEqual(match_network("Home-7k2Q", ["Home-7k2Q-5G", "Home-7k2Q"]), "Home-7k2Q")
        self.assertIsNone(match_network("Home-7k2Q", ["Home-7k2Q-5G"]), "a short name must match exactly")
        self.assertIsNone(match_network("风一样的男子，简称风子", ["风一样"]), "a short prefix is another network")
        self.assertIsNone(match_network("我的手机", []))

    def test_raw_ssids_cut_mid_character(self):
        full = "风一样的男子，简称风子".encode("utf-8")
        self.assertEqual(match_bytes(full, [b"Home", full[:32]]), full[:32], "32 bytes, half a 子 at the end")
        self.assertEqual(match_bytes(full, [full[:30]]), full[:30])
        self.assertIsNone(match_bytes(b"Home-7k2Q", [b"Home-7k2Q-5G"]))

    def test_the_air_watch_remembers_what_it_saw(self):
        import time
        air = AirWatch()
        full = "风一样的男子，简称风子".encode("utf-8")
        air.seen = {full[:32]: ("AA:BB:CC:DD:EE:FF", time.monotonic() - 40), b"Home-7k2Q": ("11:22", time.monotonic())}
        got = air.find("风一样的男子，简称风子")
        self.assertEqual(got[:2], (full[:32], "AA:BB:CC:DD:EE:FF"))
        self.assertGreater(got[2], 30)
        self.assertIsNone(air.find("风一样的男子，简称风子", max_age_s=10))

    def test_terse_nmcli_lines(self):
        self.assertEqual(split_terse(r"a\:b:802-11-wireless"), ["a:b", "802-11-wireless"])
        self.assertEqual(split_terse(r"AA\:BB:x\\y"), ["AA:BB", "x\\y"])


if __name__ == "__main__":
    unittest.main()
