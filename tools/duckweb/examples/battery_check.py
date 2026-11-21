"""Read the battery percentage from DuckBuddy's web status page."""

import argparse
import json
from urllib.request import urlopen


def main():
    parser = argparse.ArgumentParser(description="查看 DuckBuddy 的电量")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--demo", action="store_true", help="用演示数据，不连接鸭子")
    source.add_argument("--url", help="鸭子的网页地址，例如 http://192.168.1.50:8088")
    args = parser.parse_args()

    if args.demo:
        state = {"life": {"battery_pct": 78}}
    else:
        try:
            with urlopen(args.url.rstrip("/") + "/api/state", timeout=3) as response:
                state = json.load(response)
        except (OSError, ValueError):
            parser.exit(1, "读不到鸭子网页。请先用浏览器确认地址能打开。\n")

    life = state.get("life") or {}
    percent = life.get("battery_pct")
    if type(percent) not in (int, float) or not 0 <= percent <= 100:
        parser.exit(1, "网页还没收到电量。请稍等并刷新鸭子网页。\n")

    suffix = "（演示数据）" if args.demo else ""
    print(f"电量 {percent:.0f}%{suffix}")


if __name__ == "__main__":
    main()
