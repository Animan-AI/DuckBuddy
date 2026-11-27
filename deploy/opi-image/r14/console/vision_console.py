"""Keep the existing vision RPCs and share the camera with the web console."""

import base64
import subprocess
import time

import visiond


class ConsoleVision(visiond.Visiond):
    def console_frame(self, photo):
        import jevlook
        import numpy as np

        latest = self.latest
        if self.status.get("mode") == "watch" and latest and time.monotonic() - latest[3] < 0.8:
            frame, width, height = latest[:3]
        else:
            if self.status.get("mode") == "scan_qr":
                return dict(ok=False, reason="摄像头正在使用")
            if not self.busy.acquire(timeout=3.0 if photo else 0.0):
                return dict(ok=False, reason="摄像头正在使用")
            try:
                capture = subprocess.run(
                    [self.a.duckcam, "--nv12", "--every", "5", "--frames", "6"],
                    capture_output=True, timeout=5,
                )
                data, offset, frame = capture.stdout, 0, None
                while len(data) - offset >= visiond.HEADER.size:
                    magic, width, height, *_ = visiond.HEADER.unpack_from(data, offset)
                    offset += visiond.HEADER.size
                    size = width * height * 3 // 2
                    if magic != b"DCNV" or not 0 < size <= 16_000_000 or offset + size > len(data):
                        break
                    frame = np.frombuffer(data, dtype=np.uint8, count=size, offset=offset), width, height
                    offset += size
                if frame is None:
                    return dict(ok=False, reason="摄像头没返回完整画面")
                frame, width, height = frame
            except subprocess.TimeoutExpired:
                return dict(ok=False, reason="摄像头取帧超时")
            except OSError:
                return dict(ok=False, reason="摄像头无法打开")
            finally:
                self.busy.release()
        jpeg = jevlook.nv12_to_jpeg(frame, width, height, out_w=1280 if photo else 640,
                                    quality=88 if photo else 72)
        return dict(ok=True, jpeg_b64=base64.b64encode(jpeg).decode())

    def read_setup_qr(self):
        if self.status.get("mode") == "watch":
            self.watch_stop.set()
        if not self.busy.acquire(timeout=5):
            return dict(ok=False, reason="摄像头正在使用")
        try:
            self.status = {"mode": "scan_qr"}
            self.stop.clear()
            return dict(ok=True, text=visiond.scan_qr(self.a.duckcam, 30, self.stop))
        finally:
            self.status = {"mode": "idle"}
            self.busy.release()

    def handle(self, message):
        method = message.get("method")
        if method == "vision.console_frame":
            return self.console_frame(bool((message.get("params") or {}).get("photo")))
        if method == "vision.read_setup_qr":
            return self.read_setup_qr()
        return super().handle(message)


if __name__ == "__main__":
    visiond.Visiond = ConsoleVision
    visiond.main()
