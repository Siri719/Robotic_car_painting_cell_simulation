"""Camera presets (GUI debug camera) + off-screen rendering for screenshots."""
import struct
import zlib
import numpy as np
import pybullet as p

# name: (distance, yaw_deg, pitch_deg, target)
VIEWS = {
    1: ("OVERALL CELL",      (11.0, 40, -32, (0, 0, 0.8))),
    2: ("ROBOT + LEFT SIDE", (7.0, 150, -22, (0, 1.0, 0.9))),
    3: ("ROBOT + RIGHT SIDE", (7.0, 30, -22, (0, -1.0, 0.9))),
    4: ("ROOF PAINTING",     (5.5, 90, -48, (0, -0.5, 1.4))),
    5: ("SPRAY GUN CLOSE-UP", (1.4, 30, -25, (0, 0, 1))),       # target follows the TCP
    6: ("FINAL RESULT",      (10.0, 215, -28, (0, 0, 0.8))),
}


class CameraManager:
    def __init__(self, gui):
        self.gui, self.current, self.auto = gui, 1, False

    def set(self, k):
        self.current = k
        if self.gui:
            d, yaw, pitch, tgt = VIEWS[k][1]
            p.resetDebugVisualizerCamera(d, yaw, pitch, list(tgt))

    def cycle(self):
        self.set(self.current % 6 + 1)

    def follow(self, tcp):
        if self.gui and self.current == 5:
            d, yaw, pitch, _ = VIEWS[5][1]
            p.resetDebugVisualizerCamera(d, yaw, pitch, list(tcp))

    @staticmethod
    def render(view_idx, w=960, h=600, tcp=None):
        d, yaw, pitch, tgt = VIEWS[view_idx][1]
        if view_idx == 5 and tcp is not None:
            tgt = tuple(tcp)
        vm = p.computeViewMatrixFromYawPitchRoll(list(tgt), d, yaw, pitch, 0, 2)
        pm = p.computeProjectionMatrixFOV(60, w / h, 0.1, 60)
        img = p.getCameraImage(w, h, vm, pm, renderer=p.ER_TINY_RENDERER, shadow=0)
        return np.array(img[2], dtype=np.uint8).reshape(h, w, 4)[:, :, :3]


def save_png(path, rgb):
    """Minimal PNG writer (no PIL needed)."""
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) +
           chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(png)
