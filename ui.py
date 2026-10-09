"""On-screen information panel (PyBullet debug text, camera-anchored) and keyboard handling."""
import numpy as np
import pybullet as p

KEYS = {ord("1"): "HOME", ord("2"): "LEFT", ord("3"): "RIGHT", ord("4"): "ROOF", ord("5"): "HOOD",
        ord("6"): "TRUNK", ord("a"): "AUTO", ord("s"): "STOP", ord("r"): "RESET", ord("p"): "PAUSE",
        ord("x"): "COLTEST", ord("f"): "FAST", ord("v"): "AUTOCAM", ord("k"): "SHOT", ord("c"): "CAMCYCLE",
        p.B3G_F1: "CAM1", p.B3G_F2: "CAM2", p.B3G_F3: "CAM3", p.B3G_F4: "CAM4", p.B3G_F5: "CAM5",
        p.B3G_F6: "CAM6"}

LEGEND = ["[1] HOME   [2] LEFT SIDE   [3] RIGHT SIDE", "[4] ROOF   [5] HOOD   [6] TRUNK",
          "[A] AUTOMATIC DEMO   [S] STOP   [R] RESET", "[P] PAUSE/RESUME   [X] COLLISION TEST",
          "[F1-F6 / C] CAMERAS   [V] AUTO-CAMERA", "[F] FAST-FORWARD   [K] SCREENSHOT"]
COLORS = {"SAFE": [0.0, 0.5, 0.1], "WARNING / TOO CLOSE": [0.9, 0.5, 0.0], "COLLISION": [0.85, 0.0, 0.0]}


def poll_keys():
    ev = p.getKeyboardEvents()
    return [KEYS[k] for k, s in ev.items() if (s & p.KEY_WAS_TRIGGERED) and k in KEYS]


class HUD:
    PX = 20          # line spacing in pixels

    def __init__(self, gui):
        self.gui, self.ids, self.cache = gui, {}, {}

    def _cam(self):
        info = p.getDebugVisualizerCamera()
        V = np.array(info[2]).reshape(4, 4).T
        P = np.array(info[3]).reshape(4, 4).T
        R = V[:3, :3]
        pos = np.array(info[11]) + R[2] * info[10]
        return info[0], info[1], pos, R[0], R[1], -R[2], P

    def draw(self, lines):
        """lines: list of (text, rgb, size). Anchored to the top-left of the 3-D view."""
        if not self.gui:
            return
        try:
            w, h, pos, right, up, fwd, P = self._cam()
        except Exception:
            return
        if w == 0:
            return
        D = 1.0
        y_ndc = 0.93
        for i, (text, col, size) in enumerate(lines):
            x_ndc = -0.97
            wp = pos + fwd * D + right * (x_ndc * D / P[0, 0]) + up * (y_ndc * D / P[1, 1])
            key = (text, tuple(np.round(wp, 3)))
            if self.cache.get(i) != key:
                kw = dict(textColorRGB=col, textSize=size, lifeTime=0)
                if i in self.ids:
                    self.ids[i] = p.addUserDebugText(text, list(wp), replaceItemUniqueId=self.ids[i], **kw)
                else:
                    self.ids[i] = p.addUserDebugText(text, list(wp), **kw)
                self.cache[i] = key
            y_ndc -= 2.0 * self.PX * (size if size > 1 else 1) / h
