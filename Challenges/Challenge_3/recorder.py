"""
recorder.py - renders the simulation to an mp4 with the TCP 'trail' drawn in.

The trail of the current episode is bright; trails of earlier episodes stay
on screen, faded, so one video shows many different trials (as the
deliverable asks: "1 min per IK method showing different trails").
"""
import cv2
import imageio
import mujoco
import numpy as np

COLORS = {"mink": (0.1, 0.8, 1.0), "dls": (1.0, 0.6, 0.1),
          "qp": (0.6, 1.0, 0.2), "pinv": (1.0, 0.3, 0.8)}


class Recorder:
    def __init__(self, path, max_seconds=60, fps=25, w=800, h=600, camera="front"):
        import os
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.writer = imageio.get_writer(path, fps=fps, codec="libx264",
                                         quality=7, macro_block_size=8)
        self.fps, self.w, self.h, self.cam = fps, w, h, camera
        self.max_frames = int(max_seconds * fps)
        self.frames = 0
        self.renderer = None
        self.recording, self.full = True, False
        self.trail, self.old_trails = [], []
        self.n_ok = self.n_done = 0
        self.status = ""
        self.next_t = 0.0
        self.method, self.ep, self.cube = "", 0, (0, 0, 0)

    def new_episode(self, method, ep, cube):
        if self.trail:
            self.old_trails.append(self.trail[::6])   # thinned: faster rendering
            self.old_trails = self.old_trails[-5:]
        self.trail = []
        self.method, self.ep, self.cube = method, ep, cube
        self.next_t = 0.0
        self.status = "running"

    def maybe_frame(self, sim):
        if not self.recording or sim.d.time < self.next_t:
            return
        self.next_t += 1.0 / self.fps
        if self.renderer is None:
            self.renderer = mujoco.Renderer(sim.m, self.h, self.w, max_geom=20000)
        if not self.trail or np.linalg.norm(self.trail[-1] - sim.tcp_pos()) > 0.004:
            self.trail.append(sim.tcp_pos())   # only add a point when the TCP moved
        r = self.renderer
        r.update_scene(sim.d, self.cam)
        col = COLORS.get(self.method, (1, 1, 1))
        for tr, alpha, size in [(t, 0.35, 0.003) for t in self.old_trails] + \
                               [(self.trail, 1.0, 0.004)]:
            for p in tr:
                if r.scene.ngeom >= r.scene.maxgeom:
                    break
                g = r.scene.geoms[r.scene.ngeom]
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, [size, 0, 0],
                                    np.asarray(p, float), np.eye(3).flatten(),
                                    np.array([*col, alpha], np.float32))
                r.scene.ngeom += 1
        img = r.render().copy()
        self._text(img)
        self.writer.append_data(img)
        self.frames += 1
        if self.frames >= self.max_frames:
            self.recording, self.full = False, True

    def _text(self, img):
        x, y, yaw = self.cube
        lines = [f"IK method: {self.method.upper()}    episode {self.ep}",
                 f"cube x={x:.2f} y={y:.2f} yaw={np.degrees(yaw):.0f} deg",
                 f"success so far: {self.n_ok}/{self.n_done}   {self.status}"]
        for i, t in enumerate(lines):
            cv2.putText(img, t, (12, 26 + 25 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(img, t, (12, 26 + 25 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (255, 255, 255), 2, cv2.LINE_AA)

    def end_episode(self, sim, log):
        self.n_done += 1
        self.n_ok += int(log["success"])
        self.status = "SUCCESS" if log["success"] else "FAIL: " + log["failure_reason"][:40]
        # show the result for ~1 s of extra frames
        if self.recording and self.renderer is not None:
            for _ in range(self.fps):
                self.r_hold(sim)

    def r_hold(self, sim):
        self.next_t = sim.d.time          # force a frame without stepping
        self.maybe_frame(sim)   # TCP is still, so no trail point is added

    def close(self):
        self.writer.close()
        if self.renderer is not None:
            self.renderer.close()