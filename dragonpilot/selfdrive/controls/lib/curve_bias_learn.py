# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
"""
dragonpilot (dpcr, Corolla Cross Hybrid): the curve outward bias (FRAC) learns its own values.

Owner 2026-11-25: "can FRAC adjust itself - whenever a curve is taken a little inside or outside, correct it?" and then
"start on the learning program". Until now every FRAC value was set by hand after a drive (11-03, 11-05, 11-07, 11-09,
11-25), speed by speed.

What is measured. In a curve the lane-line centre says where the car sits: inside = -sign(curvature) x offset (offset
from LaneCentering.measure, positive = car left of the centre; curvature positive = right). Checked against the owner's
own words on 378 recorded curves (claude_work/dpcr_lat/curves.py; y at 0 m, at 3 / 9 m against the car's arc and a fit
over 3-19 m all agree within 2-3 cm):
  11-05 / 11-07, FRAC 0.06-0.11 ("clearly inside", "riding the inside line"): 40-50 km/h +0.15 .. +0.20 m inside,
      50-70 km/h +0.05 .. +0.10 m;
  11-24, FRAC 0.22 at 60 km/h: 60-70 km/h 0.00 m;  70-80 km/h +0.08 m, right curves +0.26 m ("80 km/h sometimes inside").
So the reading follows what he feels, in sign and in order of size.

How it learns. One curve = one episode: lateral active, no blinker, no lane change / lane choice, above LEARN_V_MIN,
the car's own curvature beyond LEARN_K for at least LEARN_T_MIN, the lines trusted for most of it, the driver's hands
off (a curve he steers through is not the system's result). The first LEARN_T_SKIP (the turn-in) and the last
LEARN_T_TAIL (the way out) are left out. The
episode's mean inside offset, beyond a LEARN_DB dead band and capped at LEARN_E_MAX, moves the learned fraction for
that side (left and right curves separately: they differ, 11-24 right +0.26 / left +0.07) at the two speed points
around the episode's speed, LEARN_GAIN per metre - an integrator over curves, not a correction inside the curve: one
curve moves a value by at most LEARN_STEP_MAX, so a wrong reading (a merge, a worn line) cannot do much, and nothing
changes while the car is in the curve that taught it.
Closed loop: the offset falls by S x the fraction with S ~ 0.5 .. 4 m per unit (the hand tuning 11-07 .. 11-24 and the
closed-loop sims of those days), so GAIN x S = 0.03 .. 0.24 per curve: it settles over ~5 .. 30 curves of a speed range
and cannot oscillate (stable below 2).

Where it acts. extra(v, curvature) is added to curve_bias's fraction (the model's curvature) AND to the line pursuit's
own outward fraction (lane_centering LINE_OUT_FRAC_V): above 70 km/h the pursuit carries 0.85 of the command and
curve_bias alone is left with ~2 %. The learned part is bounded to LEARN_RANGE around the hand-set table.

The values are kept in LEARN_FILE across drives (written from a thread, never in the control loop) and every accepted
episode goes to the log (cloudlog event "curve_bias_learn") so the next road data shows what was learned from what.
"""
import json
import os
import threading
from collections import deque

import numpy as np

from openpilot.common.swaglog import cloudlog
from dragonpilot.selfdrive.controls.lib.curve_bias import CURVE_OUTWARD_DEADZONE, CURVE_OUTWARD_V_BP

LEARN_FILE = "/data/dp_curve_bias_learn.json"
LEARN_V_MIN = 40.0 / 3.6     # m/s: below this the fixed table fades to 0 and tight curves need the full turn-in
LEARN_K = 6e-4               # 1/m: the car's own |curvature| that counts as a curve (R < ~1700 m)
LEARN_K_TAU = 0.5            # s: smoothing of that curvature
LEARN_T_SKIP = 1.0           # s: the turn-in, not learned from
LEARN_T_TAIL = 0.7           # s: the way out of the curve, not learned from either
LEARN_T_MIN = 3.0            # s: the shortest curve learned from (turn-in and way out included)
LEARN_VALID_MIN = 0.6        # share of the learned part with trusted lines
LEARN_PRESSED_MAX = 0.05     # share of the curve with the driver's hand on the wheel
LEARN_DB = 0.03              # m: inside / outside by less than this is centred
LEARN_E_MAX = 0.30           # m: the most one curve may claim
LEARN_GAIN = 0.06            # fraction per metre of mean inside offset, per curve
LEARN_STEP_MAX = 0.006       # fraction: the most one curve may move a value
LEARN_RANGE = [-0.05, 0.08]  # learned part, around the hand-set table (first road version: kept narrow)
LEARN_SAVE_MIN_T = 20.0      # s between two writes of the file


def _weights(v_ego: float) -> np.ndarray:
  """the interpolation weights of v_ego on the table's speed points"""
  bp = CURVE_OUTWARD_V_BP
  return np.array([float(np.interp(v_ego, bp, np.eye(len(bp))[i])) for i in range(len(bp))])


class CurveBiasLearner:
  def __init__(self, dt: float, path: str | None = LEARN_FILE):
    self.dt = dt
    self.path = path
    n = len(CURVE_OUTWARD_V_BP)
    self.delta = {'left': np.zeros(n), 'right': np.zeros(n)}
    self.count = {'left': np.zeros(n), 'right': np.zeros(n)}
    self.k = 0.0             # smoothed curvature of the car
    self.side = None         # the running episode's side
    self.t = 0.0             # its duration
    self.n = self.n_valid = self.n_all = self.n_pressed_all = 0
    self.sum_e = self.sum_v = 0.0
    self.tail = deque()      # the samples of the last LEARN_T_TAIL
    self.n_tail = max(int(round(LEARN_T_TAIL / dt)), 1)
    self.last = None         # the last accepted episode (side, v, e, step) - for tests and the log
    self.t_saved = -LEARN_SAVE_MIN_T
    self.clock = 0.0
    self.dirty = False
    self._load()

  # --- what the two users read ---
  def extra(self, v_ego: float, curvature: float) -> float:
    """learned outward fraction to add at this speed for a curve to this side (0 on a straight)"""
    if abs(curvature) <= CURVE_OUTWARD_DEADZONE:
      return 0.0
    return float(np.interp(v_ego, CURVE_OUTWARD_V_BP, self.delta['right' if curvature > 0.0 else 'left']))

  # --- learning ---
  def update(self, v_ego: float, curvature: float, meas, lat_active: bool, steering_pressed: bool, blinker: bool,
             lane_changing: bool):
    """once per control cycle. curvature: the car's own (positive = right); meas: LaneCentering.measure's (offset,
    heading) or None when the lines are not trusted"""
    self.clock += self.dt
    self.k += (curvature - self.k) * self.dt / LEARN_K_TAU
    side = None
    if lat_active and not blinker and not lane_changing and v_ego > LEARN_V_MIN and abs(self.k) > LEARN_K:
      side = 'right' if self.k > 0.0 else 'left'
    if side != self.side:
      if self.side is not None:
        self._finish()
      self.side, self.t = side, 0.0
      self.n = self.n_valid = self.n_all = self.n_pressed_all = 0
      self.sum_e = self.sum_v = 0.0
      self.tail.clear()
    if side is None:
      if self.dirty and self.clock - self.t_saved >= LEARN_SAVE_MIN_T:
        self._save()
      return
    self.t += self.dt
    self.n_all += 1
    self.n_pressed_all += bool(steering_pressed)   # his hand anywhere in the curve, the turn-in and the way out too
    if self.t <= LEARN_T_SKIP:
      return
    # a sample counts once it is LEARN_T_TAIL old: what is still waiting when the curve ends is the way out
    e = None
    if meas is not None and not steering_pressed:
      e = -(1.0 if side == 'right' else -1.0) * float(meas[0])   # + = the car sits toward the inside
    self.tail.append((v_ego, bool(steering_pressed), e))
    if len(self.tail) > self.n_tail:
      v_old, pressed_old, e_old = self.tail.popleft()
      self.n += 1
      self.sum_v += v_old
      if e_old is not None:
        self.n_valid += 1
        self.sum_e += e_old

  def _finish(self):
    if self.t < LEARN_T_MIN or self.n == 0 or self.n_valid < LEARN_VALID_MIN * self.n or self.n_pressed_all > LEARN_PRESSED_MAX * self.n_all:
      return
    v = self.sum_v / self.n
    e = self.sum_e / self.n_valid
    e_used = float(np.clip(e, -LEARN_E_MAX, LEARN_E_MAX))
    e_used -= float(np.clip(e_used, -LEARN_DB, LEARN_DB))
    step = float(np.clip(LEARN_GAIN * e_used, -LEARN_STEP_MAX, LEARN_STEP_MAX))
    w = _weights(v)
    self.count[self.side] += w
    self.last = (self.side, v, e, step)
    if step != 0.0:
      self.delta[self.side] = np.clip(self.delta[self.side] + step * w, LEARN_RANGE[0], LEARN_RANGE[1])
      self.dirty = True
    cloudlog.event("curve_bias_learn", side=self.side, v_kph=round(v * 3.6, 1), inside_m=round(e, 3), step=round(step, 4),
                   t=round(self.t, 1), delta=[round(float(x), 4) for x in self.delta[self.side]])

  # --- the file ---
  def _load(self):
    if not self.path or not os.path.isfile(self.path):
      return
    try:
      with open(self.path) as f:
        d = json.load(f)
      if [round(float(x), 3) for x in d['v_bp']] != [round(float(x), 3) for x in CURVE_OUTWARD_V_BP]:
        return   # the table's speed points changed: start again
      for side in ('left', 'right'):
        a = np.array(d['delta'][side], dtype=float)
        if a.shape == self.delta[side].shape and np.all(np.isfinite(a)):
          self.delta[side] = np.clip(a, LEARN_RANGE[0], LEARN_RANGE[1])
          self.count[side] = np.array(d.get('count', {}).get(side, self.count[side]), dtype=float).reshape(a.shape)
    except Exception:
      cloudlog.exception("curve_bias_learn: load failed, starting from zero")
      n = len(CURVE_OUTWARD_V_BP)
      self.delta = {'left': np.zeros(n), 'right': np.zeros(n)}
      self.count = {'left': np.zeros(n), 'right': np.zeros(n)}

  def _save(self):
    self.dirty, self.t_saved = False, self.clock
    if not self.path:
      return
    data = json.dumps({'v_bp': [float(x) for x in CURVE_OUTWARD_V_BP],
                       'delta': {s: [float(x) for x in self.delta[s]] for s in ('left', 'right')},
                       'count': {s: [float(x) for x in self.count[s]] for s in ('left', 'right')}})
    threading.Thread(target=_write, args=(self.path, data), daemon=True).start()


def _write(path: str, data: str):
  try:
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
      f.write(data)
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, path)
  except Exception:
    cloudlog.exception("curve_bias_learn: save failed")
