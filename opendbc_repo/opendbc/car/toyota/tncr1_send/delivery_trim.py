# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from opendbc.car import DT_CTRL

# Delivery trim (tncr, Corolla Cross hybrid). Owner 2026-10-15 evening: "the force the system sends must be the force the
# car gives; close that gap, so that from now on we can read the real signal that gives a given force".
# Measured on five routes of 2026-10-15 (00000069/6a/6b morning, 70/72/73/74 afternoon; sendcan 0x343 ACCEL_CMD vs
# carState aEgo 0.35 s later, engaged, no driver brake; tools claude_work/tncr_1015pm/delivery_map.py). Median
# (delivered - sent) in m/s^2, + = the car gives LESS than sent, steady command (|change| < 0.1 over 0.3 s):
#   km/h    -0.6..-0.25  -1.0..-0.6  -1.5..-1.0  -2.0..-1.5  -3.5..-2.0
#   0-2        -0.02        -0.09
#   2-4        -0.05        -0.08       +0.16
#   4-6        -0.00        -0.07       -0.07       -0.13
#   6-9        +0.03        +0.06       +0.10       -0.15
#   9-13       +0.16        +0.21       +0.23                                <- the hybrid's regen->friction handover
#   13-20      +0.01        +0.07       +0.17
#   20-30      -0.03        +0.00       +0.04       -0.20       -0.55        <- hard braking over-delivers
#   30-50      -0.02        -0.02       -0.06       -0.14
# While the command is being RELEASED (rose > 0.1 in 0.3 s) the car gives a further ~0.1 less at 6-20 km/h
# (+0.14..+0.33 vs the steady cells); while it is being FIRMED at 20-50 km/h it gives 0.1-0.4 MORE (the PCM leads).
# These are the residuals AFTER the stock PID (kp 0, ki 0.5 -> 0.1 by speed, integral unwound every frame) and the
# sunnypilot brake corrections (handover feed-forward 10-14 km/h, overshoot limiter below -2.5): the integrator never
# closes them. So this stage adds the measured residual as a feed-forward, just before the end-force cap:
#   steady:  STEADY_V over STEADY_BP (firmer), full once the command is braking at least STEADY_FULL, fading in from
#            STEADY_FROM so a light touch is untouched;
#   release: RELEASE_EXTRA more while the input command has risen by more than RELEASE_D over RELEASE_T, 6-20 km/h;
#   hard:    HARD_LIFT_V over HARD_LIFT_BP (lighter) at >= 20 km/h, never above HARD_CEIL (a hard brake stays hard).
# The trim itself moves at most TRIM_RATE so crossing a band is not a step. Below 6 km/h nothing (matched there, and
# the end of the stop belongs to the end rules). Nothing at a standstill or while not braking.
ENABLED = True
STEADY_BP = [6.0 / 3.6, 9.0 / 3.6, 11.0 / 3.6, 13.0 / 3.6, 20.0 / 3.6, 25.0 / 3.6]  # m/s
# Owner 2026-10-15 late night road test of ebff239099: "brakes too early and stops too far". The closed-loop sim
# (claude_work/tncr_1015pm/sim, route 6b 09:19:54 slow lead stop) shows why: at 10-15 km/h the plan asks -0.95, this
# steady trim plus the stock PID send -1.33 and the car gives -1.10, 0.15 MORE than the plan; without the steady trim
# the PID alone already made the car match the plan there (sim stop gap 6.07 m with it, 4.78 m without). The residual
# the map measured on sent-vs-delivered was being closed by the PID on plan-vs-delivered, so adding it again
# double-counts. -> steady part OFF (all zeros, kept as a table for a later re-measure with the PID in the loop);
# the release extra (the PID lags a release) and the hard lift stay.
STEADY_V = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]           # m/s^2 added (firmer); was [0, 0.05, 0.20, 0.15, 0.10, 0]
STEADY_FROM = -0.3   # m/s^2: no trim for a lighter command ...
STEADY_FULL = -0.6   # m/s^2: ... full trim from here
RELEASE_EXTRA = 0.10  # m/s^2 (firmer) while releasing
RELEASE_D = 0.10      # m/s^2 rise ...
RELEASE_T = 0.3       # s ... over this long
RELEASE_BP = [6.0 / 3.6, 8.0 / 3.6, 20.0 / 3.6, 25.0 / 3.6]  # m/s
RELEASE_W = [0.0, 1.0, 1.0, 0.0]
HARD_LIFT_BP = [-3.0, -2.0, -1.5]   # m/s^2 command
HARD_LIFT_V = [0.35, 0.15, 0.0]     # m/s^2 taken off (lighter)
HARD_V_BP = [15.0 / 3.6, 20.0 / 3.6]  # m/s
HARD_V_W = [0.0, 1.0]
HARD_CEIL = -1.5      # m/s^2: the lifted command never gets lighter than this
TRIM_RATE = 1.0       # m/s^3
# dppb (Corolla Altis Hybrid, 2026-11-20). The tables above are the Cross's residuals. The Altis, fitted on 547 engaged
# braking stretches (3.9 h of dppb rlogs 11-02..11-20, claude_work/stopdb/plantfit.py; sent 0x343 command -> carState
# aEgo, per speed band: delivered = GAIN x sent + OFF after a pure delay of 0.5 / 0.4 / 0.3 / 0.2 s at < 3 / 3-6 / 6-10 /
# > 10 km/h, fit rms 0.14-0.19 m/s^2):
#   km/h     0.7-3   3-6    6-10   10-15  15-25  25-40  40-65
#   GAIN     0.94    0.85   0.84   0.98   1.10   1.18   1.17
#   OFF     +0.09   +0.06  +0.08  +0.16  +0.09  +0.06  +0.04     (+ = lighter than sent)
# i.e. below ~15 km/h the car gives 0.1-0.25 LESS than sent (a stop planned on the request rolls ~1.3 m further: the
# dpcr stack moved to dppb as it was stopped at 2.1 m in the closed-loop sim), above 20 km/h 10-18 % MORE. The command
# sent is the one that delivers the request: (request - OFF) / GAIN, faded in from ALTIS_V_W (nothing below 2 km/h:
# the end of the stop keeps the owner's end rule and end_force_cap), only on a braking request, moved at TRIM_RATE.
ALTIS_ENABLED = True
ALTIS_V = [1.8 / 3.6, 4.5 / 3.6, 8.0 / 3.6, 12.5 / 3.6, 20.0 / 3.6, 32.0 / 3.6, 52.0 / 3.6]   # m/s
ALTIS_GAIN = [0.94, 0.85, 0.84, 0.98, 1.10, 1.18, 1.17]
ALTIS_OFF = [0.09, 0.06, 0.08, 0.16, 0.09, 0.06, 0.04]
ALTIS_V_W_BP = [2.0 / 3.6, 4.0 / 3.6]   # m/s
ALTIS_V_W = [0.0, 1.0]
ALTIS_SHARE = 1.0     # share of the measured residual that is fed forward (the PID closes the rest)
ALTIS_FROM = -0.15    # m/s^2: nothing on a lighter request ...
ALTIS_FULL = -0.35    # m/s^2: ... all of it from here
ALTIS_FIRM_D = 0.15   # m/s^2 firmer over RELEASE_T (0.5 m/s^3): the command is being firmed


class DeliveryTrim:
  def __init__(self, dt: float = DT_CTRL * 3):
    self.dt = dt
    self.n_hist = max(int(round(RELEASE_T / dt)), 1)
    self.reset()

  def reset(self):
    self.hist: list[float] = []
    self.trim = 0.0   # the applied correction (negative = firmer)

  def _target(self, cmd: float, v_ego: float, releasing: bool) -> float:
    if not ENABLED or cmd >= STEADY_FROM or ALTIS_ENABLED:   # the Cross's tables: not with the Altis map
      return 0.0
    w_level = float(np.interp(cmd, [STEADY_FULL, STEADY_FROM], [1.0, 0.0]))
    firmer = float(np.interp(v_ego, STEADY_BP, STEADY_V)) * w_level
    if releasing:
      firmer += RELEASE_EXTRA * float(np.interp(v_ego, RELEASE_BP, RELEASE_W)) * w_level
    lighter = float(np.interp(cmd, HARD_LIFT_BP, HARD_LIFT_V)) * float(np.interp(v_ego, HARD_V_BP, HARD_V_W))
    lighter = min(lighter, max(HARD_CEIL - cmd, 0.0))  # never above HARD_CEIL
    return -firmer + lighter

  @staticmethod
  def _altis(cmd: float, v_ego: float, firming: bool = False) -> float:
    """what to add so the Altis delivers cmd (negative = firmer). While the command is being firmed nothing is added
    on the firmer side: the map is the steady car, and a brake that is building gives 0.10-0.20 MORE than it (same
    rlogs, command firming 0.5-1.5 m/s^3 below 25 km/h) - on 11-20 19:28:00 the firmer trim on top of a building
    brake made -2.8 of a planned -1.8"""
    if not ALTIS_ENABLED or cmd >= ALTIS_FROM:
      return 0.0
    inv = (cmd - float(np.interp(v_ego, ALTIS_V, ALTIS_OFF))) / float(np.interp(v_ego, ALTIS_V, ALTIS_GAIN))
    w = float(np.interp(v_ego, ALTIS_V_W_BP, ALTIS_V_W)) * float(np.interp(cmd, [ALTIS_FULL, ALTIS_FROM], [1.0, 0.0]))
    trim = ALTIS_SHARE * w * (inv - cmd)
    return max(trim, 0.0) if firming else trim

  def apply(self, cmd: float, v_ego: float, standstill: bool, long_active: bool) -> float:
    if not long_active or standstill:
      self.reset()
      return cmd
    self.hist.append(cmd)
    if len(self.hist) > self.n_hist + 1:
      self.hist.pop(0)
    releasing = len(self.hist) > self.n_hist and (cmd - self.hist[0]) > RELEASE_D
    firming = len(self.hist) > self.n_hist and (cmd - self.hist[0]) < -ALTIS_FIRM_D
    target = self._target(cmd, v_ego, releasing) + self._altis(cmd, v_ego, firming)
    step = TRIM_RATE * self.dt
    self.trim = float(np.clip(target, self.trim - step, self.trim + step))
    return cmd + self.trim
