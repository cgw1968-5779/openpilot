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

# After the driver's gas (dpcr, owner 2026-11-26: "the stop after creeping up with the gas is still heavy - the PCM?
# keep looking for a way to solve it"). The PCM reports what it really asks for, PCM_CRUISE ACCEL_NET. On an ordinary
# low-speed brake that is our command plus its creep-torque compensation, which follows PCM_CRUISE NEUTRAL_FORCE
# (6306 samples below 9 km/h, claude_work/dpcr_1127/pcmscan.py): net - cmd = -0.00036 x neutral + 0.03, i.e. -0.33 at
# 0.4-1 km/h (955 N), -0.25 at 2.5-3 (668 N), -0.17 at 4-5 (447 N), -0.03 at 6-8 (149 N), within 0.04 of the line.
# After the gas it asks for MORE than that, from the first frame and for as long as the stop takes:
#   0000000a 11-24 22:55:04 (gas released at 3 km/h): cmd -0.25, net -0.80 (expected -0.50), 1200 N, the car on -0.55;
#   000000ec 11-06 (3 km/h): cmd -0.30, net -0.67 (expected -0.53), 1000-1040 N, the car on -0.45 .. -0.57.
# That extra (0.12 .. 0.30) is the PCM's own and it is measured every frame, with no lag: so for GAS_T after the gas,
# below GAS_V, the command is eased by exactly what the PCM adds beyond its usual compensation (less GAS_DB, at most
# GAS_MAX, moved at GAS_RATE, the brake never let go completely: GAS_FLOOR). Only eases, only while the PCM really
# does it - once net - cmd is back on the line the trim is gone - and never on a command that is not braking.
GAS_T = 6.0            # s after the gas was let go
GAS_V = 10.0 / 3.6     # m/s
GAS_NET_K = -0.00036   # m/s^2 per N of NEUTRAL_FORCE
GAS_NET_C = 0.03       # m/s^2
GAS_DB = 0.06          # m/s^2
GAS_MAX = 0.30         # m/s^2
GAS_RATE = 1.0         # m/s^3
GAS_TAU = 0.15         # s: smoothing of the measured extra
GAS_FLOOR = -0.05      # m/s^2: the eased command stays a brake
GAS_FADE = 1.0         # s: the last part of GAS_T fades the trim out
GAS_CMD_BP = [-1.0, -0.6]   # m/s^2: a firm command (the room is short) is eased less, from -1.0 not at all

# A small gas at creep speed (dpcr, owner 2026-11-28, route 00000010 14:29:50: "the lead crept off, the gap slowly grew
# too far and the car did not follow; it should add a little gas and stay with the lead - without disturbing the
# smooth stop end"). The lead left at 3-7 km/h; the plan asked +0.10 .. +0.28 for 12 s, the car reached 5.3 km/h and
# stayed there (aEgo +0.02 .. +0.08 on a +0.31 .. +0.35 command, the PID's integral already in it), the gap went
# 3.1 -> 13 m and the owner pressed the gas. The hybrid's idle creep ends near 5 km/h and a small positive command
# gives about half of itself there (all of that day's engaged samples, command +0.05 .. +0.6, speed 1.5 s later):
#   km/h      command  ->  car          km/h      command  ->  car
#   0-2        +0.19      +0.34         4-5.5      +0.29      +0.06
#   2-4        +0.20      +0.18         4-5.5      +0.30      +0.12
#   2-4        +0.42      +0.58         9-12       +0.24      +0.05
#   4-5.5      +0.97      +1.15         9-12       +0.39      +0.33
# i.e. roughly the first 0.2 of a positive command is lost between 4 and 12 km/h; a firm one (>= 0.6) is not.
# So, on a positive command only: while the car gives less than the command, the missing part is added, integrated at
# CREEP_KI (taken back CREEP_BACK times faster once the car gives more), at most CREEP_MAX. Faded in over CREEP_V_BP
# (below it the idle creep already gives more than asked) and out again above it, out over CREEP_CMD_BP for a firm
# command, and in proportion to the command over CREEP_ZERO, so it is exactly zero at and below a zero command:
# a braking command - every stop, and the end of it - never passes through this.
CREEP_V_BP = [3.0 / 3.6, 4.5 / 3.6, 12.0 / 3.6, 15.0 / 3.6]   # m/s
CREEP_V_W = [0.0, 1.0, 1.0, 0.0]
CREEP_CMD_BP = [0.40, 0.60]   # m/s^2: the command from which less / nothing is added
CREEP_ZERO = 0.10             # m/s^2: full only from this command up
CREEP_KI = 0.8                # 1/s
CREEP_BACK = 2.5              # x faster back
CREEP_MAX = 0.25              # m/s^2


class DeliveryTrim:
  def __init__(self, dt: float = DT_CTRL * 3):
    self.dt = dt
    self.n_hist = max(int(round(RELEASE_T / dt)), 1)
    self.reset()
    self.gas_t = 1e3    # s since the driver's gas
    self.gas_trim = 0.0  # what the command is eased by after the gas (GAS_*)
    self.extra = 0.0    # what the PCM asks beyond its usual compensation (smoothed, <= 0)
    self.sent = None    # the command last returned
    self.creep = 0.0    # gas added for a small positive command the car does not give (CREEP_*)

  def reset(self, gas: bool = False):
    self.hist: list[float] = []
    self.trim = 0.0   # the applied correction (negative = firmer)
    if hasattr(self, 'gas_t'):
      # long control is off: with the driver's gas the clock starts again, otherwise it just runs on
      self.gas_t = 0.0 if gas else self.gas_t + self.dt
      self.gas_trim, self.extra, self.sent = 0.0, 0.0, None
      self.creep = 0.0

  def _after_gas(self, cmd: float, v_ego: float, pcm_net, neutral) -> float:
    self.gas_t += self.dt
    live = pcm_net is not None and neutral is not None and self.gas_t < GAS_T and v_ego < GAS_V and cmd < 0.0
    target = 0.0
    if live and self.sent is not None:
      extra = min((pcm_net - self.sent) - (GAS_NET_K * neutral + GAS_NET_C), 0.0)
      self.extra += (extra - self.extra) * min(self.dt / GAS_TAU, 1.0)
      target = float(np.clip(-self.extra - GAS_DB, 0.0, GAS_MAX)) * float(np.interp(self.gas_t, [GAS_T - GAS_FADE, GAS_T], [1.0, 0.0])) * \
        float(np.interp(cmd, GAS_CMD_BP, [0.0, 1.0]))
    elif not live:
      self.extra = 0.0
    step = GAS_RATE * self.dt
    self.gas_trim = float(np.clip(target, self.gas_trim - step, self.gas_trim + step))
    return max(cmd, min(cmd + self.gas_trim, GAS_FLOOR))   # only ever lighter, and still a brake

  def _creep_gas(self, cmd: float, v_ego: float, a_ego) -> float:
    w = 0.0
    if a_ego is not None and cmd > 0.0:
      w = float(np.interp(v_ego, CREEP_V_BP, CREEP_V_W)) * float(np.interp(cmd, CREEP_CMD_BP, [1.0, 0.0]))
    if w > 0.0:
      short = cmd - a_ego
      self.creep = float(np.clip(self.creep + CREEP_KI * (short if short > 0.0 else CREEP_BACK * short) * self.dt, 0.0, CREEP_MAX * w))
    else:
      self.creep = max(self.creep - CREEP_KI * CREEP_BACK * CREEP_MAX * self.dt, 0.0)
    if cmd <= 0.0:
      return cmd
    return cmd + self.creep * min(cmd / CREEP_ZERO, 1.0)

  def _target(self, cmd: float, v_ego: float, releasing: bool) -> float:
    if not ENABLED or cmd >= STEADY_FROM:
      return 0.0
    w_level = float(np.interp(cmd, [STEADY_FULL, STEADY_FROM], [1.0, 0.0]))
    firmer = float(np.interp(v_ego, STEADY_BP, STEADY_V)) * w_level
    if releasing:
      firmer += RELEASE_EXTRA * float(np.interp(v_ego, RELEASE_BP, RELEASE_W)) * w_level
    lighter = float(np.interp(cmd, HARD_LIFT_BP, HARD_LIFT_V)) * float(np.interp(v_ego, HARD_V_BP, HARD_V_W))
    lighter = min(lighter, max(HARD_CEIL - cmd, 0.0))  # never above HARD_CEIL
    return -firmer + lighter

  def apply(self, cmd: float, v_ego: float, standstill: bool, long_active: bool, pcm_net=None, neutral=None, a_ego=None) -> float:
    if not long_active or standstill:
      self.reset()
      return cmd
    self.hist.append(cmd)
    if len(self.hist) > self.n_hist + 1:
      self.hist.pop(0)
    releasing = len(self.hist) > self.n_hist and (cmd - self.hist[0]) > RELEASE_D
    target = self._target(cmd, v_ego, releasing)
    step = TRIM_RATE * self.dt
    self.trim = float(np.clip(target, self.trim - step, self.trim + step))
    self.sent = self._after_gas(cmd + self.trim, v_ego, pcm_net, neutral)
    self.sent = self._creep_gas(self.sent, v_ego, a_ego)
    return self.sent
