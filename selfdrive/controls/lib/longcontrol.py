import numpy as np
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from dragonpilot.selfdrive.controls.lib.creep_stop import CreepStop
from dragonpilot.selfdrive.controls.lib.stop_control import StopControl
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

# dragonpilot (dppb, Corolla Altis Hybrid, owner 2026-11-03): soft end of a stop.
# The planner already eases toward the stop, but below ~10 km/h the Altis decel lags the command by ~0.5 s
# (dppb routes 17f-188: best-fit lag 0.5 s, 0.1 s at 10-25 km/h), so a stop after a firmer middle (lead braked
# a little) still ends at -0.8..-0.9 m/s^2 and the car lurches at the stop. The decel at the stop follows the
# command ~0.6 s earlier almost 1:1 (7 stops). So when the time to the stop, from the actual speed and decel,
# drops below SOFT_END_LAG + SOFT_END_RAMP, the command eases to SOFT_END_ACCEL (owner's Altis end value
# at 0 km/h) and reaches it SOFT_END_LAG before the stop. Once easing starts it never firms again until the car
# has stopped (no stop-go-stop). Only ever lighter than the planner, only in the last ~1 s, so the start and the
# middle of every stop are unchanged.
SOFT_END_ACCEL = -0.30  # m/s^2
SOFT_END_LAG = 0.5      # s
SOFT_END_RAMP = 0.5     # s, shortest easing time
SOFT_END_MAX_RATE = 1.0  # m/s^3, easing never releases faster than this: a firmer brake starts easing earlier
SOFT_END_MIN_DECEL = 0.3  # m/s^2, floor of the decel used for the time to the stop
SOFT_END_RESET_V = 3.0  # m/s, a new stop can start easing again above this speed
SOFT_END_URGENT = -1.5  # m/s^2, never ease while the planner asks for more than this (urgent stop)
# dragonpilot (dpcr, owner 2026-11-03; dppb port 2026-11-04): "at mid / low speed, the first 0.1 s of the brake on the
# first SET / RESUME press should be softer". Engaging with a lead (or a set speed) that needs braking, the planner's
# target is applied at once and carcontroller's 4 m/s^3 wind-down is the only limit: -0.4 m/s^2 after 0.1 s, felt as a
# bite. A brake that starts within ENGAGE_ONSET_WINDOW of engaging builds with a jerk rising from ENGAGE_ONSET_J0 to
# ENGAGE_ONSET_J1 over ENGAGE_ONSET_T (-0.15 at 0.1 s, -0.40 at 0.2 s; stock -0.4 / -0.8), then keeps climbing
# linearly at the stock ENGAGE_ONSET_J_STOCK until the planner's request is reached (-0.8 at 0.3 s, ~-1.6 at 0.5 s).
# dppb (Altis, owner 2026-11-04, after 11-04 13:24: engaged 44 m behind a stopped car at 45 km/h, stopped 2.1 m out):
# "keep the start soft, then climb linearly; if it cannot be higher at 0.5 s keep climbing after it, and hold the
# firmest brake a little longer, so the stop ends at 3-3.5 m". So after the onset, for the rest of this stop, behind a
# radar lead that has been stopped for ENGAGE_HOLD_LEAD_T: the decel the part of the stop above ENGAGE_HOLD_V_TAIL
# needs (lead distance less the lag travel, ENGAGE_HOLD_GAP and the distance the normal end from V_TAIL takes,
# ENGAGE_HOLD_D_TAIL - Altis 33 stops: median 7.4 m, p75 11.3) is compared with the planner's request; where the plan
# releases earlier the brake is held, at most ENGAGE_HOLD_MAX more and NEVER firmer than this stop's own firmest
# planner request so far (it holds the peak longer, it never deepens it). It builds at ENGAGE_HOLD_UP, lets go at
# ENGAGE_HOLD_DOWN, fades out between ENGAGE_HOLD_FADE_V km/h so the normal end (soft end, stop end level) is left as
# it is, and stops ENGAGE_HOLD_MAX_T after the onset. Owner-approved exception to "never firmer than the planner",
# only for this engage stop. Closed-loop sim of 13:24 (dppb x86 MPC, recorded car + 0.4 s lagged command difference):
# stock 2.53 m, soft onset only 2.39 m, with the hold 3.07 m; peak command -2.85 (stock -2.78), held to ~15 km/h.
ENGAGE_ONSET_WINDOW = 1.0   # s after engaging in which a brake onset gets the soft start
ENGAGE_ONSET_J0 = 1.0       # m/s^3 at the start of the brake
ENGAGE_ONSET_J1 = 3.0       # m/s^3 at ENGAGE_ONSET_T
ENGAGE_ONSET_T = 0.2        # s: the soft part
ENGAGE_ONSET_J_STOCK = 4.0  # m/s^3 after it (stock carcontroller wind-down limit), until the request is reached
# ENGAGE_FROM_DECEL - dppb (owner 2026-11-11, route 000001a9 22:04 bookmark "a jolt in the middle of the brake"):
# engaged at 34 km/h 30 m behind a lead braking 44 -> 0 km/h while the car was coasting on regen at -0.75 m/s^2 (the
# driver had just lifted off the gas). The onset ramp started from the last command, 0: the PCM dropped the regen
# (actual -0.75 -> -0.32), then the climb to -1.65 in 0.5 s made it bite at -2.99 while -1.67 was sent. 14 engage-and-
# brake events in the 11-04..11-11 rlogs: a release of +0.1..+0.4 in 9 of them. The onset now starts from the car's
# current decel (min(aEgo, 0)), never firmer than the request; engaging while not slowing is unchanged.

# dppb (owner 2026-11-05, exception to "never lighter than the planner" approved): "when a car cuts in, start the brake
# soft but climb quickly to the planner's force; keep it simple". 21:53:33 at 56 km/h, accelerating +0.6, a car cut in
# 19 m ahead at 48 km/h and the brake felt too strong at the front. So a brake that starts after BRAKE_ONSET_COAST_T
# without braking (accelerating or coasting: every cut-in) gets the same curve as the engage onset: -0.15 at 0.1 s,
# -0.40 at 0.2 s, then the stock ENGAGE_ONSET_J_STOCK until the planner's request is reached. A planner that builds
# slower is untouched.
BRAKE_ONSET_FROM = -0.1      # m/s^2: "not braking" is a command at or above this
BRAKE_ONSET_COAST_T = 0.5    # s not braking before a new onset
# dppb (owner 2026-11-08, route 00000198 12:34:20 bookmark: a car cut in 28 m ahead at ~42 km/h, us 50 km/h; "too heavy -
# it should be the same: soft first, then firmer only as the situation needs"). The 0.2 s soft part was too short to be
# felt: the stock 4 m/s^3 climb after it SENT -0.93 at 0.4 s, the Altis did nothing for ~0.4 s and then bit, -0.38 ->
# -2.00 in 0.3 s while sending -1.22 (planner -1.39). dppb rlogs 11-04..11-08, 55 brake onsets at 25-75 km/h after
# >= 0.5 s without braking: sent at 0.5 s -0.4..0 -> the car braked a median 0.11 more than sent (delivered jerk 1.5
# m/s^3), -0.7..-0.4 -> 0.14 (2.2), firmer than -0.7 -> 0.56-0.67 (3.7-4.4): a fast climb is what makes the Altis bite.
# So after the soft part the climb follows what the lead actually needs, from radarState now (dynamic, no table):
#   need = max(-aLeadK, 0) + max(vEgo - vLead, 0)^2 / (2 * (dRel - ONSET_S0 - vEgo * ONSET_LAG))
# climb jerk = (need - current brake) / ONSET_T_CATCH, at least ONSET_J_SLOW, at most the stock ENGAGE_ONSET_J_STOCK:
# already braking as much as needed -> 1.0 m/s^3 on to the planner's request (it is still reached); braking less than
# needed -> catch up within ONSET_T_CATCH; no radar lead, or within ONSET_S0 + the lag travel -> the stock climb.
# Replay: 12:34:20 sent at 0.5 s -0.70 instead of -1.07, planner's -1.37 reached at 1.2 s instead of ~0.6 s; over the 11
# dppb drives only 11-04 21:53:34 (the 11-05 cut-in) changes (at most 0.42 lighter for 1.1 s, same peak); no normal stop
# changes (their planner builds slower than 1 m/s^3). Extends the owner's 11-05 cut-in exception (lighter than the
# planner only during the onset climb, never firmer).
ONSET_J_SLOW = 1.0           # m/s^3
ONSET_T_CATCH = 0.5          # s
ONSET_S0 = 4.0               # m: the owner's standard gap
ONSET_LAG = 0.5              # s of travel before a firmer command takes effect
ENGAGE_HOLD_MAX_T = 15.0    # s after the onset
ENGAGE_HOLD_LEAD_V = 1.0    # m/s: lead counted as stopped below this
ENGAGE_HOLD_LEAD_T = 0.5    # s the lead must have been stopped
ENGAGE_HOLD_GAP = 3.25      # m (11-15 owner: stop at 3.25 m, was 3.5; 11-12 owner: stop at 3.5 m, was 4.0; 11-05: was 3.3)
ENGAGE_HOLD_LAG = 0.5       # s
ENGAGE_HOLD_V_TAIL = 12.0   # km/h: below this the stop is the normal end's
ENGAGE_HOLD_D_TAIL = 9.0    # m the normal end takes from ENGAGE_HOLD_V_TAIL
ENGAGE_HOLD_FADE_V = [12.0, 20.0]  # km/h: no hold below the first, full above the second (11-05: was 10-15)
ENGAGE_HOLD_MAX = 0.6       # m/s^2 at most held beyond the planner
ENGAGE_HOLD_UP = 2.0        # m/s^3
ENGAGE_HOLD_DOWN = 0.5      # m/s^3 (11-05: was 1.0)

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill):
  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid
  return long_control_state

class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    self.soft_end_frac = 1.0
    self.engage_t = 0.0  # s since engaging
    self.onset_from = 0.0  # engage onset start: the car's decel at the onset (never firmer than the request)
    self.onset_t: float | None = None  # s since the soft brake onset began (None: not in one)
    self.onset_done = False  # the onset ramp reached the request
    self.hold = 0.0  # <= 0: brake held beyond the planner in this engage stop
    self.hold_peak = 0.0  # this engage stop's firmest planner request so far
    self.hold_lead_t = 0.0  # s the lead has been stopped
    self.bo_t: float | None = None  # s since a brake onset (while driving) began
    self.bo_ramp = 0.0
    self.coast_t = 0.0  # s without braking
    self.lead = None  # radarState.leadOne, set by controlsd every frame (None: no radarState)
    self.pcm = None  # (ACCEL_NET, NEUTRAL_FORCE) from carStateExt, set by controlsd every frame (None: not reported)
    self.creep_stop = CreepStop(DT_CTRL, CP.mass)
    self.stop_control = StopControl(DT_CTRL)
    self.imu_ax = None  # livePose device acceleration x, set by controlsd (None: not available)

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      # dragonpilot: firm toward stopAccel only once stopped (holding); never add brake before the stop
      if output_accel > self.CP.stopAccel and CS.standstill:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target)

    # dragonpilot hook (stop control): one controller owns the stop behind a stopped lead (distance closed loop, true
    # stop instant); the old end layers only still shape a stop WITHOUT a radar lead
    stop_out = self.stop_control.update(output_accel, a_target, CS, self.lead, self.long_control_state != LongCtrlState.off, self.imu_ax)
    if self.stop_control.active or self.stop_control.held:
      output_accel = stop_out
      self.pid.reset()
    else:
      output_accel = self.soft_stop_end(output_accel, a_target, CS)
      output_accel = self.engage_onset(output_accel, CS)
    # dragonpilot hook (creep stop): the stop after the driver crept up with the gas keeps its own layer (the PCM is in
    # its stop-hold mode there and adds brake of its own, which only this layer counters)
    creep_out = self.creep_stop.update(output_accel, CS, self.lead, a_target, active, self.pcm)
    if self.creep_stop.on or self.creep_stop.holding:
      output_accel = creep_out
      self.stop_control.override(output_accel)

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel

  def soft_stop_end(self, output_accel, a_target, CS):  # the 11-03 soft end (stops without a stopped radar lead)
    # a new stop (or the lead moving off before we stopped) starts from scratch: no easing left over
    if CS.vEgo > SOFT_END_RESET_V or output_accel >= SOFT_END_ACCEL or \
       self.long_control_state not in (LongCtrlState.pid, LongCtrlState.stopping):
      self.soft_end_frac = 1.0
    if CS.standstill or output_accel >= SOFT_END_ACCEL or a_target < SOFT_END_URGENT or \
       self.long_control_state not in (LongCtrlState.pid, LongCtrlState.stopping):
      return output_accel
    t_stop = CS.vEgo / max(-CS.aEgo, SOFT_END_MIN_DECEL)
    ramp = max(SOFT_END_RAMP, (SOFT_END_ACCEL - output_accel) / SOFT_END_MAX_RATE)
    self.soft_end_frac = min(self.soft_end_frac, float(np.clip((t_stop - SOFT_END_LAG) / ramp, 0.0, 1.0)))
    soft = SOFT_END_ACCEL + (output_accel - SOFT_END_ACCEL) * self.soft_end_frac
    # the easing itself never releases faster than SOFT_END_MAX_RATE from the last command (the planner's own release
    # is left as it is: never firmer than the planner's output)
    return max(output_accel, min(soft, self.last_output_accel + SOFT_END_MAX_RATE * DT_CTRL))

  def engage_onset(self, output_accel, CS):
    if self.long_control_state == LongCtrlState.off:
      # off because the driver is on the gas (override) is not an engagement: releasing the gas gets no soft onset
      self.engage_t, self.onset_t = (ENGAGE_ONSET_WINDOW + 1.0 if getattr(CS, "gasPressed", False) else 0.0), None
      self.onset_done, self.hold, self.hold_peak, self.hold_lead_t = False, 0.0, 0.0, 0.0
      self.bo_t, self.coast_t = None, 0.0
      return output_accel
    self.engage_t += DT_CTRL
    if self.onset_t is None:
      # the onset starts with the first frame that asks for more brake than the last one, below zero, within the window
      if self.engage_t > ENGAGE_ONSET_WINDOW or CS.standstill or self.long_control_state != LongCtrlState.pid or \
         output_accel >= min(self.last_output_accel, 0.0):
        return self.brake_onset(output_accel, CS)
      self.onset_t = 0.0
      # engaged while the car is already slowing (regen after the driver lifted off): start from that decel - from 0
      # the PCM first drops the regen and then bites (see ENGAGE_FROM_DECEL); never firmer than the request
      self.onset_from = max(output_accel, min(float(getattr(CS, 'aEgo', 0.0)), 0.0))
    t = self.onset_t
    self.onset_t += DT_CTRL
    if not self.onset_done:
      jerk = ENGAGE_ONSET_J0 + (ENGAGE_ONSET_J1 - ENGAGE_ONSET_J0) * t / ENGAGE_ONSET_T if t < ENGAGE_ONSET_T else ENGAGE_ONSET_J_STOCK
      base = min(self.last_output_accel, 0.0) if t > 0.0 else min(self.last_output_accel, 0.0, self.onset_from)
      ramp = base - jerk * DT_CTRL
      if output_accel < ramp:
        return ramp
      self.onset_done = True  # request reached: the hold below from here on
    return self.brake_onset(self.engage_hold(output_accel, CS), CS)

  def brake_onset(self, output_accel, CS):
    if self.bo_t is None:
      self.coast_t = 0.0 if self.last_output_accel < BRAKE_ONSET_FROM else self.coast_t + DT_CTRL
      if self.coast_t < BRAKE_ONSET_COAST_T or output_accel >= BRAKE_ONSET_FROM or CS.standstill or \
         self.long_control_state != LongCtrlState.pid:
        return output_accel
      self.bo_t, self.bo_ramp = 0.0, min(self.last_output_accel, 0.0)
    t = self.bo_t
    self.bo_t += DT_CTRL
    if t < ENGAGE_ONSET_T:
      jerk = ENGAGE_ONSET_J0 + (ENGAGE_ONSET_J1 - ENGAGE_ONSET_J0) * t / ENGAGE_ONSET_T
    else:
      jerk = float(np.clip((self._onset_need(CS) + min(self.bo_ramp, 0.0)) / ONSET_T_CATCH, ONSET_J_SLOW, ENGAGE_ONSET_J_STOCK))
    self.bo_ramp -= jerk * DT_CTRL
    if output_accel < self.bo_ramp:
      return self.bo_ramp
    self.bo_t, self.coast_t = None, 0.0  # the planner's request is reached
    return output_accel

  def _onset_need(self, CS) -> float:
    """decel (m/s^2, >= 0) the radar lead needs now; inf = unknown / too close (stock climb)"""
    lead = self.lead
    if lead is None or not lead.status:
      return float('inf')
    room = lead.dRel - ONSET_S0 - CS.vEgo * ONSET_LAG
    if room <= 0.5:
      return float('inf')
    return max(-float(getattr(lead, 'aLeadK', 0.0)), 0.0) + max(CS.vEgo - lead.vLead, 0.0) ** 2 / (2.0 * room)

  def engage_hold(self, output_accel, CS):
    lead = self.lead
    still = lead is not None and lead.status and lead.vLead < ENGAGE_HOLD_LEAD_V
    self.hold_lead_t = self.hold_lead_t + DT_CTRL if still else 0.0
    want = 0.0
    if self.onset_t < ENGAGE_HOLD_MAX_T and not CS.standstill and self.hold_lead_t >= ENGAGE_HOLD_LEAD_T and output_accel < 0.0:
      vt = ENGAGE_HOLD_V_TAIL / 3.6
      room = lead.dRel - CS.vEgo * ENGAGE_HOLD_LAG - ENGAGE_HOLD_GAP - ENGAGE_HOLD_D_TAIL
      b_need = max(CS.vEgo ** 2 - vt ** 2, 0.0) / (2.0 * max(room, 0.5))
      cap = ENGAGE_HOLD_MAX * float(np.interp(CS.vEgo * 3.6, ENGAGE_HOLD_FADE_V, [0.0, 1.0]))
      want = float(np.clip(-b_need - output_accel, -cap, 0.0))
    if want < self.hold:
      self.hold = max(want, self.hold - ENGAGE_HOLD_UP * DT_CTRL)
    else:
      self.hold = min(want, self.hold + ENGAGE_HOLD_DOWN * DT_CTRL)
    # hold, never deepen: never firmer than this stop's own firmest planner request so far
    self.hold_peak = min(self.hold_peak, output_accel)
    return max(output_accel + self.hold, min(self.hold_peak, output_accel))
