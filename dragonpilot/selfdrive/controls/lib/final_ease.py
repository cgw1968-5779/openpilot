"""
dragonpilot (dppb, Corolla Altis): the final ease - the brake is let off only in the last instant of a stop.

Owner 2026-11-18: "the last instant of the stop must not have that must-stop force"; "at the instant the car comes to
rest the force must be back to -0.30 or lighter, never firmer"; "easing too early makes the car coast too far - the key
is easing to -0.30 below 1 km/h, e.g. 0.5 km/h"; "I don't mind -0.4 above 1 km/h, just ease in the very last instant".
The owner explicitly allowed one layer that overrides the existing stop logic for this (11-17).

Why the existing end did not do it (routes 1c9/1ca, 6 stops): the command was held at -0.40 (SOFT_END_HOLD) down to the
wheel-speed floor and only stepped to -0.30 when standstill was flagged, but on the Altis
  - the wheel speeds read nothing below ~0.55-0.6 km/h: standstill is flagged while the car still rolls the last
    centimetres (the smooth 0.5 -> 0 seen in vEgo is only the filter decaying within 0.08 s);
  - the PCM's own request (PCM_CRUISE ACCEL_NET) was 0.15-0.35 firmer than the command and kept firming toward the
    stop (-0.52..-0.76 at the stop with commands -0.30..-0.40), and after the flag -0.53 -> -0.86 within 0.6 s;
  - the car answers the PCM's request ~0.3 s late.
So a step at the flag reached the car after it had stopped, on a PCM request about twice the owner's -0.30.

What it does: below FE_V_TRIG_MAX, while the car is really slowing, it triggers when the car is predicted to be FE_LAG
from FE_V_FEEL (v <= FE_V_FEEL + decel * FE_LAG; always at the wheel-speed floor FE_V_FLOOR). From then the PCM's
request is walked to FE_NET: the command is FE_NET minus what the PCM adds beyond our command (ACCEL_NET - our command
FE_NET_DELAY frames ago, the creep stop's fast loop), eased at most FE_UP and firmed back at most FE_DOWN so the request
ends at FE_NET, not lighter (so the car feels the eased brake at about 0.5 km/h and comes to rest on it); never firmer
than where it started, never firmer than the layers before it, at most FE_CMD_MAX.
Without ACCEL_NET it never arms (a command eased without it lets the creep force push the car on).
At the standstill flag the brake firms back at once at FE_PARK_RATE to FE_PARK (or the pre-ease level if firmer): the car
answers ~0.3 s late, i.e. just after it has come to rest and before the creep force can push it on (11-18 08:00: with a
0.2 s wait and 1.0 m/s^3, every stop was pushed on 0.4-0.6 s after the flag and re-stopped 2-3 times, 0.5-0.8 m closer).
Longcontrol's normal standstill hold (stoppingDecelRate) carries on from there.
Hands back (firming at most FE_REFIRM) if the car speeds up again or goes back above FE_V_ABORT; once per stop
(FE_MAX_USES); not used in an urgent stop (planner firmer than FE_URGENT), while the gas is pressed, while the creep stop
runs its own end, or when longitudinal control is not active. Above FE_V_TRIG_MAX nothing changes.
"""
from collections import deque

import numpy as np

FE_V_FEEL = 0.5 / 3.6       # m/s: the car should feel the eased brake from here to the stop (owner: e.g. 0.5 km/h)
FE_LAG = 0.35               # s: the car answers the PCM's request ~0.3 s late (+ the easing ramp)
FE_V_TRIG_MAX = 1.0 / 3.6   # m/s: never eases above this (owner: below 1 km/h; -0.40 above is fine)
FE_V_FLOOR = 0.6 / 3.6      # m/s: the wheel speeds read nothing below ~0.55-0.6 km/h: at this reading it eases anyway
FE_A_MIN = 0.15             # m/s^2: floor of the decel used for the prediction
FE_SLOWING = -0.08          # m/s^2: the car must be decelerating at least this to trigger
FE_NET = -0.30              # m/s^2: the PCM request the car comes to rest on (owner: -0.30 or lighter)
FE_CMD_MAX = 0.10           # m/s^2: most positive command (the PCM added up to -0.40 beyond ours at the stop)
FE_UP = 1.5                 # m/s^3: easing rate of the command
FE_NET_DELAY = 5            # frames: our command -> ACCEL_NET
FE_DOWN = 2.0               # m/s^3: the command firms back toward the goal at most this fast while moving (the PCM's
                            #   request must not end lighter than FE_NET: 11-18 08:00 it ended at -0.21..-0.24)
FE_PARK_RATE = 4.0          # m/s^3: at the standstill flag the brake firms back at once at this rate (the car answers
                            #   ~0.3 s late = just after it has come to rest; 11-18 08:00 the creep pushed it on 0.4-0.6 s
                            #   after the flag when it waited 0.2 s and firmed at 1.0 m/s^3)
FE_PARK = -0.40             # m/s^2: ... to at least this (or where it was before the ease if firmer)
FE_REFIRM = 1.0             # m/s^3: firming back on a hand-back while moving
FE_V_ABORT = 1.5 / 3.6      # m/s: back above this = not the end of the stop
FE_REACCEL_A = 0.05         # m/s^2: speeding up again (after FE_LAG on) hands back
FE_URGENT = -1.5            # m/s^2
FE_RESET_V = 2.0 / 3.6      # m/s: a new stop can ease again above this
FE_MAX_USES = 1             # eases per stop (11-18 08:00: a second ease after the creep pushed the car on made it stop-go-stop)


class FinalEase:
  def __init__(self, dt: float):
    self.dt = dt
    self.on = False       # easing (moving)
    self.hold = False     # stopped on the eased brake, firming back
    self.done = False     # used (or handed back) for this stop
    self.uses = 0
    self.t = 0.0
    self.cmd = 0.0
    self.before = 0.0     # the command just before the ease (the hold firms back to it)
    self.sent = deque([0.0] * (FE_NET_DELAY + 1), maxlen=FE_NET_DELAY + 1)

  def _back(self, output_accel: float) -> float:
    # hand back without a sudden bite: the command firms toward the other layers' at most FE_REFIRM
    self.cmd = max(output_accel, self.cmd - FE_REFIRM * self.dt)
    if self.cmd <= output_accel + 1e-6:
      self.on = self.hold = False
    return self.cmd

  def update(self, output_accel: float, a_target: float, CS, active: bool, pcm, creep_stop=None) -> float:
    out = self._update(output_accel, a_target, CS, active, pcm, creep_stop)
    self.sent.append(out)
    return out

  def _update(self, output_accel, a_target, CS, active, pcm, creep_stop):
    v = CS.vEgo
    if v > FE_RESET_V:
      self.done, self.uses = False, 0
    creep = creep_stop is not None and (creep_stop.on or creep_stop.holding)
    if not active or pcm is None or getattr(CS, 'gasPressed', False) or creep or a_target < FE_URGENT:
      self.done = self.done or self.on or self.hold
      self.on = self.hold = False
      return output_accel

    if self.hold:
      # standstill flagged: firm back at once to the park level (it reaches the car ~0.3 s later, after it has come to
      # rest, before the creep force can push it on); longcontrol's own hold firms only 0.3 m/s^3
      # then never lighter than it while at rest, until the planner asks to go (like the creep stop's hold)
      if not CS.standstill or a_target > 0.0:
        self.hold = False
        return output_accel
      park = min(self.before, FE_PARK)
      self.cmd = max(park, self.cmd - FE_PARK_RATE * self.dt)
      return min(output_accel, self.cmd)

    if self.on:
      if CS.standstill:
        self.on, self.hold, self.done = False, True, True
        return self._update(output_accel, a_target, CS, active, pcm, creep_stop)
      self.t += self.dt
      if self.done or v > FE_V_ABORT or (self.t > FE_LAG and CS.aEgo > FE_REACCEL_A):
        self.done = True
        return self._back(output_accel)
      extra = min(pcm[0] - self.sent[0], 0.0)   # what the PCM adds beyond our command (never used to firm)
      goal = min(FE_NET - extra, FE_CMD_MAX)
      # toward the goal: eases at most FE_UP, firms back at most FE_DOWN only so the PCM's request does not end lighter
      # than FE_NET (never firmer than where it started, never firmer than the layers before it)
      self.cmd = float(np.clip(goal, self.cmd - FE_DOWN * self.dt, self.cmd + FE_UP * self.dt))
      self.cmd = max(self.cmd, self.before)
      return max(output_accel, self.cmd)

    # trigger: the last instant - the car reaches FE_V_FEEL within FE_LAG (or the wheel speeds hit their floor)
    if self.done or self.uses >= FE_MAX_USES or CS.standstill or v > FE_V_TRIG_MAX or output_accel >= 0.0 or CS.aEgo > FE_SLOWING:
      return output_accel
    decel = max(-CS.aEgo, FE_A_MIN)
    if v <= FE_V_FEEL + decel * FE_LAG or v <= FE_V_FLOOR:
      self.on, self.t, self.uses = True, 0.0, self.uses + 1
      self.before = self.cmd = output_accel
      return self._update(output_accel, a_target, CS, active, pcm, creep_stop)
    return output_accel
