"""
dragonpilot (dppb, Corolla Altis): the creep stop - a standalone layer for the stop after the driver creeps up with the gas.

Owner 2026-11-15: "after a stop, creeping up with the gas to adjust the gap: when the gas is released the stop is too
heavy (11:26, 16:53); it should feel like the smooth 11:27 three-bookmark stop (from 4.5 km/h: up to -0.73 in 0.4 s,
-0.8..-0.89 for ~1 s, then -0.6, -0.36, stopped)". "Whether it is the MPC or the planner interfering, cut them out and
make a separate creep-stop layer that alone controls the creep below 10 km/h." After route 000001c2 17:20:44 (a
send-side relief made the car creep on toward the lead): "a soft stop, but it MUST stop - never again". Option B (owner
做 2026-11-15): the layer in longcontrol, fed the PCM's own net request (PCM_CRUISE ACCEL_NET) by card.

Why (38 gas releases, Oct-Nov): the planner asks only 0..-0.3 here; ~0.25 s after the release the PCM itself adds brake
on top of the command - after a launch from standstill on average -1.03 (released at 0-2 km/h -1.2..-1.4), after a
re-press while moving -0.30 - it stays in its stop-hold mode at creep speed. aEgo shows it only ~0.3 s later, so a
command that watches aEgo alone is always one bite late; ACCEL_NET shows it at once. But at creep speed the creep force
makes aEgo = ACCEL_NET + 0..+0.66 (17:20:46: ACCEL_NET -0.26, aEgo +0.33), so ACCEL_NET alone let the car speed up.

The PCM also reports its creep force (NEUTRAL_FORCE, 17:20:46 924 N = +0.60 m/s^2) and it has two states: with its
request above about -0.4 it does not brake and the creep pushes the car (17:20:47 ACCEL_NET -0.30, aEgo +0.25); below
about -0.6 its friction brake is on and the car slows a little more than asked (17:20:44.7 -0.82 -> -1.14). A very
light -0.3 stop at 1-3 km/h therefore keeps jumping between the two (sim: two-stage, re-acceleration).

Owner 11-15 (sim of the first version stopped 4-6 m out when the gas was released far back): "stop at 3.25-3.5 m".
So, from the release of the gas (engaged, < CS_V_MAX, a stopped lead within CS_LEAD_D) until standstill:
  - far from the stop position: the PCM's request is held at CS_APPROACH_NET, its no-brake state - the car creeps on by
    the creep force (which fades with speed, so it settles near 4 km/h), like a car rolling off the brake;
  - the one stop starts where CS_BRAKE_B ends at CS_GAP (plus the ramp time and CS_BRAKE_MARGIN; the speed still gained
    over the lag counted). Owner 11-15: "a firmer force to adjust the stop distance belongs before the end braking, not
    at the end below 5 km/h" - so its force is decided there, once: the decel that ends at CS_GAP, as a request within
    CS_NET_MIN..CS_NET_FIRM (usually -0.7, aEgo about -0.85, the 11:27 template; the PCM alone gave -1.2..-1.6), reached
    at CS_ONSET_J (no sudden bite) and held to the stop. Only if the stop would then end more than CS_REPLAN_GAP short
    is it raised (safety);
  - fast loop: what the PCM adds beyond our command (ACCEL_NET - our command CS_NET_DELAY ago) is taken off the command;
  - MUST STOP: never lighter than CS_NET_MIN once stopping (the creep cannot push through); CS_SETTLE_T into the stop a
    car slowing less than CS_FIRM_A firms the request (integral, only ever firmer), speeding up again firms it once by
    CS_REACCEL_STEP at CS_CUT_RATE; never lighter than what stops by CS_MUST_GAP; hands back to the planner (never
    lighter than where it was) when the planner asks for an urgent stop (a_target < CS_URGENT), when stopping by
    CS_MUST_GAP needs more than CS_HAND_B, after CS_MAX_T, or when the lead moves off / is lost. Without the PCM values
    it never arms.
Stops without the gas never arm it; the planner, MPC and the other stop-end layers are not used while it is on.
Sim (physical low-speed plant: the PCM's two states, creep table, 0.3 s lag; 27 releases behind a stopped lead):
all stop, no two-stage; peak aEgo median -1.32 -> -0.98; released more than 4 m out (19): stops 3.05-3.55 m, median
3.37 m (3.05 m: released 4.4-5.1 m out at 3.7-5 km/h - a soft stop has no more room); released inside 3.5 m: 2.1-2.9 m.
"""
from collections import deque

import numpy as np

CS_V_MAX = 10.0 / 3.6     # m/s
CS_LEAD_V = 1.0           # m/s: lead counted as stopped below this
CS_LEAD_D = 15.0          # m
CS_GAP = 3.35             # m (radar dRel): where the stop aims (owner 11-15: stop at 3.25-3.5 m)
CS_MUST_GAP = 2.5         # m: never closer than this
CS_LAG = 0.3              # s travelled before a command takes effect
CS_NET_MIN = 0.7          # m/s^2: the PCM's request is held at least this firm (its friction brake is on; above about
                          #   -0.4 it does not brake and the creep force pushes the car: 17:20:47 ACCEL_NET -0.30, aEgo +0.25)
CS_NET_FIRM = 1.3         # m/s^2: firmest request the stop may be planned with at its start (owner: decide the force before
                          #   the end, not by adding at the end)
CS_BRAKE_DROP = 0.15      # m/s^2: in the brake state aEgo is about this firmer than the request
CS_REPLAN_GAP = 0.25      # m: only if the stop is then predicted this much short of CS_GAP is the force raised (safety)
CS_END_T = 0.45           # s: predicted time to the stop at which the end eases (the PCM answers ~0.3 s late, so the
                          #   car stops on the eased request before the creep force can act on it)
CS_END_NET = -0.35        # m/s^2: the request the end eases to (owner 11-15: 0.05 lighter than -0.40; 11:27 template ended at -0.36)
CS_END_RATE = 2.0         # m/s^3: easing rate
CS_END_ROOM = 0.05        # m: room the eased end takes, given to the mid when the force is decided
CS_HAND_B = 1.5           # m/s^2: stopping by CS_MUST_GAP needs more -> the planner
CS_URGENT = -1.5          # m/s^2: planner request firmer than this -> the planner
CS_NET_DELAY = 5          # frames: our command -> ACCEL_NET
CS_ARM_T = 0.4            # s after the release before speeding-up counts (the gas is still fading)
CS_FIRM_A = -0.2          # m/s^2: after CS_ARM_T the car must decelerate at least this; less firms the request ...
CS_KC = 2.0               # 1/s: ... by this integral (only ever firmer)
CS_C_MAX = 1.0            # m/s^2
CS_REACCEL_A = 0.05       # m/s^2: speeding up again ...
CS_REACCEL_STEP = 0.3     # m/s^2: ... firms the request at once by this
CS_MAX_CMD = 1.2          # m/s^2: most positive command
CS_ONSET_J = 2.0          # m/s^3: the request goes firmer at most this fast (no sudden bite)
CS_UP = 10.0              # m/s^3: command lighter at most this fast (cancelling the PCM's bite; its request stays smooth)
CS_CUT_RATE = 6.0         # m/s^3: firmer when the car speeds up again
CS_MAX_T = 12.0           # s
CS_APPROACH_NET = -0.35   # m/s^2: while far: the PCM's request in its no-brake state, the car creeps on (to ~4 km/h)
CS_BRAKE_B = 0.9          # m/s^2: the one stop is started where this decel (what the -0.7..-0.85 request delivers) ends at CS_GAP ...
CS_BRAKE_MARGIN = 0.10    # m: ... this much early
CS_BRAKE_RAMP_T = 0.2     # s: time the request takes to reach the stop decel (counted in the start point)
CS_SETTLE_T = 0.6         # s after the stop starts before a weak decel / speeding up counts (lag + ramp)
CS_HOLD_A = -0.30         # m/s^2: into the stop hold no lighter than this


def set_pcm_ext(ext, car_state) -> None:
  # card: publish the PCM's own net request and creep force (Toyota) in carStateExt
  net, nf = getattr(car_state, 'pcm_accel_net', None), getattr(car_state, 'pcm_neutral_force', None)
  ext.pcmValid = net is not None and nf is not None
  if ext.pcmValid:
    ext.pcmAccelNet, ext.pcmNeutralForce = float(net), float(nf)


def pcm_from_ext(ext):
  # controlsd: (ACCEL_NET, NEUTRAL_FORCE) for the creep stop, None when the car does not report them
  if ext is None or not ext.pcmValid:
    return None
  return float(ext.pcmAccelNet), float(ext.pcmNeutralForce)


class CreepStop:
  def __init__(self, dt: float, mass: float):
    self.dt = dt
    self.mass = max(float(mass), 800.0)
    self.armed = False
    self.on = False
    self.t = 0.0
    self.c = 0.0
    self.net_t = 0.0
    self.brake = False  # the one stop has started (latched)
    self.tb = 0.0       # s since it started
    self.was_reaccel = False
    self.level = 0.0    # the stop's request, decided once at its start
    self.end = False    # the end is easing
    self.end_used = False  # the end eased once already (a speeding-up cancels it for good)
    self.holding = False   # stopped by the creep stop: held until the gas / the lead moves off
    self.cmd = 0.0
    self.sent = deque([0.0] * (CS_NET_DELAY + 1), maxlen=CS_NET_DELAY + 1)

  def _off(self):
    self.on = False
    self.armed = False

  def update(self, output_accel: float, CS, lead, a_target: float, active: bool, pcm) -> float:
    self.sent.append(output_accel)  # replaced below while on
    if getattr(CS, 'gasPressed', False):
      self.armed = CS.vEgo < CS_V_MAX and pcm is not None
      self.on = self.holding = False
      return output_accel
    if self.holding:
      # stopped behind the still-stopped lead: never a request lighter than the hold, so the creep cannot move the car
      if active and lead is not None and lead.status and lead.vLead < CS_LEAD_V and a_target < 0.3:
        return min(output_accel, CS_HOLD_A)
      self.holding = False
    if not active or pcm is None or CS.vEgo >= CS_V_MAX or lead is None or not lead.status or \
       lead.vLead >= CS_LEAD_V or lead.dRel > CS_LEAD_D:
      self._off()
      return output_accel
    pcm_net, _nf = pcm
    if not self.on:
      if not self.armed or CS.standstill:
        return output_accel
      self.on, self.armed, self.t, self.c, self.brake, self.tb, self.end, self.end_used = True, False, 0.0, 0.0, False, 0.0, False, False
      self.cmd = min(self.sent[-2], 0.0)  # from what was sent last (no step at the hand-over)
      self.net_t = min(pcm_net, 0.0)
    if CS.standstill:
      self._off()
      self.holding = True
      return min(output_accel, CS_HOLD_A)
    self.t += self.dt

    v = CS.vEgo
    must_room = lead.dRel - CS_MUST_GAP - v * CS_LAG
    b_must = v ** 2 / (2.0 * must_room) if must_room > 0.05 else 99.0
    if a_target < CS_URGENT or b_must > CS_HAND_B or self.t > CS_MAX_T:
      self._off()
      return min(output_accel, self.cmd)
    # the speed the car will still gain over the lag (the gas fading out / the creep) counts when planning the stop
    vp = v + max(CS.aEgo, 0.0) * CS_LAG
    room = lead.dRel - CS_GAP - 0.5 * (v + vp) * CS_LAG
    if not self.brake and room <= vp ** 2 / (2.0 * CS_BRAKE_B) + vp * CS_BRAKE_RAMP_T + CS_BRAKE_MARGIN:
      self.brake = True
      # the force is decided here, once, for the whole stop: the decel that ends at CS_GAP after the ramp
      stop_room = max(room - vp * CS_BRAKE_RAMP_T - CS_END_ROOM, 0.05)
      b_need = vp ** 2 / (2.0 * stop_room)
      self.level = float(np.clip(b_need - CS_BRAKE_DROP, CS_NET_MIN, CS_NET_FIRM))
    reaccel = False
    if not self.brake:
      # still far: creep on toward the stop position (PCM request in its no-brake state, the creep force moves the car)
      goal = CS_APPROACH_NET
    else:
      self.tb += self.dt
      # safety only: raise the force if this stop would now end more than CS_REPLAN_GAP short of CS_GAP
      pred_gap = lead.dRel - v * CS_LAG - v ** 2 / (2.0 * (self.level + CS_BRAKE_DROP))
      if pred_gap < CS_GAP - CS_REPLAN_GAP:
        self.level = min(self.level + CS_ONSET_J * self.dt, CS_HAND_B)
      b_t = max(self.level, b_must + 0.1)
      # the car must keep slowing: weaker than CS_FIRM_A (once the stop has settled) firms it, never lighter again
      settled = self.tb > CS_SETTLE_T and self.t > CS_ARM_T
      reaccel = settled and CS.aEgo > CS_REACCEL_A
      if settled:
        step = CS_REACCEL_STEP if (reaccel and not self.was_reaccel) else 0.0   # once per speeding-up
        self.c = min(self.c + CS_KC * max(CS.aEgo - CS_FIRM_A, 0.0) * self.dt + step, CS_C_MAX)
      self.was_reaccel = reaccel
      goal = -(b_t + self.c)
      # end: ease to CS_END_NET once the stop is CS_END_T away; any speeding-up ends the easing at once
      t_left = v / max(self.level + CS_BRAKE_DROP, 0.3)
      # only a stop that is really under way eases its end (a short stop from creep speed keeps its force to the end)
      if not self.end and not self.end_used and settled and CS.aEgo <= -0.8 * (self.level + CS_BRAKE_DROP) and \
         t_left <= CS_END_T and b_must + 0.1 <= self.level:
        self.end = self.end_used = True
      if self.end and (reaccel or CS.aEgo > CS_REACCEL_A):
        self.end = False   # for good: the rest of this stop at the decided force
      if self.end:
        goal = max(goal, CS_END_NET)
    # the PCM's request walks to the goal no faster than CS_ONSET_J (from where it was at the release)
    up = CS_END_RATE if self.end else CS_UP
    self.net_t = float(np.clip(goal, self.net_t - (CS_CUT_RATE if reaccel else CS_ONSET_J) * self.dt, self.net_t + up * self.dt))
    # fast loop: take off what the PCM adds beyond our command
    extra = pcm_net - self.sent[0]
    target = float(np.clip(self.net_t - extra, -3.5, CS_MAX_CMD))
    self.cmd = float(np.clip(target, self.cmd - CS_CUT_RATE * self.dt, self.cmd + CS_UP * self.dt))
    self.sent[-1] = self.cmd
    return self.cmd
