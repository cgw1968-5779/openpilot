"""
dragonpilot (dppb, Corolla Altis Hybrid): close up from a far stop.

Owner 2026-11-20: "whenever the car stands still more than 4 m behind the lead, creep forward and soft-stop at 3.5 m
+-0.25 m." (11-20 routes 000001da/db: stops at 5.28 m and 6.16 m behind a lead that was still rolling; the car stayed
there until the owner crept up with the gas himself.)

Why the existing plan does not do it: the MPC's own stop distance is 6 m (compiled), so standing 4-6 m behind a
stopped lead it sees no reason to move, and the stop target (stop_approach.py) only shapes a stop that is under way.
This is the planner's target for that one situation (a hook after stop_approach, not a layer after the planner):
  - armed while we stand (v_ego < CU_STILL_V) behind a radar lead at rest (|vLead| < CU_LEAD_V) more than CU_TRIGGER
    away for CU_WAIT (a lead that is about to move is not chased; a lead beyond CU_MAX_D is not this);
  - then the target follows a speed that ends at CU_GAP: v_ref = min(CU_V, sqrt(2 CU_B (room - v CU_LAG))) with
    room = dRel - CU_GAP - the creep is a brake release / light gas (at most CU_A), the stop a light brake (CU_B, at
    most CU_B_MAX), the target moving at most CU_J;
  - should_stop is cleared while it is on (controls would hold the standstill brake otherwise);
  - it ends when the car stands again, and at once (back to the plan) when the lead moves, is lost or jumps, the
    driver brakes / the system is off, or after CU_MAX_T.
Closed-loop sim (claude_work/stopdb/dcu2.py: full chain incl. the real Toyota send side and the fitted Altis plant;
we stop, the lead then rolls on 0.8 / 1.5 / 2.5 / 4 m at 0.25 m/s and stands; 48 runs): none left beyond 4 m (first
version, CU_B 0.18 / CU_LAG 0.8: final gap 5/50/95 % 2.66/3.34/3.65 m).
First road stop 11-20 22:32:28-35 (route 000001dc): started from 6.3 m after 1.5 s, crept to 3 km/h on +0.30, braked from
5.05 m and stood at 4.33 m (the real brake after a release is firmer than the sim's), then rolled on once to 4.0 m
when it handed back -> CU_B 0.42, CU_LAG 0.3, CU_B_MAX 0.35 (the stop starts ~0.5 m later and asks less), CU_END_T.
"""
import math

import numpy as np

CU_GAP = 3.5            # m (radar dRel): where it ends (owner: 3.5 +- 0.25)
CU_TRIGGER = 4.0        # m: standing farther than this starts it (owner)
CU_MAX_D = 15.0         # m
CU_STILL_V = 0.1        # m/s
CU_LEAD_V = 0.3         # m/s: the lead is at rest below this (launch_start's number)
CU_LEAD_GO_V = 0.6      # m/s: the lead moves again -> the plan follows it
CU_WAIT = 1.5           # s
CU_V = 3.0 / 3.6        # m/s: creep speed
CU_A = 0.30             # m/s^2: most it asks to get rolling
CU_B = 0.42             # m/s^2: the decel the stop is planned with. 11-20 22:32:31 (the first one on the road): a -0.44 request at
                        #   3 km/h gave -0.6 and the car stood 0.72 m after the brake began (the sim's car brakes weaker there)
CU_B_MAX = 0.35         # m/s^2: firmest request (a light brake: -0.44 already gave -0.6 on the road)
CU_LAG = 0.3            # s of travel before the brake answers at creep speed (same stop)
CU_K = 1.5              # 1/s
CU_J = 1.0              # m/s^3
CU_END_ROOM = 0.10      # m: this close to the point the stop is the end value's
CU_END_A = -0.40        # m/s^2: request once it is there (holds the Altis: -0.42 or firmer below 2 km/h)
CU_MAX_T = 20.0         # s
CU_END_T = 1.0          # s the end value is kept after the car stands again (22:32:33: handed back at once, +0.23 was sent and it rolled on 0.7 km/h)
CU_JUMP_D = 1.0         # m: a dRel step this large is another lead


class CloseUp:
  def __init__(self, dt: float):
    self.dt = dt
    self.active = False
    self.wait_t = 0.0
    self.t = 0.0
    self.a = 0.0
    self.moved = False
    self.last_d = None
    self.end_t = 0.0

  def _off(self):
    self.active, self.wait_t, self.t, self.moved, self.last_d = False, 0.0, 0.0, False, None

  def update(self, v_ego: float, lead, a_plan: float, off: bool) -> float:
    ok = not off and lead is not None and bool(lead.status) and CU_GAP < lead.dRel < CU_MAX_D
    if not ok:
      self._off()
      self.end_t = 0.0
      return a_plan
    if self.end_t > 0.0:             # just stood again: the end value a moment longer, then the plan's own standstill
      self.end_t = max(self.end_t - self.dt, 0.0)
      return min(a_plan, CU_END_A)
    jumped = self.last_d is not None and abs(lead.dRel - self.last_d) > CU_JUMP_D
    self.last_d = lead.dRel
    if not self.active:
      still = v_ego < CU_STILL_V and abs(lead.vLead) < CU_LEAD_V and lead.dRel > CU_TRIGGER and not jumped
      self.wait_t = self.wait_t + self.dt if still else 0.0
      if self.wait_t < CU_WAIT:
        return a_plan
      self.active, self.t, self.moved, self.a = True, 0.0, False, min(a_plan, 0.0)
    self.t += self.dt
    if lead.vLead > CU_LEAD_GO_V or jumped or self.t > CU_MAX_T:
      self._off()
      return a_plan
    room = lead.dRel - CU_GAP
    self.moved = self.moved or v_ego > 2.0 * CU_STILL_V
    if self.moved and v_ego < CU_STILL_V:
      self._off()                      # standing again: the stop is the plan's (and controls' standstill hold)
      self.end_t = CU_END_T
      return min(a_plan, CU_END_A)
    if room <= CU_END_ROOM:
      target = CU_END_A
    else:
      v_ref = min(CU_V, math.sqrt(2.0 * CU_B * max(room - v_ego * CU_LAG, 0.0)))
      target = float(np.clip(CU_K * (v_ref - v_ego), -CU_B_MAX, CU_A))
    self.a = float(np.clip(target, self.a - CU_J * self.dt, self.a + CU_J * self.dt))
    return self.a
