"""
dragonpilot (dpcr, Corolla Cross): the planner's own target for a stop behind a STOPPED lead, shaped like the owner's
manual stops (owner 2026-11-06: "do not slow early at 40-60 km/h; when a firm brake is needed, build it up linearly and
smoothly; at the end, from whatever level, ease off smoothly into a stop with no jolt").

Measured on the C4 (manual stops behind a stopped lead 10-27 21:01, 10-28 09:19 template, 10-29 19:46 vs the same lead
replayed through dpcr a0114478): the MPC starts a light brake ~80-100 m out and its decel shrinks as the gap closes
(09:19 at 15/10/5 m: dpcr 12/8/2 km/h at -0.82/-0.53/-0.37, the owner 23/15/6 km/h at -2.31/-2.42/-1.13), 10 km/h -> stop
~7 s vs the owner's 2.6-4.5 s. The owner waits until the decel needed to stop at the gap reaches ~1.7 m/s^2, builds it
at 1.5-3 m/s^3, holds about that decel, and eases off only in the last ~1.5 s.

This is the planner's stop target (not a layer after it): while the lead is stopped,
  need = v^2 / (2 (dRel - STOP_GAP))   the constant decel that stops exactly at STOP_GAP
  WAIT   need < START_NEED: a light lift (WAIT_ACCEL), and since 11-08 at least K_EARLY x need from the moment the
         stopped lead is seen (owner 2026-11-08: "when a stopped car is seen far ahead it should start slowing down -
         braking still starts too late"; route 000000f3: seen at 122 m at 49 km/h, -0.30 lift to 53 m, -1.8 from 52 m,
         2.3 m. K_EARLY 0.6 at J_EARLY 0.6 m/s^3: point mass 49 km/h / 122 m -0.51 at 100 m, -0.66 at 60, -0.82 at 40,
         mid -1.47 (was -1.82 from 52 m); closed-loop f0 23:09:52 40 m reached at 30 km/h (was 38), peak -1.86 ->
         -1.48, 3.32 -> 3.61 m; f3 3.31 -> 3.44 m; ec 21:49 1.18 -> 2.97 m)
  BRAKE  -K_STOPPED * need (K_NEED for a lead that slowed to a stop, below), built at <= J_BUILD, eased at <= J_EASE
  RELEASE once easing at J_RELEASE would just reach the stopping controller's end line by END_V: ease linearly at
         J_RELEASE onto that line and follow it (never firmer than needed again: no second bite)
Anything else - a moving lead, no lead, an urgent need (>= URGENT_NEED) where the MPC brakes harder, standstill,
disengaged - hands back to the MPC target at <= J_HANDBACK.

A lead that slows to a stop while we already brake for it (owner 2026-11-07, route 000000f0 23:13:04 / 23:14:05:
"the halved brake start needs a little more around 15 km/h, mid toward the end, so it does not stop too close"): the
lead dropped under LEAD_STOPPED_V at 15 km/h / 14 m while the plan was at -1.2; WAIT eased it to -0.30 at J_EASE and
BRAKE never started (need ~0.8), the car ended 2.9 / 2.3 m back. When the stopped lead was tracked MOVING the frame
before and the target is already firmer than WAIT_ACCEL, the approach goes straight to BRAKE (-K_NEED x need, eased at
J_EASE), so it keeps the decel that stops at the gap and ends through the normal RELEASE. A lead first seen already
stopped (the far approach) is unchanged.

Earlier start behind a stopped lead (first experimental only 0383453b; owner 2026-11-07 night for every mode: "stops
behind a stopped car start braking far too late"; route 000000f0 23:09:52, non-experimental, stopped lead seen at 110 m
at 43 km/h, brake from 38.6 m, peak -2.69, stopped 2.4 m then crept to 1.5 m):
  - the start looks T_PRE (1.0 s, was 0.5) ahead: 11-06 21:55 / 11-07 23:09 the brake started at 37-39 m and the send
    side's soft onset lagged ~0.5 s behind it, so the car peaked at -2.6/-2.7;
  - it brakes K_STOPPED (1.05) x the need: with the earlier start K_NEED 1.2 eased the mid brake -1.7 -> -1.1 and the sim
    ended 3.7-3.9 m back; 1.05 holds the mid brake steady (sim 23:09:52 3.53 -> 3.32 m, peak -2.54 -> -1.86, 40 m
    -0.40 -> -1.01; 09:19 template 3.44 -> 3.30 m);
Experimental mode only (owner 2026-11-07: "tuning experimental mode must not touch the non-experimental longitudinal"):
  - WAIT follows the car's real decel: 11-06 21:51 (uphill ~4 %, 8 km/h, lead stopped 13.8 m ahead) the light lift
    -0.30 delivered -0.8 and the car stopped 8.7 m back. While the car slows harder than the need (+ WAIT_FB_MARGIN),
    the lift eases toward 0 at WAIT_FB_RATE; as soon as it does not, it comes back to WAIT_ACCEL at J_BUILD and the
    normal BRAKE start (need >= START_NEED) is unchanged. Never positive (no gas toward a stopped car).

A slow lead braking to a stop close ahead (owner 2026-11-08 13:05, route 000000fa: "the brake before 5 km/h was too
heavy"): a cut-in lead braked from 36 km/h to a stop 4-5 m ahead; it kept rolling at 3-12 km/h until the very end, so
the MPC (which keeps STOP_DISTANCE 6 m) drove the stop and asked -2.7 at 17 km/h, -2.1 at 9.5, -1.5 at 6.4, -1.1 at
4.5 km/h (the car -2.8 / -2.35 / -1.8 / -1.2) - about twice what stopping at the lead's own stop point needed
(-2.2 / -1.5 / -0.8 / -0.5 with K_NEED). Then the car stopped short at 4.4 m, the command dropped to the end line, it
crept forward and stood at 1.8 m. Now a lead slower than V_STOPPING that brakes at least A_STOPPING and will stop within
S_STOPPING is aimed at like a stopped lead, at the point where it will stand (dRel + vLead^2 / (2 aLead)), with the same
phases; the stop point stays in use (aLead floored at A_STOPPING) until the lead stands, speeds up past V_STOPPING + 1 or
accelerates (A_LEAD_GO), and only below V_EGO_STOPPING. Closed loop (sim_stop2 with the recorded lead decel fed to the
MPC as on the car, 25 stops): stopped-lead stops identical; 11 moving-lead stops that ended 4.4-5.5 m back now 3.2-4.5 m
(10-26 22:26:54 5.49 -> 5.30), the decel at 5 km/h lighter (13:05 -1.66 -> -1.43, 10-29 19:39 -1.45 -> -1.19, 10-25
20:39 -1.96 -> -1.52) and the mid brake up to 0.16 firmer.
"""
import numpy as np

from openpilot.common.realtime import DT_MDL

STOP_GAP = 3.4          # m: radar gap the plan aims at; the car ends ~0.2-0.4 m further back (Cross rule 3-3.5 m; owner manual 3.6-3.9 m)
LEAD_STOPPED_V = 1.0    # m/s: lead slower than this counts as stopped
START_NEED = 1.8        # m/s^2: start braking once the needed decel (0.5 s ahead) reaches this (owner's manual: 1.7-1.75)
URGENT_NEED = 2.8       # m/s^2: from here the MPC may brake harder than this target
WAIT_ACCEL = -0.30      # m/s^2: light lift while waiting
K_EARLY = 0.85          # WAIT brakes at least this x the need from the moment the stopped lead is seen (11-08 0.6, owner "more" -> 0.85)
J_EARLY = 0.6           # m/s^3: WAIT firming rate (a soft first touch far out)
K_NEED = 1.2            # brake a little more than needed so the need falls slowly (owner's template ran 1.1-1.3)
J_BUILD = 1.5           # m/s^3: brake build-up (owner 1.5-3)
J_EASE = 0.8            # m/s^3: easing while braking
J_RELEASE = 0.8         # m/s^3: end release (owner's last 1.5 s: <= ~0.9)
J_HANDBACK = 2.0        # m/s^3
END_V = 12.0 / 3.6      # m/s: the release reaches the end line by here (dpcr 5 km/h; dppb 11-20: see END_CURVE)
END_ROOM = 1.0          # share of the end line's own distance (from END_V) the brake phase leaves in front of the stop point
# the Altis's delivery along the end line (delivered = GAIN x request - OFF; tncr1_send/delivery_trim.py's fit), for end_distance()
END_DELIVER_V = [1.8 / 3.6, 4.5 / 3.6, 8.0 / 3.6, 12.5 / 3.6]
END_DELIVER_GAIN = [0.94, 0.85, 0.84, 0.98]
END_DELIVER_OFF = [0.09, 0.06, 0.08, 0.16]
# the stopping controller's end line (tncr1_long/stopping_controller.py END_CURVE), so the release lands on it
END_CURVE_V = [0.0, 1.0 / 3.6, 2.0 / 3.6, 3.0 / 3.6, 5.0 / 3.6, 8.0 / 3.6, 12.0 / 3.6]
END_CURVE_A = [-0.32, -0.32, -0.32, -0.40, -0.60, -0.95, -1.35]  # 11-20: -0.32 held from 2 km/h (was -0.34/-0.42/-0.44/-0.48/-0.65)
# dppb (Corolla Altis Hybrid, owner 2026-11-20 on 22:19:27, route 000001dc: "approaching a standing car the end brake did
# not join the mid brake smoothly"): dpcr holds the mid brake to ~5 km/h and its end line only starts there. 22:19:27:
# -1.4..-1.7 held down to 4.2 km/h, then -1.55 -> -0.43 sent within 0.8 s - a cliff, not a blend - and the Altis brake,
# released that fast, let the car speed up again (1.9 -> 2.65 km/h) and roll from 4.0 m to 1.6 m. The end line now
# starts at 12 km/h (-1.35) and comes down continuously (-0.95 at 8, -0.65 at 5, -0.48 at 3, -0.44 / -0.42 at 2 / 1,
# -0.34 at 0: ~0.4 m/s^3 while slowing), and the mid brake is planned to END ON it (END_ROOM 1: the distance the line
# itself takes, with what the Altis really gives for it - END_DELIVER_*, 8.3 m from 12 km/h - is left in front of the
# stop point). dpcr: END_V 5 km/h, END_ROOM 0, line -0.34 -0.36 -0.37 -0.37 -0.40 -0.45 -0.50 -0.57 -0.65 at 0..8 km/h.
MAX_DIST = 150.0        # m
T_PRE = 1.0             # s: the start looks this far ahead (the build and the car's lag; 0.5 until 11-07)
K_STOPPED = 1.05        # a lead first seen stopped: the earlier start already has margin, so brake closer to the need
WAIT_FB_MARGIN = 0.15   # m/s^2: experimental WAIT eases only while the car slows this much harder than the need
WAIT_FB_RATE = 0.6      # m/s^3: how fast the experimental WAIT lift eases toward 0
# a slow lead that is braking to a stop close ahead: aim at the point where it will stop
V_STOPPING = 4.0        # m/s: lead slower than this
A_STOPPING = 0.5        # m/s^2: and braking at least this (also the floor for its stop distance once latched)
S_STOPPING = 6.0        # m: and it will stop within this distance
V_EGO_STOPPING = 25.0 / 3.6  # m/s: below this ego speed
A_KEEP = 0.3            # m/s^2: a latched lead now braking less than this is not stopping (00000103 16:13:25)
VIRTUAL_PASSED = 0.3    # m: an experimental virtual stop-line lead this close to the aim (or past it) stops on the end line
A_LEAD_GO = 0.3         # m/s^2: a latched lead that accelerates this much is moving again


# dppb (Altis), owner 2026-11-20 after the end was set to -0.32: "if it is going to stop too close, add a little force at
# 10 to 5 km/h". On the end line the room left (to STOP_GAP) is compared with the distance the line itself still needs
# from this speed (the Altis's real delivery, SHORT_LAG s of travel first). Only when the room is short by more than
# SHORT_DEAD is the line scaled up by that ratio - between 10 and 5 km/h, at most SHORT_MAX, built at SHORT_RATE; it
# fades out from 5 to 3 km/h, so the last 3 km/h and the -0.32 end are the plain line. A stop that fits is unchanged.
SHORT_V = [3.0 / 3.6, 5.0 / 3.6, 10.0 / 3.6, 12.0 / 3.6]
SHORT_W = [0.0, 1.0, 1.0, 0.0]
SHORT_DEAD = 0.08       # ratio: room within 8 % of what the line needs = fits
SHORT_MAX = 0.35        # m/s^2: "a little"
SHORT_RATE = 0.6        # m/s^3 in and out
SHORT_LAG = 0.3         # s
SHORT_LIGHT = 0.0       # m/s^2: how much lighter than the line when the stop would end far (0 = never lighter)
SHORT_LIGHT_SHARE = 0.5  # ... and never more than this share of the line itself


def _short_extra(v_ego, d_rel, goal):
  """what the end line is scaled by (as an extra, + = firmer) so that it ends at STOP_GAP from the present speed and gap"""
  w = float(np.interp(v_ego, SHORT_V, SHORT_W))
  if w <= 0.0:
    return 0.0
  room = max(d_rel - STOP_GAP - v_ego * SHORT_LAG, 0.3)
  dev = end_distance(v_ego) / room - 1.0
  dev = max(dev - SHORT_DEAD, 0.0) if dev > 0.0 else min(dev + SHORT_DEAD, 0.0)
  return w * float(np.clip(dev * -goal, max(-SHORT_LIGHT, SHORT_LIGHT_SHARE * goal), SHORT_MAX))


# dppb (Altis), owner 2026-11-20: "3.5 m is the first priority, smooth is second; from any speed the car must come down
# onto a smooth deceleration slope" and "work back from the stop what speed it must have at what distance - then it
# stops smoothly at 3.5 m every time". So ONE slope for the whole stop: SLOPE_A over speed is the shape (firm in the
# middle, easing continuously to -0.32 for the last 2 km/h). Integrated back from the stop point - with what the Altis
# really gives for a request - it is the speed the car must have at every distance. Each cycle the distance the slope
# still needs from the present speed is compared with the room really left to STOP_GAP, and the whole slope is scaled
# by that ratio k: k = 1 the car is on the reference, k > 1 it is too fast for its distance (a little firmer), k < 1 too
# slow (a little lighter). The stop starts when the car reaches the slope (k >= SLOPE_START, looking T_PRE ahead);
# before that the MPC drives. Below SLOPE_W_V the scaling fades out: the last 2 km/h are the plain -0.32.
SLOPE_ENABLED = True
SLOPE_V = [0.0, 1.0 / 3.6, 2.0 / 3.6, 3.0 / 3.6, 5.0 / 3.6, 8.0 / 3.6, 12.0 / 3.6, 20.0 / 3.6, 30.0 / 3.6, 60.0 / 3.6]
# owner 2026-11-21: "do it after my braking pattern". His 16 manual stops marked with three bookmarks (10-01 .. 11-01,
# claude_work/manual3): a light touch built at ~1.4 m/s^3, then -1.4..-1.6 HELD nearly constant down to ~12 km/h / ~10 m
# before the stop point, eased only over the last ~4.6 s, and the last second nearly constant at ~-0.5 to the wheel
# stop. Median real decel by speed: 40 km/h -1.8, 30 -1.6, 20 -1.6, 15 -1.5, 12 -1.35, 8 -1.15, 5 -0.85, 3 -0.85,
# 2 -0.70, 1 -0.50. (The line before: -2.0 at 60, easing from 20 km/h on to -0.32 held from 2 km/h - firm too early,
# released too early, and a long light tail the Altis brake does not hold.) The requests below are those decels through
# the Altis's delivery (above 8 km/h it gives a little less than asked at 8-12 and more above 20).
# 2026-11-21 08:42:59 (route 000001e2, the first road stop on it; owner: "still no good - the brake before the end is too
# heavy and not smooth, and the very end is not eased light enough"): engaged 32 m behind a standing car at 30 km/h, the
# scaled slope still asked -2.14 at 10 km/h (real -2.5), fell to -0.66 within 1 s (the car stopped slowing at 3.5 km/h:
# the Altis after a fast release), then went back to -0.88 (controls -1.14) and ended at -0.45..-0.6. So:
#  - the low end is lighter again and eases all the way: -0.65 at 3 km/h, -0.52 at 2, -0.40 at 1, -0.32 at the stop;
#  - firmer scaling ends higher up (SLOPE_W_V), the easing is slower (SLOPE_J_EASE), a light scale is bounded
#    (SLOPE_K_LIGHT_LOW) and, once the request has begun to ease below SLOPE_EASED_V, it never firms again (owner's
#    data: bad stops re-firm ~0.6 inside the last 2 s, perfect ones <= 0.2) unless the stop is really short (SLOPE_K_REFIRM).
SLOPE_A = [-0.32, -0.40, -0.52, -0.65, -0.85, -1.15, -1.40, -1.50, -1.50, -1.60]
SLOPE_DELIVER_V = [1.8 / 3.6, 4.5 / 3.6, 8.0 / 3.6, 12.5 / 3.6, 20.0 / 3.6, 32.0 / 3.6, 52.0 / 3.6]
SLOPE_DELIVER_GAIN = [0.94, 0.85, 0.84, 0.98, 1.10, 1.18, 1.17]
SLOPE_DELIVER_OFF = [0.09, 0.06, 0.08, 0.16, 0.09, 0.06, 0.04]
SLOPE_START = 0.90      # the stop begins once the room is down to the slope's own distance / this
SLOPE_K = [0.3, 2.5]    # the slope is scaled within this (1.5 until 11-21: the depth is limited by SLOPE_A_MAX now)
SLOPE_W_V = [1.0 / 3.6, 3.0 / 3.6]  # m/s: firmer scaling to 3 km/h, none below 1 - bounded by SLOPE_ENV at every speed
SLOPE_W_V_LIGHT = [1.0 / 3.6, 2.5 / 3.6]  # m/s: lighter scaling fades out below 2.5 km/h (to 1.5 km/h it let the car creep 4 s on -0.19)
SLOPE_LAG = 0.15        # s of travel before a change is felt
# owner 2026-11-21: "the mid decel must be allowed up to -2.6, the first part light as before. What makes a brake
# uncomfortable is the decel changing too fast; reached smoothly, -2.6 or even -2.8 is still not too uncomfortable."
# So the limit is on the RATE, not on the depth: ... down to SLOPE_A_MAX, built at SLOPE_J_BUILD (light at the first touch)
# the firmest the scaled slope may ask at each speed = the firm quarter of the owner's own perfect stops (75th percentile:
# -1.24 at 5 km/h, -1.03 at 3, -0.88 at 2, -0.58 at 1), opening to SLOPE_A_MAX in the middle. A stop that is short is
# made up where he makes it up - a firmer middle that is then eased all the way - not by a heavy brake at 10 km/h.
SLOPE_ENV_V = [0.0, 1.0 / 3.6, 2.0 / 3.6, 3.0 / 3.6, 5.0 / 3.6, 8.0 / 3.6, 12.0 / 3.6, 20.0 / 3.6, 30.0 / 3.6]
SLOPE_ENV_A = [-0.32, -0.50, -0.75, -0.95, -1.20, -1.40, -1.70, -2.20, -2.60]
SLOPE_A_MAX = -2.6      # m/s^2: firmest request of the slope (beyond it: the urgent rule / the MPC)
SLOPE_J_BUILD_A = [0.0, 0.5, 1.0]   # m/s^2 already asked
SLOPE_J_BUILD = [0.8, 1.2, 1.5]     # m/s^3: the build - soft at the first touch, then steady
SLOPE_J_EASE = 1.35     # m/s^3: easing (the owner's own release rate; 2.0 let -2.1 fall to -0.7 in 1 s)
SLOPE_K_LIGHT_LOW = 0.4   # below SLOPE_EASED_V the slope is never scaled lighter than this (0.75: stops ended up to 9 m out)
SLOPE_EASED_V = 12.0 / 3.6  # m/s
SLOPE_STILL_T = 1.0     # s the end value is kept after the wheels stand
SLOPE_K_REFIRM = 1.2    # ... unless the room is this short of what the slope needs


def slope_distance(v_from):
  """distance the slope takes from v_from to rest with the Altis's real delivery"""
  vs = np.linspace(0.0, max(v_from, 0.0), 81)
  a = -np.interp(vs, SLOPE_V, SLOPE_A)
  a = np.maximum(a * np.interp(vs, SLOPE_DELIVER_V, SLOPE_DELIVER_GAIN) - np.interp(vs, SLOPE_DELIVER_V, SLOPE_DELIVER_OFF), 0.1)
  return float(np.sum((vs[1:] ** 2 - vs[:-1] ** 2) / (a[1:] + a[:-1])))


def slope_k(v_ego, d_rel):
  return slope_distance(v_ego) / max(d_rel - STOP_GAP - v_ego * SLOPE_LAG, 0.3)


def _stop_point(lead, latched):
  """the distance at which a slow lead braking to a stop will stand still, or None"""
  a = -float(getattr(lead, 'aLeadK', 0.0))
  if latched:
    if lead.vLead >= V_STOPPING + 1.0 or a <= -A_LEAD_GO or a < A_KEEP:
      return None
    a = max(a, A_STOPPING)
  elif lead.vLead >= V_STOPPING or a < A_STOPPING:
    return None
  s = lead.vLead ** 2 / (2.0 * a)
  if not latched and s > S_STOPPING:
    return None
  return lead.dRel + s


class _Lead:
  def __init__(self, status, d_rel, v_lead):
    self.status, self.dRel, self.vLead = status, d_rel, v_lead


def end_distance(v_from=None):
  """distance the end line takes from v_from (END_V) to rest, the car slowing as the line asks"""
  v_from = END_V if v_from is None else v_from
  vs = np.linspace(0.0, v_from, 81)
  a = -np.interp(vs, END_CURVE_V, END_CURVE_A)
  # what the car really gives for the line (END_DELIVER_*: 1 / 0 = as asked)
  a = np.maximum(a * np.interp(vs, END_DELIVER_V, END_DELIVER_GAIN) - np.interp(vs, END_DELIVER_V, END_DELIVER_OFF), 0.1)
  return float(np.sum((vs[1:] ** 2 - vs[:-1] ** 2) / (a[1:] + a[:-1])))


def _need_plain(v_ego, d_rel):
  return v_ego ** 2 / (2.0 * max(d_rel - STOP_GAP, 0.3))


def _need(v_ego, d_rel):
  # dppb (Altis, 2026-11-20): the brake phase ends ON the end line at END_V, END_ROOM x the line's own distance before
  # the stop point - the constant decel that gets the car from v_ego to END_V by there (END_ROOM 0: dpcr's plain
  # "stop at STOP_GAP at one decel", where the eased end rolls further than the plan counted)
  if END_ROOM <= 0.0:
    return v_ego ** 2 / (2.0 * max(d_rel - STOP_GAP, 0.3))
  if v_ego <= END_V:
    return v_ego ** 2 / (2.0 * max(d_rel - STOP_GAP, 0.3))
  return (v_ego ** 2 - END_V ** 2) / (2.0 * max(d_rel - STOP_GAP - END_ROOM * end_distance(), 0.3))


class StopApproach:
  def __init__(self, dt=DT_MDL):
    self.dt = dt
    self.phase = None  # None (MPC), 'wait', 'brake', 'release', 'handback'
    self.a = 0.0
    self.still_t = 0.0
    self.eased = False        # slope: the request has begun to ease toward the stop (no firming again)
    self.short = 0.0          # extra on the end line while the stop would end short (10-5 km/h)
    self.wait_a = WAIT_ACCEL  # experimental WAIT lift (follows the car's real decel)
    self.lead_moving = False  # the lead was tracked moving the frame before
    self.moved = False        # this approach started from a lead that slowed to a stop (K_NEED, not K_STOPPED)
    self.stopping = False     # the lead is a slow lead braking to a stop close ahead (aimed at its stop point)
    self.via_stopping = False  # this approach started from such a lead (its stop point is already a prediction: K_STOPPED)

  def _step(self, target, up, down):
    """move self.a toward target: up = easing rate (toward 0/positive), down = firming rate"""
    self.a = float(np.clip(target, self.a - down * self.dt, self.a + up * self.dt))
    return self.a

  def update(self, v_ego, lead, a_mpc, off, exp=False, a_ego=None):
    if not off and lead is not None and bool(lead.status) and lead.vLead >= LEAD_STOPPED_V and v_ego < V_EGO_STOPPING:
      d_stop = _stop_point(lead, self.stopping)
    else:
      d_stop = None
    if d_stop is not None:
      lead = _Lead(lead.status, d_stop, 0.0)
    if d_stop is not None and not self.stopping and self.phase in (None, 'handback'):
      self.via_stopping = True
    elif d_stop is None and lead is not None and lead.vLead >= LEAD_STOPPED_V:
      self.via_stopping = False
    self.stopping = d_stop is not None
    stopped_lead = (not off and lead is not None and bool(lead.status) and lead.vLead < LEAD_STOPPED_V
                    and 0.0 < lead.dRel < MAX_DIST
                    # the slope keeps its end value down to the wheel stop: handing back at 0.3 m/s let the plan drop to
                    # -0.1 for the last km/h, and the stopping controller then put -0.32 back on (eased, then re-applied)
                    and (v_ego > 0.3 or (SLOPE_ENABLED and self.phase == 'release' and self.still_t < SLOPE_STILL_T)))
    self.still_t = self.still_t + self.dt if v_ego <= 0.02 else 0.0   # ... and SLOPE_STILL_T longer, until the hold has the car
    was_moving = self.lead_moving
    prev_short, self.short = self.short, 0.0  # self.a is what was asked; the end line's own state is self.a + prev_short
    self.lead_moving = not off and lead is not None and bool(lead.status) and lead.vLead >= LEAD_STOPPED_V
    if not stopped_lead:
      self.wait_a = WAIT_ACCEL
      if self.phase in (None, 'handback') and (self.phase is None or abs(self.a - a_mpc) < 0.05):
        self.phase = None
        self.a = a_mpc
        return a_mpc
      self.phase = 'handback'
      return self._step(a_mpc, J_HANDBACK, J_HANDBACK)

    need = _need(v_ego, lead.dRel)
    if getattr(lead, 'virtual', False) and lead.dRel - STOP_GAP <= VIRTUAL_PASSED:
      # experimental stop line (exp_stop_line.py): the car is at or past the aim. The MPC never sees this virtual lead,
      # so no hand-back; stop on the end line itself - firmed at J_BUILD from a lighter target (the car was not braking
      # yet, e.g. after the driver's gas), eased at J_RELEASE from a firmer one. Never firmer than the end line.
      self.phase = 'release'
      return self._step(float(np.interp(v_ego, END_CURVE_V, END_CURVE_A)), J_RELEASE, J_BUILD)
    if self.phase in (None, 'handback'):
      self.a = a_mpc if self.phase is None else self.a
      self.moved = was_moving and self.a < WAIT_ACCEL
      self.phase = 'brake' if self.moved else 'wait'
    need_pre = _need(v_ego, lead.dRel - v_ego * T_PRE)
    if SLOPE_ENABLED:
      if self.phase in ('wait', 'brake') and slope_k(v_ego, lead.dRel - v_ego * T_PRE) >= SLOPE_START:
        self.phase = 'release'   # on the slope from here to the stop
        self.eased = False
    elif self.phase == 'wait' and need_pre >= START_NEED:
      self.phase = 'brake'

    if SLOPE_ENABLED and self.phase == 'brake':
      self.a = out = a_mpc       # the lead slowed to its stop and the MPC is braking behind it: not on the slope yet
    elif SLOPE_ENABLED and self.phase == 'release':
      shape = float(np.interp(v_ego, SLOPE_V, SLOPE_A))
      k_raw = slope_k(v_ego, lead.dRel)
      k = float(np.clip(k_raw, SLOPE_K[0] if v_ego > SLOPE_EASED_V else SLOPE_K_LIGHT_LOW, SLOPE_K[1]))
      w = float(np.interp(v_ego, SLOPE_W_V if k > 1.0 else SLOPE_W_V_LIGHT, [0.0, 1.0]))  # firmer: never in the last km/h
      target = max(shape * (1.0 + (k - 1.0) * w), float(np.interp(v_ego, SLOPE_ENV_V, SLOPE_ENV_A)))
      if v_ego < SLOPE_EASED_V and target > self.a + 0.05:
        self.eased = True              # the end has begun
      if self.eased and k_raw < SLOPE_K_REFIRM:
        target = max(target, self.a)   # from here the request only eases
      out = self._step(target, SLOPE_J_EASE, float(np.interp(-self.a, SLOPE_J_BUILD_A, SLOPE_J_BUILD)))
    elif self.phase == 'wait':
      if exp and a_ego is not None and a_ego < -(need + WAIT_FB_MARGIN):
        self.wait_a = min(self.wait_a + WAIT_FB_RATE * self.dt, 0.0)
      else:
        self.wait_a = max(self.wait_a - J_BUILD * self.dt, WAIT_ACCEL)
      # not while the experimental feedback eases the lift (the car already slows harder than needed, e.g. uphill)
      early = 0.0 if exp and self.wait_a > WAIT_ACCEL else -K_EARLY * need
      out = self._step(min(max(a_mpc, self.wait_a), early), J_EASE, J_EARLY)
    else:
      goal = float(np.interp(v_ego, END_CURVE_V, END_CURVE_A))
      if self.phase == 'brake':
        a_end = float(np.interp(END_V, END_CURVE_V, END_CURVE_A))
        t_rel = max(a_end - self.a, 0.0) / J_RELEASE
        dv = -(self.a + a_end) / 2.0 * t_rel  # speed lost while easing to the end line
        if v_ego - END_V <= dv or (v_ego < END_CURVE_V[-1] and self.a >= goal):
          self.phase = 'release'
      if self.phase == 'brake':
        out = self._step(-(K_NEED if self.moved else K_STOPPED) * need, J_EASE, J_BUILD)
      else:
        # ease at J_RELEASE onto the end line, then follow it; hold (do not ease further) while the need runs ahead,
        # never firm up again
        base = self.a + prev_short
        target = base if _need_plain(v_ego, lead.dRel) > -base + 0.3 else min(goal, base + J_RELEASE * self.dt)
        base = max(target, base)
        want = _short_extra(v_ego, lead.dRel, goal)
        self.short = float(np.clip(want, prev_short - SHORT_RATE * self.dt, prev_short + SHORT_RATE * self.dt))
        self.a = out = base - self.short
    if _need_plain(v_ego, lead.dRel) >= URGENT_NEED and a_mpc < out:
      self.a = out = a_mpc
    return out
