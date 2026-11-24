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
from dragonpilot.selfdrive.controls.lib.tncr1_long.end_force_limit import FEEL_A, FEEL_BP, FEEL_V

STOP_GAP = 3.5          # m: radar gap the stop ends at (owner 2026-11-23: 3.5 m; 3.2 until then, when the eased end was open loop)
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
END_V = 5.0 / 3.6       # m/s: the release reaches the end line by here
# the stopping controller's end line (tncr1_long/stopping_controller.py END_CURVE), so the release lands on it
END_CURVE_V = [0.0, 1.0 / 3.6, 2.0 / 3.6, 3.0 / 3.6, 4.0 / 3.6, 5.0 / 3.6, 6.0 / 3.6, 7.0 / 3.6, 8.0 / 3.6]
END_CURVE_A = [-0.34, -0.36, -0.37, -0.37, -0.40, -0.45, -0.50, -0.57, -0.65]
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

# ONE deceleration slope worked back from the stop point (dppb 2026-11-20..22, owner 2026-11-23: "apply the principle of
# the dppb end-brake work to dpcr too; dpcr is already close to good and only needs the same kind of fine tuning; the
# stop is to be held at 3.5 m"). SLOPE_A over speed is the shape of the stop - the Cross's own: the median plan of the
# dpcr road stops that ended 3.2-3.8 m back (11-05..11-09: -1.7 at 20-30 km/h, -1.2 at 12-16, -0.87 at 8-10, -0.6 at
# 4-6, then the stopping controller's end line -0.37 .. -0.34). Integrated back from the stop point, with what the Cross
# gives for a plan (SLOPE_DELIVER_*), it is the speed the car must have at every distance. Each cycle the distance the
# slope still needs from the present speed is compared with the room really left to STOP_GAP and the whole slope is
# scaled by that ratio k (k > 1: too fast for the distance, a little firmer; k < 1: a little lighter) - where the plan
# before aimed a constant decel at STOP_GAP and then eased OPEN LOOP onto the end line (the eased end rolled on:
# closed-loop sim of the 58 recorded starts 2.0 / 2.9 / 4.3 m). The stop starts when the car reaches the slope
# (k >= SLOPE_START, looking T_PRE ahead); until then WAIT is unchanged. The scaling fades out below SLOPE_W_V: the last
# km/h are the plain end line. Once the request has begun to ease below SLOPE_EASED_V it only eases (no second bite)
# unless the stop is really short (SLOPE_K_REFIRM), and the end value is kept until the wheels stand + SLOPE_STILL_T.
SLOPE_ENABLED = True
SLOPE_V = [0.0, 1.0 / 3.6, 2.0 / 3.6, 3.0 / 3.6, 4.0 / 3.6, 5.0 / 3.6, 8.0 / 3.6, 12.0 / 3.6, 20.0 / 3.6, 30.0 / 3.6, 60.0 / 3.6]
SLOPE_A = [-0.34, -0.36, -0.37, -0.37, -0.48, -0.58, -0.82, -1.10, -1.50, -1.75, -1.75]
# what the Cross gives for a plan on such a stop: real decel = GAIN x plan + ADD (closed-loop sim with the real send
# side, dpcr_1123/calib2.py). Between 9 and 30 km/h the send side's fixed additions (mid-speed add, delivery trim) do
# not shrink with a light plan: 0.7-0.8 x plan + 0.3-0.4, so -0.6 asked gives -0.8. Below 4.5 km/h the Cross gives
# 1.1-1.3 x the request (rlogs: real / sent 1.12-1.16 below 3 km/h), and what it gives is what was asked
# SLOPE_ANSWER_T earlier - with both the eased end from 5 km/h is 1.7 m, not 2.3. The first version (5fb6efa33) took
# the map as unity and k as a plain ratio:
# a car that came in light (k 0.5-0.7) was slowed more than planned down to 9 km/h, the scale fell to its floor, the
# plan went to -0.15..-0.18 - lighter than the end line - and, only allowed to ease from there, let the car creep 5-8 s
# at 1-3 km/h past the stop point (sim cases 33 / 32 / 21: 2.0 / 2.6 / 1.7 m).
SLOPE_DELIVER_V = [1.2 / 3.6, 2.3 / 3.6, 4.5 / 3.6, 6.5 / 3.6, 9.0 / 3.6, 12.0 / 3.6, 16.0 / 3.6, 22.0 / 3.6, 30.0 / 3.6]
SLOPE_DELIVER_GAIN = [1.2, 1.3, 1.1, 0.95, 0.80, 0.68, 0.69, 0.79, 0.80]
SLOPE_DELIVER_ADD = [0.0, 0.0, 0.0, 0.05, 0.25, 0.41, 0.41, 0.30, 0.27]
SLOPE_ANSWER_T = 0.35   # s: the brake's answer time at low speed (plant fit: 0.4 s at 2 km/h, 0.3 at 4.5, 0.2 from 8)
SLOPE_LIGHT_FLOOR = -0.25   # m/s^2: a light scale never takes the request above this (the Cross creeps on -0.25)
SLOPE_START = 0.90      # the stop begins once the room is down to the slope's own distance / this
SLOPE_K = [0.3, 2.5]    # the slope is scaled within this
SLOPE_W_V = [1.0 / 3.6, 3.0 / 3.6]        # m/s: firmer scaling to 3 km/h, none below 1 - bounded by SLOPE_ENV at every speed
SLOPE_W_V_LIGHT = [1.0 / 3.6, 2.5 / 3.6]  # m/s: lighter scaling fades out below 2.5 km/h
SLOPE_LAG = 0.0         # s of travel before a change is felt (the answer time is in slope_distance: SLOPE_ANSWER_T)
# the firmest the scaled slope may ask at each speed (the end ceiling of tncr1_long/end_force_limit.py below 3 km/h)
SLOPE_ENV_V = [0.0, 1.0 / 3.6, 2.0 / 3.6, 3.0 / 3.6, 5.0 / 3.6, 8.0 / 3.6, 12.0 / 3.6, 20.0 / 3.6, 30.0 / 3.6]
SLOPE_ENV_A = [-0.34, -0.36, -0.37, -0.37, -1.00, -1.40, -1.80, -2.30, -2.60]
SLOPE_ENV_HARD_A = [-0.34, -0.36, -0.37, -0.37, -1.50, -2.00, -2.50, -2.60, -2.60]
SLOPE_K_HARD = 1.5      # a stop this short of room (late cut-in, late engagement) may use the hard bound
SLOPE_J_BUILD_A = [0.0, 0.5, 1.0]   # m/s^2 already asked
SLOPE_J_BUILD = [1.5, 1.5, 1.5]     # m/s^3: the build = J_BUILD, dpcr's own (0.8 at the first touch cost a short stop 0.5 s: sim 36 km/h / 37.6 m 3.0 -> 2.2 m)
SLOPE_J_EASE = 0.9      # m/s^3: easing (owner's Cross release: <= ~0.9)
SLOPE_EASED_V = 12.0 / 3.6  # m/s
SLOPE_EASED_D = 0.005   # m/s^2 the target is above the request for the end to count as begun
SLOPE_STILL_T = 1.0     # s the end value is kept after the wheels stand
SLOPE_K_REFIRM = 1.2    # ... unless the room is this short of what the slope needs
SLOPE_WAIT_NEED = 1.0   # WAIT's light lift is at most this x the decel needed to stop at STOP_GAP
# The stop after the driver's gas creep (owner 2026-11-23: "dpcr seemed to have it already; if it is gone, put it back,
# and it must be soft enough"). dpcr never had a stage of its own for it (dppb's creep_stop.py is the Altis's); what it
# had was the send side's soft brake onset. Sim of a gas release at 5-9 km/h, 4-6 m behind a standing car: -1.2..-2.3
# built at 2 m/s^3 on 8627c39d3 and on the slope alike (the short-stop scaling, its hard bound, the urgent hand-over to
# the MPC) - and the bite buys nothing, the brake's answer time decides where such a stop ends (5 km/h at 4.5 m: 2.4 m
# with -1.16, 2.6 m with -0.7). So from a gas press below CREEP_V until the car stands (or is faster than CREEP_V_OFF):
# the request starts from ~0, builds at CREEP_J and is never firmer than CREEP_SOFT - unless even that would end closer
# than CREEP_MIN_GAP to the lead; then it is the decel that ends there (a soft stop, but it must stop). Smooth comes
# before the distance here: a release closer than ~5.5 m ends inside 3.5 m.
CREEP_V = 10.0 / 3.6        # m/s
CREEP_V_OFF = 12.0 / 3.6    # m/s
CREEP_SOFT = 0.60           # m/s^2 asked (the Cross gives 1.1-1.3 x at these speeds: ~-0.75 felt)
CREEP_J = 1.0               # m/s^3
CREEP_MIN_GAP = 1.8         # m
CREEP_HARD_GAP = 1.3        # m: the decided force is raised only if the stop would end inside this
CREEP_GIVE_UP = 1.0          # m/s^2: a stop that needs more than this is not a soft stop - the ordinary one takes it
CREEP_ONSET_T = 0.5         # s the send side's soft onset takes before the brake is really on (counted when the force is decided)
SLOPE_REFIRM_MAX = 0.15  # m/s^2: the most a short stop is made up once the end has begun ...
SLOPE_REFIRM_J = 0.6     # m/s^3: ... and how fast
SLOPE_SAFE_GAP = 2.2     # m: ... unless the stop would end inside this
SLOPE_REFIRM_V = 5.0 / 3.6  # m/s: ... and only above this speed (below it nothing is added for the distance)


def slope_request(v, k, hard=False):
  """the slope scaled by k at speed(s) v: firmer fades out below SLOPE_W_V and is bounded by SLOPE_ENV, lighter fades
  out below SLOPE_W_V_LIGHT (the end of the stop is always the plain slope) and never goes above SLOPE_LIGHT_FLOOR"""
  shape = np.interp(v, SLOPE_V, SLOPE_A)
  w = np.interp(v, SLOPE_W_V if k > 1.0 else SLOPE_W_V_LIGHT, [0.0, 1.0])
  a = np.maximum(shape * (1.0 + (k - 1.0) * w), np.interp(v, SLOPE_ENV_V, SLOPE_ENV_HARD_A if hard else SLOPE_ENV_A))
  return np.minimum(a, np.maximum(shape, SLOPE_LIGHT_FLOOR))


def slope_deliver(v, request):
  """the decel the car gives for a request (<= 0) at speed v, by the fit"""
  a = -request * np.interp(v, SLOPE_DELIVER_V, SLOPE_DELIVER_GAIN) + np.interp(v, SLOPE_DELIVER_V, SLOPE_DELIVER_ADD)
  # the last km/h is eased to the feel line when the car gives more (end_force_limit.py FEEL_*): the stop takes that
  # much longer, so the slope counts on it and begins that much earlier. Only the lower FEEL_PLAN of the feel window: the
  # ease grows at FEEL_RATE and the car answers ~0.4 s late, so the upper part is still on what the request gives.
  return np.where(np.asarray(v) < FEEL_PLAN * FEEL_V, np.minimum(a, -np.interp(v, FEEL_BP, FEEL_A)), a)


def slope_distance(v_from, k=1.0):
  """distance the slope scaled by k takes from v_from to rest with what the car really gives"""
  vs = np.linspace(0.0, max(v_from, 0.0), 81)
  hard = k >= SLOPE_K_HARD   # the same bound the request itself will use
  a = -slope_request(vs, k, hard)
  a = -slope_request(vs + a * SLOPE_ANSWER_T, k, hard)   # the car gives what was asked SLOPE_ANSWER_T earlier, when it was faster
  a = np.maximum(slope_deliver(vs, -a), 0.1)
  return float(np.sum((vs[1:] ** 2 - vs[:-1] ** 2) / (a[1:] + a[:-1])))


def slope_k(v_ego, d_rel, gap=None):
  """the scale at which the slope, with what the car really gives, ends at STOP_GAP (gap) from this speed and distance"""
  room = max(d_rel - (STOP_GAP if gap is None else gap) - v_ego * SLOPE_LAG, 0.3)
  lo, hi = 0.1, 4.0
  if slope_distance(v_ego, hi) >= room:
    return hi
  if slope_distance(v_ego, lo) <= room:
    return lo
  for _ in range(12):
    mid = 0.5 * (lo + hi)
    if slope_distance(v_ego, mid) > room:
      lo = mid
    else:
      hi = mid
  return 0.5 * (lo + hi)


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


# Crawl-on (owner 2026-11-24, C4 road test of 5cfac4a3d: "creeping up to a standing lead at low speed it stops too far
# back - let the brake go and add only a short touch of gas, a little push forward"). Route 0000000a 23:14:13: 5 km/h
# with 9.2 m to the lead, the request stayed -0.52 .. -0.19 (the slope's light floor, the MPC's own 6 m stop) and the
# Cross, which gives -0.3 .. -0.45 for that below 4 km/h, stood at 5.6 m; 22:55:03 5.7 m, 09:09:00 5.9 m, and the same
# on the versions before the slope (08:55 6.4 m, 08:58 6.2 m). The end of a stop from 3 km/h takes ~1.2 m: while the
# car is slower than CRAWL_V_IN, still rolling, and has more than that plus CRAWL_ROOM_ON left to STOP_GAP, the request
# is a speed hold at CRAWL_V instead of a brake - mostly the released brake (the hybrid's own creep carries it), a
# little positive only while the car is below CRAWL_V - changed at CRAWL_J, never a step. Once the room is down to the
# slope's own distance (+ CRAWL_ROOM_OFF) the slope takes the stop as before. Never from a standstill: a car that has
# stopped stays stopped (owner 11-20).
FEEL_PLAN = 0.7            # of end_force_limit.FEEL_V: below this the slope counts on the eased end (slope_deliver)
CRAWL_V = 3.0 / 3.6        # m/s: the speed held on the way to the stop point
CRAWL_V_IN = 5.0 / 3.6     # m/s: only below this (a faster approach is the slope's)
CRAWL_V_MIN = 0.35         # m/s: only while still rolling
CRAWL_ROOM_ON = 1.0        # m more than the slope needs from the present speed: crawl on
CRAWL_ROOM_OFF = 0.30      # m more than the slope needs (+ CRAWL_T_OFF of travel): hand over to the slope
CRAWL_T_OFF = 0.30         # s: the slope's request builds from the released brake, the car rolls on meanwhile
CRAWL_KP = 0.6             # 1/s
CRAWL_A = [-0.30, 0.25]    # m/s^2: the speed hold's request range ("a little push", never a launch)
CRAWL_J = 0.6              # m/s^3


RADAR_LAG = 0.19   # s. What radarState says is this old: over 246 recorded stops behind a standing lead (65 Cross, 181 Altis)
                   # dRel was 0.19 s x speed MORE than the stop distance + the wheel odometry (0.2 m at 4 km/h, 0.5 m at
                   # 10 km/h, 1.3 m at 30 km/h), and the lead's plate in the road camera (no lag) agreed with the odometry.
                   # Planned on the late distance the slope is set for a stop point ~0.3 m too far and the scale k
                   # climbs as the car slows. The stop target uses the distance as it is now.


class _Now:
  """the radar lead with dRel brought forward by RADAR_LAG at the closing speed (everything else as radard gives it)"""

  def __init__(self, lead, v_ego):
    self._lead = lead
    self.dRel = max(lead.dRel - max(v_ego - lead.vLead, 0.0) * RADAR_LAG, min(lead.dRel, 0.05))

  def __getattr__(self, name):
    return getattr(self._lead, name)


class _Lead:
  def __init__(self, status, d_rel, v_lead):
    self.status, self.dRel, self.vLead = status, d_rel, v_lead


def _need(v_ego, d_rel):
  return v_ego ** 2 / (2.0 * max(d_rel - STOP_GAP, 0.3))


class StopApproach:
  def __init__(self, dt=DT_MDL):
    self.dt = dt
    self.phase = None  # None (MPC), 'wait', 'brake', 'release', 'handback'
    self.a = 0.0
    self.crawl = False
    self.still_t = 0.0        # slope: time the wheels have been standing
    self.eased = False        # slope: the request has begun to ease toward the stop (no firming again)
    self.eased_a = 0.0        # slope: the lightest request since the end began
    self.creep = False        # the driver crept up with the gas: the stop that follows is the soft one (CREEP_*)
    self.creep_cap = None     # ... its firmest decel, decided when it begins
    self.wait_a = WAIT_ACCEL  # experimental WAIT lift (follows the car's real decel)
    self.lead_moving = False  # the lead was tracked moving the frame before
    self.moved = False        # this approach started from a lead that slowed to a stop (K_NEED, not K_STOPPED)
    self.stopping = False     # the lead is a slow lead braking to a stop close ahead (aimed at its stop point)
    self.via_stopping = False  # this approach started from such a lead (its stop point is already a prediction: K_STOPPED)

  def _creep_cap(self, v_ego, d_rel):
    """the firmest decel of the soft stop: decided ONCE, when the stop begins - CREEP_SOFT, or what it takes to end
    CREEP_MIN_GAP behind the lead from there - and raised afterwards only if the stop would end inside CREEP_HARD_GAP"""
    to = lambda gap: v_ego ** 2 / (2.0 * max(d_rel - gap - v_ego * (SLOPE_ANSWER_T + (CREEP_ONSET_T if self.creep_cap is None else 0.0)), 0.3))
    if self.creep_cap is None:
      self.creep_cap = max(CREEP_SOFT, to(CREEP_MIN_GAP))
    return max(self.creep_cap, to(CREEP_HARD_GAP))

  def _step(self, target, up, down):
    """move self.a toward target: up = easing rate (toward 0/positive), down = firming rate"""
    self.a = float(np.clip(target, self.a - down * self.dt, self.a + up * self.dt))
    return self.a

  def update(self, v_ego, lead, a_mpc, off, exp=False, a_ego=None, gas=False):
    if gas and v_ego < CREEP_V:
      self.creep, self.creep_cap = True, None
    elif v_ego <= 0.02 or v_ego > CREEP_V_OFF:
      self.creep, self.creep_cap = False, None
    if RADAR_LAG > 0.0 and lead is not None and bool(lead.status) and not getattr(lead, 'virtual', False):
      lead = _Now(lead, v_ego)
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
                    # the slope keeps its end value down to the wheel stop (a hand-back at 0.3 m/s is an ease, then the
                    # stopping controller's re-application)
                    and (v_ego > 0.3 or (SLOPE_ENABLED and self.phase == 'release' and self.still_t < SLOPE_STILL_T)))
    self.still_t = self.still_t + self.dt if v_ego <= 0.02 else 0.0   # ... and SLOPE_STILL_T longer, until the hold has the car
    was_moving = self.lead_moving
    self.lead_moving = not off and lead is not None and bool(lead.status) and lead.vLead >= LEAD_STOPPED_V
    if not stopped_lead:
      self.crawl = False
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
      if self.creep:
        self.a = float(np.clip(self.a, -0.1, 0.0))   # the soft stop builds from the released gas, not from the MPC's first answer
      self.moved = was_moving and self.a < WAIT_ACCEL
      self.phase = 'brake' if self.moved else 'wait'
    if SLOPE_ENABLED and CRAWL_V > 0.0:
      spare = lead.dRel - STOP_GAP - slope_distance(max(v_ego, CRAWL_V) if self.crawl else v_ego, 1.0)
      if self.crawl:
        self.crawl = spare > CRAWL_ROOM_OFF + v_ego * CRAWL_T_OFF and v_ego > CRAWL_V_MIN and not self.creep
        if not self.crawl:
          self.phase, self.eased = 'release', False   # the slope from here to the stop
      elif (self.phase in ('wait', 'release') and CRAWL_V_MIN < v_ego < CRAWL_V_IN and spare > CRAWL_ROOM_ON
            and not self.creep and not getattr(lead, 'virtual', False)):
        self.crawl = True
      if self.crawl:
        return self._step(float(np.clip(CRAWL_KP * (CRAWL_V - v_ego), CRAWL_A[0], CRAWL_A[1])), CRAWL_J, CRAWL_J)
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
      k_raw = slope_k(v_ego, lead.dRel)
      k = float(np.clip(k_raw, SLOPE_K[0], SLOPE_K[1]))
      target = float(slope_request(v_ego, k, hard=k_raw >= SLOPE_K_HARD))
      j_build = float(np.interp(-self.a, SLOPE_J_BUILD_A, SLOPE_J_BUILD))
      if v_ego < SLOPE_EASED_V and target > self.a + SLOPE_EASED_D and not self.eased:
        self.eased, self.eased_a = True, self.a   # the end has begun
      if self.eased:
        self.eased_a = max(self.eased_a, self.a)  # the lightest the request has been since
        if k_raw < SLOPE_K_REFIRM or v_ego < SLOPE_REFIRM_V:
          target = max(target, self.a)            # from here the request only eases
        elif slope_k(v_ego, lead.dRel, SLOPE_SAFE_GAP) < 1.0:
          # a stop that turns out short is made up a LITTLE and slowly (owner: "if it is going to stop too close, add a
          # little force at 10 to 5 km/h") - not put back on: sim case 5, the car gave half of -0.55 at 6 km/h for 1.2 s
          # and the plan went -0.54 -> -1.29 in 0.5 s at 5 km/h, the "heavy first, then eases". The full slope again
          # only when the stop would otherwise end inside SLOPE_SAFE_GAP.
          target, j_build = max(target, self.eased_a - SLOPE_REFIRM_MAX), SLOPE_REFIRM_J
      if self.creep and self._creep_cap(v_ego, lead.dRel) > CREEP_GIVE_UP:
        self.creep = False             # no room for a soft stop: the ordinary stop (and the urgent rule) from here
      if self.creep:
        target, j_build = max(target, -self._creep_cap(v_ego, lead.dRel)), CREEP_J
      out = self._step(target, SLOPE_J_EASE, j_build)
    elif self.phase == 'wait':
      if exp and a_ego is not None and a_ego < -(need + WAIT_FB_MARGIN):
        self.wait_a = min(self.wait_a + WAIT_FB_RATE * self.dt, 0.0)
      else:
        self.wait_a = max(self.wait_a - J_BUILD * self.dt, WAIT_ACCEL)
      # not while the experimental feedback eases the lift (the car already slows harder than needed, e.g. uphill)
      early = 0.0 if exp and self.wait_a > WAIT_ACCEL else -K_EARLY * need
      # slope: the light lift is never more than the decel that stops at STOP_GAP - at crawl speed -0.30 is a real brake
      # (5 km/h, 10 m: need 0.15) and ended such stops 4.3 m (up to 7 m) back, before the slope was ever reached
      lift = max(self.wait_a, -need * SLOPE_WAIT_NEED) if SLOPE_ENABLED else self.wait_a
      out = self._step(min(max(a_mpc, lift), early), J_EASE, J_EARLY)
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
        target = self.a if need > -self.a + 0.3 else min(goal, self.a + J_RELEASE * self.dt)
        self.a = out = max(target, self.a)
    if self.creep and self.phase == 'release':
      return out
    if need >= URGENT_NEED and a_mpc < out:
      self.a = out = a_mpc
    return out
