"""
dragonpilot (dpcr, Corolla Cross Hybrid): curve outward bias (FRAC), ported unchanged from the owner's sunnypilot branch
tncr1 (sunnypilot/selfdrive/controls/lib/lateral_path/curve_bias.py, the values tncr1 runs today, 636f66c3df).
Owner 2026-11-03 (dpcr route 000000d2, a gentle left curve at 65 km/h): "corners clearly sit on the inside; port tncr1's FRAC".
Applied to the model's desired curvature in controlsd (one hook line), before the stock curvature clip.
"""
import numpy as np

from openpilot.common.constants import CV

# Corner-cutting fix (Altis rlog 2026-09-20: the e2e path sits ~0.6 m inside on curves, both directions). Relax the
# commanded curvature by a fraction so the car runs a little wider (outward) in proportion to how tight the curve is.
# Straights and small lane corrections below the deadzone are untouched. Schedule ported from tncr18 (Corolla Cross
# road tests 2026-09-23..25): the outward bias only helps at higher speed, where the path cuts the inside. On tight
# LOW-speed curves the car tends to run WIDE instead, so relaxing the curvature there makes it worse.
CURVE_OUTWARD_DEADZONE = 0.0005  # 1/m (~radius 2000 m): below this is a straight / small correction, untouched.
# Speed schedule (m/s -> fraction): 0 up to 40 km/h, 0.09 at 70, 0.155 at 90 km/h, 0.19 at 120 km/h.
# tncr, owner 2026-10-15 night: "port tncr19's FRAC values and the other lateral corrections" - tncr was cut with the
# Altis (tnpb3) lateral values; the Corolla Cross keeps its own road-tested tune (tncr18 -> tncr19 693947f88b):
# -0.05 at 30 km/h (inward), 0.02 at 40, 0.06 at 50, 0.085 at 60, 0.09 at 70, flat 0.155 from 90 km/h.
# (Altis/tnpb3 values were -0.01 @30, 0 @35, 0.03 @40, 0.063 @50, 0.087 @60.)
# dpcr, owner 2026-11-05 night: the 40 km/h point removed, 30 -> 50 km/h interpolated (0.04 at 40 km/h, was 0.05).
CURVE_OUTWARD_V_BP = [v * CV.KPH_TO_MS for v in (30., 50., 60., 70., 90., 100., 120.)]   # m/s
# tncr, owner 2026-10-27 (route b8 mountain ALKA, signs re-checked): in 30-60 km/h curves the car sits 0.25-0.4 m toward
# the INSIDE (straights centred) and the owner's own drive steered outward in 27 of 32 curves to keep the centre, so
# the low-speed inward bias goes: 30 km/h -0.05 -> 0, 40 km/h 0.02 -> 0.05 (50 km/h and up unchanged).
# dpcr, owner 2026-11-03 night (after the first road test of these values: "FRAC clearly improved curve centring"):
# 60 and 70 km/h +0.02 (0.085 -> 0.105, 0.09 -> 0.11); then 60 km/h +0.005 more (0.105 -> 0.11).
# dpcr, owner 2026-11-05 night: 50 km/h +0.02 (0.06 -> 0.08).
# dpcr, owner 2026-11-07 (route 000000f2 23:41-23:42, ALKA: right curve R 120-190 m at 50-68 km/h, left curve R 70-80 m
# at 46 km/h, "close to / riding the inside line - centre it"): f2 segs 7-9 sat 0.4-0.7 m inside, the 11-03..11-06 routes
# 0.1-0.6 m inside in 40-70 km/h curves. 50 / 60 / 70 km/h 0.08 / 0.11 / 0.11 -> 0.13 / 0.16 / 0.13. Closed-loop
# perturbation sim (routes d3 / e3 / e4 rlog, model pull-back 1.5-2.5 s): 55-70 km/h curves 6-30 cm further out, 40-55
# km/h 3-11 cm, straights and command jerk unchanged (p95 3.6 -> 3.5). Lane-line feedback (the PD centering or line mode)
# below 70 km/h was tried first: in curves the model's near and far lane estimates disagree and both pushed INWARD.
# dpcr, owner 2026-11-09 (route 00000000--41925a5692 13:14-13:16, 50-76 km/h S-bends, "the 55 km/h left curve still
# drifts inside; mid-speed centring wanders"): curves still 0.3-0.8 m inside (inner-line gap down to 0.17 m), each bend
# cut on its own side = the felt side-to-side wander; straights 50-70 km/h std 8 cm = same as 70-95. 60 / 70 km/h
# 0.16 / 0.13 -> 0.22 / 0.17 (50 unchanged: raising it pushed 40-55 km/h curves 10-44 cm outward). Closed-loop sim
# (whole route, model pull-back 1.5 / 2.0 / 2.5 s): 55-70 km/h curves 11-25 cm further out (were 37-43 cm inside),
# 40-55 km/h 1-5 cm, never past the centre; straights unchanged, command jerk p95 2.69 -> 2.60.
# dpcr, owner 2026-11-25 (after the 11-24 drive: "80 km/h curves sometimes sit on the inside"; shown the table, he chose
# "for now hold 0.22 from 60 km/h up", until the learning logic he asked for at the same time exists): 70 / 90 / 100 /
# 120 km/h 0.17 / 0.155 / 0.155 / 0.155 -> 0.22. Told to the owner: above 70 km/h the line pursuit carries 0.85 of the
# command (lane_centering.py LINE_W_MAX), so this fraction acts on the remaining 15 % only - at 80 km/h the command
# changes by about 0.9 % of the curvature (0.163 -> 0.22 on 15 %); the pursuit's own outward fraction (LINE_OUT_FRAC_V,
# 0 below 85 km/h) is not changed here.
CURVE_OUTWARD_FRAC_V = [0.0, 0.13, 0.22, 0.22, 0.22, 0.22, 0.22]


CURVE_OUTWARD_FRAC_RANGE = [0.0, 0.35]   # the table plus the learned part (curve_bias_learn.py) stays inside this


# dpcr, owner 2026-11-28 (route 00000015 16:33:35-40, bookmark: "the 70 km/h right curve was very bad, the car almost ran
# out of the lane; the first dragonpilot cornered well, it only did not centre"): an S-bend, the right curve tightening
# to R 90 m at 71 km/h (the model asked 3.5 -> 11.3 1/km in 1.7 s, 4.3 m/s^2). The command was the model x 0.73 (table
# 0.22 + learned 0.047) minus the centering's push away from the inside line (-1.3 1/km): 1.8 where the model asked
# 4.3, the wheel unwound from 10 to 3 deg, and one second later the torque sat at 100 % for 2 s while the car swept
# from 87 cm inside to the outer line. Same shape on route 00000011 14:38:24 / 14:38:31 (left curve, 61-66 km/h, model
# 8.7 -> command 6.8 1/km, the car slid 50-100 cm outward in 2 s). The outward fraction is for curves the car has
# margin in (there the model cuts the inside); a curve that takes about all the steering there is (the command is
# capped at 3.0 m/s^2, drive_helpers MAX_LATERAL_ACCEL_NO_ROLL, and the torque gives 2.9-3.1 at 70 km/h) has no margin
# to run wide and come back. So the fraction fades out with the lateral acceleration the curve asks for, from the
# model's curvature or the lane centre's curvature ahead (lane_centering's fit, which saw this curve tighten 0.5 s
# before the model's action), whichever is larger. Dynamic and continuous (no step). First version (8127ecd04): whole up to 2.0 m/s^2, gone from
# 3.0 m/s^2 at every speed; superseded the same night by the torque share below.
# dpcr, owner 2026-11-28 night, after reading the above: "what I want is the car's full torque kept for the curve, the
# torque is never to be cut; every curve the full torque can take must be taken (I hold the speed myself when needed).
# FRAC is there to place the car as centred as possible; on a straight it must not touch the stability." So:
# 1. The outward fraction may only use steering the curve does not need. What the full torque holds, measured with the
#    torque command at 100 % for >= 0.5 s, hands off (routes 0d-15, 26 s): 20-50 km/h 1.9-2.2 m/s^2 (p10 1.6-2.1, the
#    wheel at 40-100 deg), 70-80 km/h 3.0 (p10 2.86) -> CURVE_TORQUE_AY, taken a little under the measured values. The
#    fraction is whole while the curve asks less than CURVE_TORQUE_SHARE[0] of that and gone from CURVE_TORQUE_SHARE[1]
#    on, so the last 10 % and the torque limit itself are never reached with a cut command.
# 2. A straight is a straight by the ROAD, not by the command: while the lane lines are seen the fraction fades in with
#    the lane centre's own curvature ahead (CURVE_ROAD_K, the same band as the line pursuit's LINE_OUT_K_BP), so a
#    correction on a straight road - however large - is never scaled. Without lines: the command's deadzone as before.
# dpcr, owner 2026-11-28 late (route 00000017 22:00-22:20 on 3d6311b9f, "it crosses the inside line completely"; then:
# "at every speed, in a tight curve, the full torque must be usable and the car must not sit on the inside"). The table
# above (1.8 / 2.1 / 2.9 at 30 / 50 / 70 km/h, fraction gone from 90 % of it) was far too low at 40-60 km/h: with that
# night's route added, the torque command at 100 % for >= 0.5 s, hands off (45 s in all), holds 2.1 m/s^2 at 30-40
# km/h, 2.7 at 40-50, 2.9-3.0 from 50 km/h up (50-60 km/h: median 2.94, p10 2.77). So the fade took the outward
# fraction and the centering's push away from the inside line out of ordinary mountain curves (R 77-150 m at 45-60
# km/h, 1.8-2.6 m/s^2), which the car takes with torque to spare - without them it ran 60-100 cm inside, up to 1.8 m.
# The outward fraction does not cost steady-state torque (the curve's curvature is the road's either way, the fraction
# only moves where the car settles); it costs when the curve asks MORE than the torque holds. So: whole up to the
# measured limit, gone from 120 % of it. Open-loop replay of the logged model frames through the real classes against
# the daytime state (2bb3276a3): 11-28 day routes 11 + 15 (80 min) 0.67 % of the frames change by > 0.02 m/s^2, model
# demand below 2.0 m/s^2 unchanged (4767 s, mean 0.000); night route 17 1.7 %, demand 2.0-2.5 m/s^2 mean 0.013.
# 16:33:36-37: the command reaches the model's 9.9 1/km at 16:33:37.0 (was 6.7), estimated farthest point toward the
# outer line 90 -> ~40 cm. With [0.9, 1.1] the estimate was ~20 cm, but twice as many of route 17's frames changed.
CURVE_TORQUE_V_BP = [35.0 * CV.KPH_TO_MS, 45.0 * CV.KPH_TO_MS, 55.0 * CV.KPH_TO_MS]   # m/s
CURVE_TORQUE_AY = [2.1, 2.7, 2.9]      # m/s^2 the full steering torque holds (measured, see above)
CURVE_TORQUE_SHARE = [1.0, 1.2]        # of it: outward fraction whole .. gone
CURVE_ROAD_K = [1.0e-4, 3.0e-4]        # 1/m: lane-centre curvature ahead, straight road .. curve


def curve_demand_weight(curvature: float, lane_k: float | None, v_ego: float) -> float:
  """1 while the curve leaves steering torque to spare (outward bias in full), 0 when it needs about all of it.
  lane_k: |curvature| of the lane centre ahead, None = lines not seen"""
  cap = float(np.interp(v_ego, CURVE_TORQUE_V_BP, CURVE_TORQUE_AY))
  demand = max(abs(curvature), lane_k or 0.0) * v_ego ** 2
  return float(np.interp(demand, [CURVE_TORQUE_SHARE[0] * cap, CURVE_TORQUE_SHARE[1] * cap], [1.0, 0.0]))


def apply_curve_outward_bias(desired_curvature: float, v_ego: float, extra: float = 0.0, lane_k: float | None = None) -> float:
  # In a fast curve, command a slightly smaller curvature so the car runs wider and stops cutting the inside. Faded
  # out at low speed (tight curves there need the full turn-in or the car runs wide). Straights and small lane
  # corrections below the deadzone are untouched. lane_k: |curvature| of the lane centre ahead (None = not seen).
  frac = float(np.interp(v_ego, CURVE_OUTWARD_V_BP, CURVE_OUTWARD_FRAC_V))
  if extra != 0.0:   # the learned part for this speed and side
    frac = float(np.clip(frac + extra, CURVE_OUTWARD_FRAC_RANGE[0], CURVE_OUTWARD_FRAC_RANGE[1]))
  if frac == 0.0 or abs(desired_curvature) <= CURVE_OUTWARD_DEADZONE:
    return desired_curvature
  if frac > 0.0:
    # the demand is what the ROAD asks: the command with the fraction in full, not the model's request. In a steady
    # curve the model asks road / (1 - frac) to hold its line (R 96 m at 57 km/h = 2.6 m/s^2 -> it asks 3.2), so
    # weighing the request itself took a quarter of the fraction out of curves well inside the torque limit (route
    # 00000017 22:15:18: 34 % of the curve's time, 22:17:15: 39 %).
    frac *= curve_demand_weight(desired_curvature * (1.0 - frac), lane_k, v_ego)
    if lane_k is not None:
      frac *= float(np.interp(lane_k, CURVE_ROAD_K, [0.0, 1.0]))
  return desired_curvature * (1.0 - frac)   # frac < 0 scales the curvature UP (inward)
