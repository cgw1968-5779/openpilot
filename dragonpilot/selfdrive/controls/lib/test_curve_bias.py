import numpy as np
from dragonpilot.selfdrive.controls.lib.curve_bias import apply_curve_outward_bias, CURVE_OUTWARD_DEADZONE

KPH = 1 / 3.6


def test_tncr1_values():
  # owner 2026-11-09: 60 / 70 km/h 0.22 / 0.17 (11-07: 0.16 / 0.13; 50 km/h 0.13 unchanged)
  # owner 2026-11-25: 0.22 held from 60 km/h up (70 / 90-120 km/h were 0.17 / 0.155)
  for kph, frac in ((30, 0.0), (40, 0.065), (50, 0.13), (55, 0.175), (60, 0.22), (65, 0.22), (70, 0.22), (80, 0.22), (90, 0.22), (130, 0.22)):
    for k in (0.001, -0.001):   # 1.3 m/s^2 at 130 km/h: below the demand fade (11-28)
      assert abs(apply_curve_outward_bias(k, kph * KPH) - k * (1 - frac)) < 1e-12


def test_straights_untouched():
  for k in (0.0, CURVE_OUTWARD_DEADZONE, -CURVE_OUTWARD_DEADZONE / 2):
    assert apply_curve_outward_bias(k, 80 * KPH) == k


def test_outward_fraction_never_takes_steering_the_curve_needs():
  # owner 2026-11-28 ("the torque is never to be cut; every curve the full torque can take must be taken"): the
  # fraction is whole while the curve asks no more than the full torque holds at that speed, gone from 120 % of it,
  # continuous in between
  from dragonpilot.selfdrive.controls.lib.curve_bias import curve_demand_weight, CURVE_TORQUE_AY, CURVE_TORQUE_SHARE, CURVE_TORQUE_V_BP
  for kph, frac in ((50, 0.13), (70, 0.22), (110, 0.22)):
    v = kph * KPH
    cap = float(np.interp(v, CURVE_TORQUE_V_BP, CURVE_TORQUE_AY))
    lo, hi = CURVE_TORQUE_SHARE[0] * cap, CURVE_TORQUE_SHARE[1] * cap
    for sign in (1.0, -1.0):
      # the demand is the command with the fraction in full (what the road asks), not the model's request
      k_easy, k_mid, k_hard = (sign * a / (1 - frac) / v ** 2 for a in (lo, (lo + hi) / 2, hi))
      assert abs(apply_curve_outward_bias(k_easy, v) - k_easy * (1 - frac)) < 1e-12
      assert abs(apply_curve_outward_bias(k_mid, v) - k_mid * (1 - frac / 2)) < 1e-9
      assert abs(apply_curve_outward_bias(k_hard, v) - k_hard) < 1e-12
      assert apply_curve_outward_bias(2 * k_hard, v, 0.05) == 2 * k_hard           # the learned part goes with it
      # the lane centre ahead already that tight: the fraction goes before the model's action asks for it
      assert abs(apply_curve_outward_bias(k_easy, v, 0.0, abs(k_hard)) - k_easy) < 1e-12
      assert apply_curve_outward_bias(k_easy, v, 0.0, abs(k_easy) * (1 - frac)) == apply_curve_outward_bias(k_easy, v)
    prev = None
    for a in [x * 0.01 for x in range(50, 501)]:      # no step anywhere: 0.01 m/s^2 moves the command by < 0.03 m/s^2
      out = apply_curve_outward_bias(a / v ** 2, v) * v ** 2
      assert prev is None or 0.0 < out - prev < 0.03
      prev = out
    assert curve_demand_weight(0.0, None, v) == curve_demand_weight(0.0, 0.0, v) == 1.0


def test_a_straight_road_is_never_scaled_whatever_the_correction():
  # owner 2026-11-28: "on a straight FRAC must not touch the stability". With the lines seen the road decides:
  # lane centre ahead straighter than R 10000 m -> the command passes unchanged, even a large correction
  from dragonpilot.selfdrive.controls.lib.curve_bias import CURVE_ROAD_K
  for kph in (60, 90, 110):
    v = kph * KPH
    for k in (0.0006, -0.0012, 0.002):
      assert apply_curve_outward_bias(k, v, 0.03, 0.0) == k
      assert apply_curve_outward_bias(k, v, 0.03, CURVE_ROAD_K[0]) == k
      full = apply_curve_outward_bias(k, v, 0.03)                                   # lines not seen: as before
      assert apply_curve_outward_bias(k, v, 0.03, CURVE_ROAD_K[1]) == full != k     # a real curve: as before
      half = apply_curve_outward_bias(k, v, 0.03, sum(CURVE_ROAD_K) / 2)
      assert abs(half - (k + full) / 2) < 1e-12
