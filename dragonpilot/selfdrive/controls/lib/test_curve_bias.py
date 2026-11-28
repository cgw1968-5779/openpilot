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


def test_outward_fraction_fades_out_as_the_curve_takes_all_the_steering():
  # owner 2026-11-28 (route 00000015 16:33:36): a curve asking 2.0 m/s^2 or less is as before, 3.0 or more gets the
  # model's full curvature, and the change in between is continuous
  from dragonpilot.selfdrive.controls.lib.curve_bias import curve_demand_weight, CURVE_DEMAND_AY
  v = 70 * KPH
  for sign in (1.0, -1.0):
    k_easy, k_mid, k_hard = (sign * a / v ** 2 for a in (CURVE_DEMAND_AY[0], 2.5, CURVE_DEMAND_AY[1]))
    assert abs(apply_curve_outward_bias(k_easy, v) - k_easy * (1 - 0.22)) < 1e-12
    assert abs(apply_curve_outward_bias(k_mid, v) - k_mid * (1 - 0.11)) < 1e-9
    assert apply_curve_outward_bias(k_hard, v) == k_hard
    assert apply_curve_outward_bias(2 * k_hard, v, 0.05) == 2 * k_hard           # the learned part goes with it
    # the lane centre ahead already that tight: the fraction goes before the model's action asks for it
    assert apply_curve_outward_bias(k_easy, v, 0.0, abs(k_hard)) == k_easy
    assert apply_curve_outward_bias(k_easy, v, 0.0, abs(k_easy) / 2) == apply_curve_outward_bias(k_easy, v)
  prev = None
  for a in [x * 0.01 for x in range(150, 351)]:      # no step anywhere: 0.01 m/s^2 moves the command by < 0.01 m/s^2
    out = apply_curve_outward_bias(a / v ** 2, v) * v ** 2
    assert prev is None or 0.0 < out - prev < 0.02
    prev = out
  assert curve_demand_weight(0.0, 0.0, v) == 1.0
