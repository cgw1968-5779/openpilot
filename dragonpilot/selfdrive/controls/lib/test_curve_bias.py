from dragonpilot.selfdrive.controls.lib.curve_bias import (CurveTurnIn, curve_demand_weight, CURVE_IN_T, CURVE_OUTWARD_DEADZONE,
                                                          CURVE_ROAD_K, CURVE_TORQUE_AY, CURVE_TORQUE_SHARE)

KPH = 1 / 3.6
DT = 0.01


def _turn_in(v, k, seconds, extra=0.0, lane_k=None, ti=None):
  ti = ti or CurveTurnIn(DT)
  out = []
  for _ in range(int(seconds / DT)):
    out.append(ti.update(k, v, extra, lane_k))
  return out


def test_tncr1_values_are_the_turn_in_lag():
  # owner 2026-11-28: the fraction table (11-09 / 11-25 values) is kept, it now sets the turn-in lag
  ti = CurveTurnIn(DT)
  for kph, frac in ((30, 0.0), (40, 0.065), (50, 0.13), (55, 0.175), (60, 0.22), (65, 0.22), (70, 0.22), (80, 0.22), (90, 0.22), (130, 0.22)):
    for k in (0.001, -0.001):   # 1.3 m/s^2 at 130 km/h: below the torque share
      assert abs(ti.lag(k, kph * KPH) - frac * CURVE_IN_T) < 1e-12


def test_turn_in_is_held_back_then_the_models_curvature_in_full():
  # owner 2026-11-28: "FRAC only as a pure turn-in timing adjustment": a step of the model's curvature is followed
  # through a first-order lag (63 % after the lag time), and the command ends AT the model's curvature - nothing is
  # taken off the steady curve
  v, k = 65 * KPH, 0.003
  tau = 0.22 * CURVE_IN_T
  out = _turn_in(v, k, 4.0)
  assert 0.0 < out[0] < 0.05 * k
  assert abs(out[int(tau / DT) - 1] / k - (1 - 2.718281828 ** -1)) < 0.02
  assert all(b >= a for a, b in zip(out, out[1:])) and out[-1] > 0.999 * k
  assert [abs(x) for x in _turn_in(v, -k, 4.0)] == [abs(x) for x in out]      # both sides alike


def test_easing_the_way_out_and_a_change_of_side_are_not_delayed():
  v, k = 65 * KPH, 0.003
  ti = CurveTurnIn(DT)
  _turn_in(v, k, 4.0, ti=ti)
  assert ti.update(0.8 * k, v) == 0.8 * k           # easing: at once
  assert ti.update(0.0, v) == 0.0                   # the way out: at once
  _turn_in(v, k, 0.2, ti=ti)                        # turning in again, still behind...
  assert 0.0 < ti.k < 0.5 * k
  below = 0.5 * ti.k
  assert ti.update(below, v) == below               # ...the model eases below the held value: follow it down at once
  assert abs(ti.update(-k, v)) < 0.05 * k and ti.k < 0.0    # other side: starts from zero, no carry-over
  assert ti.update(k, v, active=False) == k and ti.k == k   # not steering: the model's value, state kept in step


def test_no_hold_back_when_the_curve_needs_the_steering():
  # owner 2026-11-28 ("the torque is never to be cut; every curve the full torque can take must be taken"): the lag is
  # whole while the curve asks less than 60 % of what the full torque holds at that speed and gone from 90 %
  for kph, cap, frac in ((50, CURVE_TORQUE_AY[1], 0.13), (70, CURVE_TORQUE_AY[2], 0.22), (110, CURVE_TORQUE_AY[2], 0.22)):
    v = kph * KPH
    lo, hi = CURVE_TORQUE_SHARE[0] * cap, CURVE_TORQUE_SHARE[1] * cap
    ti = CurveTurnIn(DT)
    for sign in (1.0, -1.0):
      k_easy, k_mid, k_hard = (sign * a / v ** 2 for a in (lo, (lo + hi) / 2, hi))
      assert abs(ti.lag(k_easy, v) - frac * CURVE_IN_T) < 1e-12
      assert abs(ti.lag(k_mid, v) - frac * CURVE_IN_T / 2) < 1e-9
      assert ti.lag(k_hard, v) == 0.0 and ti.lag(2 * k_hard, v, 0.05) == 0.0
      assert CurveTurnIn(DT).update(k_hard, v) == k_hard                      # the full command in the first cycle
      assert ti.lag(k_easy, v, 0.0, abs(k_hard)) == 0.0                       # the lane ahead already that tight
    assert curve_demand_weight(0.0, None, v) == curve_demand_weight(0.0, 0.0, v) == 1.0
  # a curve that tightens toward the limit: the command catches the model up before the limit, without a step
  v = 70 * KPH
  ti, prev, worst = CurveTurnIn(DT), 0.0, 0.0
  for i in range(300):
    k = (0.5 + 0.01 * i) / v ** 2                   # 0.5 -> 3.5 m/s^2 in 3 s
    out = ti.update(k, v)
    worst, prev = max(worst, (out - prev) * v ** 2), out
    if k * v ** 2 >= CURVE_TORQUE_SHARE[1] * CURVE_TORQUE_AY[2]:
      assert out == k
  assert worst < 0.06                               # m/s^2 per 10 ms cycle at the fastest


def test_a_straight_road_is_never_delayed_whatever_the_correction():
  # owner 2026-11-28: "on a straight FRAC must not touch the stability". With the lines seen the road decides.
  for kph in (60, 90, 110):
    v = kph * KPH
    for k in (0.0006, -0.0012, 0.002):
      assert CurveTurnIn(DT).update(k, v, 0.03, 0.0) == k
      assert CurveTurnIn(DT).update(k, v, 0.03, CURVE_ROAD_K[0]) == k
      full = CurveTurnIn(DT).lag(k, v, 0.03)
      assert full > 0.0 and CurveTurnIn(DT).lag(k, v, 0.03, CURVE_ROAD_K[1]) == full     # a real curve: the whole lag
      assert abs(CurveTurnIn(DT).lag(k, v, 0.03, sum(CURVE_ROAD_K) / 2) - full / 2) < 1e-12
    for k in (0.0, CURVE_OUTWARD_DEADZONE, -CURVE_OUTWARD_DEADZONE / 2):     # lines not seen: the command's deadzone
      assert CurveTurnIn(DT).update(k, v) == k
