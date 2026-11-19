# dppb (owner 2026-11-18/19): StopControl - every stop 3.5 m behind the lead (+-0.25 m), no jolt at the stop, no two-stage
import math
from types import SimpleNamespace

import numpy as np
import pytest

from dragonpilot.selfdrive.controls.lib import stop_control as sc

DT = 0.01


def lead(d, v=0.0, status=True):
  return SimpleNamespace(status=status, dRel=d, vLead=v)


def car_state(v, a, gas=False):
  vw = v if v > 0.6 / 3.6 else 0.0          # the wheel speeds read 0 below ~0.6 km/h
  return SimpleNamespace(vEgo=vw, aEgo=a, standstill=vw <= 0.0, gasPressed=gas)


def delay_of(v):
  return float(np.interp(v, sc.ST_DELAY_V, sc.ST_DELAY_T))


def run(v0, gap0, gain=1.0, delay_k=1.0, lead_v=None, t_max=40.0, imu=True, planner=None, after=3.0):
  """closed loop on the measured car: pure delay by speed, real decel = gain x nominal gain x command; a released brake
  creeps. Returns rows (t, v, a, cmd, gap)"""
  c = sc.StopControl(DT)
  v, x, xl, a = v0, 0.0, gap0, 0.0
  buf = [0.0] * 200
  rows, t_stop = [], None
  for k in range(int(t_max / DT)):
    t = k * DT
    vl = lead_v(t) if lead_v else 0.0
    xl += vl * DT
    gap = xl - x
    plan = planner(gap, vl) if planner else 0.0
    cmd = c.update(plan, plan, car_state(v, a), lead(gap, vl), True, a if imu else None)
    buf.append(cmd)
    d = buf[-1 - int(round(delay_k * delay_of(v) / DT))]
    if d < -0.05:
      a = d * gain * float(np.interp(v, sc.ST_GAIN_V, sc.ST_GAIN)) if v > 0.0 else 0.0
    else:
      a = max(d, 0.0) + 0.30 * max(1.0 - v / 1.9, 0.0)
    v = max(v + a * DT, 0.0)
    x += v * DT
    rows.append((t, v, a, cmd, gap))
    if v <= 0.0 and t_stop is None and t > 1.0:
      t_stop = t
    if v > 0.0 and lead_v is not None:
      t_stop = None
    if t_stop is not None and t > t_stop + after:
      break
  return np.array(rows), c


def first_stop(R):
  i = int(np.argmax((R[1:, 1] <= 0.0) & (R[:-1, 1] > 0.0))) + 1
  return i, float(R[i, 4]), float(np.mean(R[max(i - 10, 0):i, 2]))


def test_profile_distance_matches_its_own_decel():
  for v0 in (0.15, 0.5, 1.5, 4.0, 7.0):
    for a_mid in (0.3, 0.8, 2.0):
      v, d = v0, 0.0
      while v > 0.0:
        v -= sc.profile_decel(v, a_mid) * 1e-3
        d += max(v, 0.0) * 1e-3
      assert sc.profile_distance(v0, a_mid) == pytest.approx(d, abs=0.02 + 0.01 * d)


def test_solve_ends_at_the_point():
  for v0, s in ((5.0, 20.0), (3.0, 6.0), (1.0, 1.2), (6.0, 20.0)):
    need, jerk, a_end = sc.solve_decel(s, v0)
    assert sc.ST_A_END <= need <= sc.ST_A_MAX and jerk == sc.ST_JERK_END and a_end == sc.ST_A_END
  # too late for the light end: faster easing and a firmer end, never a lighter one
  need, jerk, a_end = sc.solve_decel(0.6, 2.0)
  assert jerk > sc.ST_JERK_END_MAX and a_end > sc.ST_A_END


def test_no_lead_or_not_active_is_untouched():
  c = sc.StopControl(DT)
  assert c.update(-0.7, -0.7, car_state(5.0, -0.7), None, True, -0.7) == -0.7
  assert c.update(-0.7, -0.7, car_state(5.0, -0.7), lead(20.0), False, -0.7) == -0.7
  assert c.update(0.4, 0.4, car_state(5.0, 0.0, gas=True), lead(8.0), True, 0.0) == 0.4
  assert not c.active and not c.held


@pytest.mark.parametrize("v0_kmh,gap0", [(8, 9.0), (15, 16.0), (24, 30.0), (20, 45.0), (5, 6.5)])
@pytest.mark.parametrize("gain,delay_k", [(1.0, 1.0), (0.85, 1.0), (1.15, 1.0), (1.0, 0.6), (1.0, 1.4)])
def test_stops_at_the_gap_without_a_jolt(v0_kmh, gap0, gain, delay_k):
  R, c = run(v0_kmh / 3.6, gap0, gain=gain, delay_k=delay_k)
  i, gap, dec = first_stop(R)
  assert 3.25 <= gap <= 3.75, gap
  assert dec > -0.36, dec                                   # the felt decel just before the stop
  j = i - 25                                                # what is sent in the last 0.25 s is felt after the stop
  low = R[:j][(R[:j, 1] < 1.5 / 3.6) & (R[:j, 1] > 0.0)]
  assert low[:, 3].min() > low[0, 3] - 0.08                 # the command is not firmed in the last 1.5 km/h
  assert c.held and R[-1, 1] == 0.0 and R[-1, 3] <= sc.ST_HOLD + 1e-6     # held, no roll-on
  assert abs(R[-1, 4] - gap) < 0.02


def test_no_imu_still_stops_at_the_gap():
  R, _ = run(20 / 3.6, 25.0, imu=False)
  _, gap, dec = first_stop(R)
  assert 3.25 <= gap <= 3.75 and dec > -0.36


def test_the_hold_starts_after_the_real_stop_not_at_the_wheel_speed_flag():
  R, _ = run(15 / 3.6, 16.0)
  i, _, _ = first_stop(R)
  flag = int(np.argmax(R[:, 1] < 0.6 / 3.6))               # the wheel speeds read 0 from here
  assert i - flag > 20                                       # the car really stops later
  assert R[flag:i, 3].min() > R[flag, 3] - 0.05             # nothing is firmed in between


def test_past_the_point_is_firmed_at_any_speed():
  c = sc.StopControl(DT)
  out = 0.0
  for _ in range(300):                                       # creeping on at 1.5 km/h, already 0.5 m past the point
    out = c.update(-0.3, -0.3, car_state(1.5 / 3.6, 0.0), lead(3.0), True, 0.0)
  assert c.active and out < -0.6


def test_follows_a_creeping_queue_and_stops_at_the_gap_again():
  def lv(t):                                                 # the lead moves on 3 m at up to 4.3 km/h, twice
    for t0 in (12.0, 28.0):
      if t0 < t <= t0 + 2.0:
        return 0.6 * (t - t0)
      if t0 + 2.0 < t <= t0 + 2.5:
        return 1.2
      if t0 + 2.5 < t <= t0 + 4.5:
        return 1.2 - 0.6 * (t - t0 - 2.5)
    return 0.0

  # the planner's part (the launch) is a light acceleration while the lead moves
  c = sc.StopControl(DT)
  v, x, xl, a = 20 / 3.6, 0.0, 25.0, 0.0
  buf, stops = [0.0] * 200, []
  for k in range(int(45.0 / DT)):
    t = k * DT
    vl = lv(t)
    xl += vl * DT
    gap = xl - x
    plan = 0.5 if (vl > 0.3 and gap > 4.2) else 0.0
    cmd = c.update(plan, plan, car_state(v, a), lead(gap, vl), True, a)
    buf.append(cmd)
    d = buf[-1 - int(round(delay_of(v) / DT))]
    a = (d * float(np.interp(v, sc.ST_GAIN_V, sc.ST_GAIN)) if v > 0.0 else 0.0) if d < -0.05 else max(d, 0.0) + 0.30 * max(1.0 - v / 1.9, 0.0)
    v_new = max(v + a * DT, 0.0)
    if v_new <= 0.0 < v:
      stops.append((t, gap, a))
    v = v_new
    x += v * DT
  assert len(stops) == 3, stops                              # one stop per lead stop: never a two-stage stop
  for _, gap, dec in stops:
    assert 3.25 <= gap <= 3.85 and dec > -0.40, stops


def test_lead_driving_off_hands_back_at_once():
  R, c = run(15 / 3.6, 16.0, lead_v=lambda t: 0.0 if t < 12.0 else min(1.2 * (t - 12.0), 10.0),
             planner=lambda gap, vl: 0.6 if (vl > 0.3 and gap > 4.5) else 0.0, t_max=16.0, after=20.0)
  assert not c.held and not c.active
  assert R[-1, 1] > 1.5                                      # following (the planner's command passes through)
  moving = R[(R[:, 0] > 12.0) & (R[:, 1] > 0.1)]
  assert moving[0, 0] - 12.0 < 1.5


def test_lead_rolling_on_a_little_is_closed_up():
  # the lead creeps 0.6 m at 0.25 m/s (too slow to count as moving off): the car closes up and stops at the gap again
  R, c = run(15 / 3.6, 16.0, lead_v=lambda t: 0.25 if 12.0 < t <= 14.4 else 0.0, t_max=30.0, after=8.0)
  assert c.held and 3.25 <= R[-1, 4] <= 3.85, R[-1, 4]


def test_override_stands_down():
  c = sc.StopControl(DT)
  for _ in range(50):
    c.update(-0.5, -0.5, car_state(2.0, -0.5), lead(6.0), True, -0.5)
  assert c.active
  c.override(-0.8)
  assert not c.active and not c.held and c.pipe[-1] == -0.8 and math.isclose(c.cmd, -0.8)
