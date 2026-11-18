# dppb (owner 2026-11-15): the MPC's braking floor is -2.54 (11:37 three-bookmark template); it opens to -3.5 only
# while braking at -2.54 would still hit the lead, and closes again once that clears or we have stopped
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib import long_mpc as lm
from opendbc.car.interfaces import ACCEL_MIN


def mpc(a=0.0):
  m = SimpleNamespace(mid_open=False, x0=np.array([0.0, 0.0, a]), a_solution=np.zeros(33))
  m.mid_floor_gap = lambda ld, v: lm.LongitudinalMpc.mid_floor_gap(m, ld, v)
  return m


def lead(d, v, a=0.0, status=True):
  return SimpleNamespace(status=status, dRel=d, vLead=v, aLeadK=a, aLeadTau=1.5)


def rs(l1, l2=None):
  return SimpleNamespace(leadOne=l1, leadTwo=l2 or lead(0.0, 0.0, status=False))


def floor(m, r, v):
  return lm.LongitudinalMpc.mid_floor(m, r, v)


def test_no_lead_is_the_mid_floor():
  assert floor(mpc(), rs(lead(0.0, 0.0, status=False)), 20.0) == lm.MID_MIN


def test_steady_following_is_the_mid_floor():
  assert floor(mpc(), rs(lead(30.0, 15.0)), 15.0) == lm.MID_MIN


def test_10_32_like_lead_stop_keeps_the_mid_floor():
  # 47 km/h behind a lead at 24 m that starts braking at -2
  assert floor(mpc(), rs(lead(24.0, 13.0, -2.0)), 13.0) == lm.MID_MIN


def test_stopped_car_close_ahead_opens_to_accel_min():
  # 50 km/h, stopped car 30 m ahead: -2.54 needs ~38 m + reaction
  assert floor(mpc(), rs(lead(30.0, 0.0)), 50 / 3.6) == ACCEL_MIN


def test_hard_lead_brake_close_opens():
  assert floor(mpc(), rs(lead(15.0, 15.0, -7.0)), 15.0) == ACCEL_MIN


def test_cut_in_lead_two_counts():
  assert floor(mpc(), rs(lead(60.0, 20.0), lead(8.0, 5.0)), 50 / 3.6) == ACCEL_MIN


def test_open_closes_when_clear_or_stopped():
  m = mpc()
  assert floor(m, rs(lead(30.0, 0.0)), 50 / 3.6) == ACCEL_MIN
  assert floor(m, rs(lead(3.0, 0.0)), 0.0) == lm.MID_MIN          # stopped
  m = mpc()
  assert floor(m, rs(lead(30.0, 0.0)), 50 / 3.6) == ACCEL_MIN
  assert floor(m, rs(lead(30.0, 15.0)), 15.0) == lm.MID_MIN       # lead pulled away: clear


def test_floor_is_never_below_accel_min():
  assert lm.MID_MIN > ACCEL_MIN


def test_opens_when_mid_min_would_stop_within_3_m():
  # 11-18 owner: open once -2.54 would stop within 3 m (was 0.5 m); 40 km/h, not yet braking, stopped car 38 m ahead
  # (-2.54 after the reaction would stop ~1.1 m behind it)
  m = mpc()
  gap = m.mid_floor_gap(lead(38.0, 0.0), 40 / 3.6)
  assert 0.5 < gap < lm.CRASH_GAP
  assert floor(m, rs(lead(38.0, 0.0)), 40 / 3.6) == ACCEL_MIN


def test_does_not_close_while_the_plan_still_brakes_past_mid_min():
  m = mpc()
  assert floor(m, rs(lead(30.0, 0.0)), 50 / 3.6) == ACCEL_MIN
  m.a_solution = np.full(33, -2.9)
  assert floor(m, rs(lead(30.0, 15.0)), 15.0) == ACCEL_MIN        # clear, but the plan is still at -2.9
  m.a_solution = np.full(33, -2.0)
  assert floor(m, rs(lead(30.0, 15.0)), 15.0) == lm.MID_MIN       # plan back above -2.54: close
