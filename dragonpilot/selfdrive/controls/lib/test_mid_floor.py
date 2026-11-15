# dppb (owner 2026-11-15): the MPC's braking floor is -2.54 (11:37 three-bookmark template); it opens to -3.5 only
# while braking at -2.54 would still hit the lead, and closes again once that clears or we have stopped
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib import long_mpc as lm
from opendbc.car.interfaces import ACCEL_MIN


def mpc(a=0.0):
  m = SimpleNamespace(mid_open=False, x0=np.array([0.0, 0.0, a]))
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
