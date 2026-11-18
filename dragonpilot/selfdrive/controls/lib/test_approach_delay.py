# dppb (owner 2026-11-18): while closing in on a slower, still far lead the MPC obstacle is moved out by
# APPROACH_DELAY_T * (vEgo - vLead), so a normal stop starts ~0.2-0.3 s later; never for a hard brake, a near lead,
# or steady following
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib import long_mpc as lm


def lead(d, v, a=0.0, status=True):
  return SimpleNamespace(status=status, dRel=d, vLead=v, aLeadK=a, aLeadTau=1.5)


def delay(ld, v):
  return lm.LongitudinalMpc.approach_delay(ld, v)


def test_no_lead():
  assert delay(lead(0.0, 0.0, status=False), 15.0) == 0.0


def test_steady_following_is_untouched():
  assert delay(lead(100.0, 15.0), 15.0) == 0.0
  assert delay(lead(100.0, 20.0), 15.0) == 0.0     # lead pulling away


def test_far_stopped_lead_full_delay():
  # 50 km/h, stopped car 110 m ahead: needs ~0.6 m/s^2 -> full APPROACH_DELAY_T of closing travel
  v = 50 / 3.6
  assert delay(lead(110.0, 0.0), v) == pytest.approx(lm.APPROACH_DELAY_T * v)


def test_faded_out_near_the_lead():
  assert delay(lead(lm.APPROACH_FADE_D[0], 0.0), 30 / 3.6) == 0.0
  assert delay(lead(30.0, 0.0), 30 / 3.6) == 0.0


def test_hard_brake_is_never_delayed():
  v = 60 / 3.6
  assert delay(lead(80.0, 0.0, -4.0), v) == 0.0     # lead braking hard
  assert delay(lead(55.0, 0.0), v) == 0.0           # stopped car 55 m ahead at 60 km/h: needs > 3 m/s^2


def test_partial_between_the_fades():
  v = 50 / 3.6
  d = delay(lead(sum(lm.APPROACH_FADE_D) / 2, 0.0), v)
  assert 0.0 < d < lm.APPROACH_DELAY_T * v
