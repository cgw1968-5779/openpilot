import pytest
from types import SimpleNamespace as N

import numpy as np

import dragonpilot.selfdrive.controls.lib.stop_approach as SA
from dragonpilot.selfdrive.controls.lib.stop_approach import StopApproach

DT = 0.05


import pytest
import dragonpilot.selfdrive.controls.lib.stop_approach as _sa


@pytest.fixture(autouse=True)
def _dpcr_end(monkeypatch, request):
  # dpcr's tests, written around its end line (from 5 km/h, the plan not leaving room for it); dppb's own are *_altis*
  if 'altis' not in request.node.name:
    monkeypatch.setattr(_sa, 'END_V', 5.0 / 3.6)
    monkeypatch.setattr(_sa, 'END_ROOM', 0.0)
    monkeypatch.setattr(_sa, 'SHORT_MAX', 0.0)   # dpcr: the end line is never firmed
    monkeypatch.setattr(_sa, 'SLOPE_ENABLED', False)   # dpcr: brake phase + release (dppb: one slope)
    monkeypatch.setattr(_sa, 'STOP_GAP', 3.5)
    monkeypatch.setattr(_sa, 'END_CURVE_V', [0.0, 1.0 / 3.6, 2.0 / 3.6, 3.0 / 3.6, 4.0 / 3.6, 5.0 / 3.6, 6.0 / 3.6, 7.0 / 3.6, 8.0 / 3.6])
    monkeypatch.setattr(_sa, 'END_CURVE_A', [-0.34, -0.36, -0.37, -0.37, -0.40, -0.45, -0.50, -0.57, -0.65])


def lead(d, v=0.0, status=True):
  return N(status=status, dRel=d, vLead=v)


def drive(v0, d0, a_mpc=-1.0, t_max=40.0):
  """point-mass car that follows the target exactly, approaching a stopped lead"""
  sa = StopApproach(DT)
  v, x, out = v0, 0.0, []
  for _ in range(int(t_max / DT)):
    a = sa.update(v, lead(d0 - x), a_mpc, False)
    out.append((v, d0 - x, a, sa.phase))
    v = max(0.0, v + a * DT)
    x += v * DT
    if v <= 0.0:
      break
  return out


def test_no_lead_and_moving_lead_pass_the_plan_through():
  sa = StopApproach(DT)
  for a in (-1.3, 0.2, -0.4):
    assert sa.update(15.0, lead(40.0, status=False), a, False) == a
    assert sa.update(15.0, lead(40.0, v=8.0), a, False) == a
    assert sa.phase is None


def test_far_stopped_lead_brakes_gently_at_once():
  # owner 2026-11-08: slow down as soon as the stopped car is seen. 50 km/h, 100 m: need ~0.99 -> K_EARLY x need,
  # built at J_EARLY (soft first touch), not the MPC's -1.0
  sa = StopApproach(DT)
  outs = [sa.update(50 / 3.6, lead(100.0), 0.2, False) for _ in range(40)]
  assert sa.phase == 'wait'
  need = (50 / 3.6) ** 2 / (2 * (100.0 - SA.STOP_GAP))
  assert abs(outs[-1] + SA.K_EARLY * need) < 1e-6
  assert outs[0] >= -SA.J_EARLY * DT - 1e-9 and np.min(np.diff(outs)) >= -SA.J_EARLY * DT - 1e-9


def test_build_is_linear_and_limited():
  out = drive(50 / 3.6, 90.0)
  a = np.array([o[2] for o in out if o[0] > 0.3])  # below 0.3 m/s the stopping controller owns the request
  assert np.min(np.diff(a)) >= -SA.J_BUILD * DT - 1e-9  # never firmer faster than J_BUILD
  assert any(o[3] == 'brake' for o in out)


def test_stops_near_the_gap_and_eases_at_the_end():
  for v0, d0 in ((50 / 3.6, 90.0), (40 / 3.6, 60.0), (30 / 3.6, 40.0)):
    out = drive(v0, d0)
    # an ideal lag-free car stops a little short of the aim (the end release); on the Cross model with the stopping
    # controller's end line the closed-loop sim ends 3.4-4.1 m (tncr1_curves/sim/sim_lead.py)
    d_end = out[-1][1]
    assert SA.STOP_GAP - 1.0 < d_end < SA.STOP_GAP + 1.0, (v0, d0, d_end)
    rel = [o for o in out if o[3] == 'release' and o[0] > 0.3]
    assert rel, 'no end release'
    a = np.array([o[2] for o in rel])
    assert np.all(np.diff(a) >= -1e-9)  # the release never firms up again (no second bite)
    assert np.max(np.diff(a)) <= SA.J_RELEASE * DT + 1e-9
    assert rel[-1][2] >= -0.7  # eased onto the end line before the stopping controller takes over


def test_urgent_follows_a_firmer_plan():
  sa = StopApproach(DT)
  a = sa.update(50 / 3.6, lead(12.0), -3.0, False)
  assert a <= -3.0


def test_handback_is_rate_limited():
  sa = StopApproach(DT)
  for _ in range(60):
    sa.update(40 / 3.6, lead(35.0), -1.0, False)
  a0 = sa.a
  a1 = sa.update(40 / 3.6, lead(35.0, v=5.0), 0.5, False)  # the lead pulls away
  assert sa.phase == 'handback'
  assert abs(a1 - a0) <= SA.J_HANDBACK * DT + 1e-9


def test_disengaged_is_untouched():
  sa = StopApproach(DT)
  assert sa.update(15.0, lead(30.0), 0.3, True) == 0.3


# the committed stop target before the experimental-only changes (11-07), kept here so experimental-off stays identical
class _Before11_07(StopApproach):
  def update(self, v_ego, lead, a_mpc, off, exp=False, a_ego=None):
    stopped_lead = (not off and lead is not None and bool(lead.status) and lead.vLead < SA.LEAD_STOPPED_V
                    and 0.0 < lead.dRel < SA.MAX_DIST and v_ego > 0.3)
    if not stopped_lead:
      if self.phase in (None, 'handback') and (self.phase is None or abs(self.a - a_mpc) < 0.05):
        self.phase = None
        self.a = a_mpc
        return a_mpc
      self.phase = 'handback'
      return self._step(a_mpc, SA.J_HANDBACK, SA.J_HANDBACK)
    need = SA._need(v_ego, lead.dRel)
    if self.phase in (None, 'handback'):
      self.a = a_mpc if self.phase is None else self.a
      self.phase = 'wait'
    if self.phase == 'wait' and SA._need(v_ego, lead.dRel - v_ego * 0.5) >= SA.START_NEED:
      self.phase = 'brake'
    if self.phase == 'wait':
      out = self._step(max(a_mpc, SA.WAIT_ACCEL), SA.J_EASE, SA.J_BUILD)
    else:
      goal = float(np.interp(v_ego, SA.END_CURVE_V, SA.END_CURVE_A))
      if self.phase == 'brake':
        a_end = float(np.interp(SA.END_V, SA.END_CURVE_V, SA.END_CURVE_A))
        t_rel = max(a_end - self.a, 0.0) / SA.J_RELEASE
        dv = -(self.a + a_end) / 2.0 * t_rel
        if v_ego - SA.END_V <= dv or (v_ego < SA.END_CURVE_V[-1] and self.a >= goal):
          self.phase = 'release'
      if self.phase == 'brake':
        out = self._step(-SA.K_NEED * need, SA.J_EASE, SA.J_BUILD)
      else:
        target = self.a if need > -self.a + 0.3 else min(goal, self.a + SA.J_RELEASE * self.dt)
        self.a = out = max(target, self.a)
    if need >= SA.URGENT_NEED and a_mpc < out:
      self.a = out = a_mpc
    return out


def test_exp_differs_only_in_the_wait_feedback():
  # owner 2026-11-07: experimental and normal share the stop target; only the WAIT feedback (needs a_ego) is experimental
  rng = np.random.default_rng(7)
  for _ in range(200):
    on, offm = StopApproach(DT), StopApproach(DT)
    v, d, vl = rng.uniform(1, 20), rng.uniform(5, 120), rng.choice([0.0, 0.0, 3.0])
    for _ in range(300):
      a_mpc = rng.uniform(-3.5, 0.5)
      ln = lead(d, vl, status=rng.random() > 0.05)
      assert on.update(v, ln, a_mpc, False, True, None) == offm.update(v, ln, a_mpc, False, False, None)
      v = max(0.0, v + rng.uniform(-0.15, 0.05)); d = max(0.5, d - v * DT + vl * DT)


def test_stopped_lead_starts_earlier_than_before_11_07():
  # 23:09:52: the brake started at 38.6 m from 43 km/h; the start now looks 1.0 s ahead (was 0.5 s)
  def start_d(cls):
    sa, v, x = cls(DT), 43 / 3.6, 0.0
    while True:
      sa.update(v, lead(110.0 - x), -0.3, False)
      if sa.phase == 'brake':
        return 110.0 - x
      x += v * DT
  assert start_d(StopApproach) > start_d(_Before11_07) + 4.0


def test_exp_wait_follows_the_real_decel():
  # 11-06 21:51: 8 km/h, stopped lead 12.7 m ahead, the car slows -0.8 on the -0.30 lift (uphill) -> the lift eases
  sa = StopApproach(DT)
  outs = [sa.update(8.3 / 3.6, lead(12.7), -0.3, False, True, -0.8) for _ in range(20)]
  assert sa.phase == 'wait'
  assert outs[-1] > SA.WAIT_ACCEL + 0.2 and max(outs) <= 0.0  # eased, never gas
  assert np.max(np.diff(outs)) <= SA.WAIT_FB_RATE * DT + 1e-9
  # experimental off: unchanged light lift
  sa = StopApproach(DT)
  assert sa.update(8.3 / 3.6, lead(12.7), -0.3, False, False, -0.8) == SA.WAIT_ACCEL
  # the car no longer slows harder than needed -> back to the light lift
  sa = StopApproach(DT)
  for _ in range(20):
    sa.update(8.3 / 3.6, lead(12.7), -0.3, False, True, -0.8)
  for _ in range(20):
    a = sa.update(8.3 / 3.6, lead(12.7), -0.3, False, True, -0.1)
  assert abs(a - SA.WAIT_ACCEL) < 1e-9


def test_lead_that_stops_while_braking_keeps_the_brake():
  # 11-07 000000f0 23:13:04: 15 km/h, the lead (moving) drops under LEAD_STOPPED_V 14.2 m ahead, plan -1.07:
  # no lift to WAIT_ACCEL, BRAKE on the need straight away, smooth ease to the end
  sa = StopApproach(DT)
  sa.update(15 / 3.6, lead(14.2, v=3.0), -1.07, False)
  a = sa.update(15 / 3.6, lead(14.2), -1.07, False)
  assert sa.phase == 'brake' and a < -0.9
  v, x, out = 15 / 3.6, 0.0, []
  for _ in range(200):
    a = sa.update(v, lead(14.2 - x), -0.8, False)
    out.append(a); v = max(0.0, v + a * DT); x += v * DT
    if v <= 0.3:
      break
  assert max(out[:20]) < -0.8                                      # held near the need at 15 -> 11 km/h (was -0.30)
  assert np.max(np.diff(out)) <= SA.J_RELEASE * DT + 1e-9          # eases, no jump
  # a lead first seen already stopped is unchanged: WAIT
  sa = StopApproach(DT)
  sa.update(15 / 3.6, lead(14.2), -1.07, False)
  assert sa.phase == 'wait'



def test_slow_lead_braking_to_a_stop_aims_at_its_stop_point():
  # 13:05 route 000000fa: 9.5 km/h, lead 5.0 m ahead at 6.8 km/h braking -1.76 -> it stands ~1.0 m further
  from types import SimpleNamespace as N2
  from dragonpilot.selfdrive.controls.lib import stop_approach as SA2
  sa = SA2.StopApproach(0.05)
  ld = N2(status=True, dRel=5.0, vLead=6.8 / 3.6, aLeadK=-1.76)
  sa.update(9.5 / 3.6, N2(status=True, dRel=5.3, vLead=7.5 / 3.6, aLeadK=-0.1), -1.5, False)   # tracked moving before
  out = [sa.update(9.5 / 3.6, ld, -2.06, False) for _ in range(10)][-1]
  need = (9.5 / 3.6) ** 2 / (2 * (5.0 + (6.8 / 3.6) ** 2 / (2 * 1.76) - SA2.STOP_GAP))
  assert sa.stopping and out > -2.06                       # lighter than the MPC's 6 m aim
  assert out == pytest.approx(-SA2.K_NEED * need, abs=0.05)


def test_slow_lead_not_braking_stays_with_mpc():
  from types import SimpleNamespace as N2
  from dragonpilot.selfdrive.controls.lib import stop_approach as SA2
  sa = SA2.StopApproach(0.05)
  ld = N2(status=True, dRel=8.0, vLead=10 / 3.6, aLeadK=-0.1)
  assert sa.update(12 / 3.6, ld, -0.4, False) == -0.4
  assert not sa.stopping


def test_latched_slow_lead_that_stops_braking_hands_back():
  # 00000103 16:13:24.2: 5.6 km/h, lead 7.5 m at 9.7 km/h braking -0.69 -> latched; by 16:13:25.5 it brakes only
  # -0.22 and keeps rolling at 8-9 km/h -> no longer a stop point, back to the MPC (smoothly)
  from types import SimpleNamespace as N2
  from dragonpilot.selfdrive.controls.lib import stop_approach as SA2
  sa = SA2.StopApproach(0.05)
  sa.update(6.2 / 3.6, N2(status=True, dRel=7.3, vLead=10.1 / 3.6, aLeadK=-0.58), -0.55, False)
  sa.update(5.6 / 3.6, N2(status=True, dRel=7.5, vLead=9.7 / 3.6, aLeadK=-0.69), -0.50, False)
  assert sa.stopping
  sa.update(4.6 / 3.6, N2(status=True, dRel=8.4, vLead=9.3 / 3.6, aLeadK=-0.35), -0.10, False)
  assert sa.stopping                                       # still braking >= A_KEEP: kept
  outs = [sa.update(4.1 / 3.6, N2(status=True, dRel=9.1, vLead=8.9 / 3.6, aLeadK=-0.22), 0.2, False) for _ in range(20)]
  assert not sa.stopping
  assert outs[-1] == 0.2                                   # back to the MPC
  assert np.max(np.diff(outs)) <= SA2.J_HANDBACK * 0.05 + 1e-9


def test_virtual_lead_passed_stops_on_the_end_line():
  # owner 2026-11-09 12:57: the experimental stop point was already behind the car after the driver's gas; the MPC
  # never sees the virtual lead, so it must not be handed back (it sent -0.05 and the car crept over the line)
  from types import SimpleNamespace as N
  import numpy as np
  from dragonpilot.selfdrive.controls.lib import stop_approach as SA
  sa = SA.StopApproach(0.05)
  ld = N(status=True, dRel=0.1, vLead=0.0, virtual=True)
  v, outs = 4.5 / 3.6, []
  for _ in range(20):
    outs.append(sa.update(v, ld, 0.0, False, True, 0.0))
  goal = float(np.interp(v, SA.END_CURVE_V, SA.END_CURVE_A))
  assert abs(outs[-1] - goal) < 1e-6                           # reaches the end line
  assert min(outs) >= goal - 1e-9                              # never firmer than it
  assert all(b - a >= -SA.J_BUILD * 0.05 - 1e-9 for a, b in zip(outs, outs[1:], strict=False))  # built at J_BUILD
  # a real (radar) lead that close still goes through the normal phases (unchanged)
  sb = SA.StopApproach(0.05)
  real = N(status=True, dRel=0.1, vLead=0.0)
  out = sb.update(v, real, -3.0, False, True, 0.0)
  assert sb.phase != 'release' or abs(out - goal) > 1e-3


def test_altis_end_line_is_continuous_and_the_mid_brake_ends_on_it():
  # dppb 2026-11-20 (22:19:27): no cliff between the mid brake and the end - the line comes down from 12 km/h, and the
  # mid brake leaves the line's own distance in front of the stop point
  v = [k / 3.6 for k in range(0, 13)]
  a = [float(__import__('numpy').interp(x, _sa.END_CURVE_V, _sa.END_CURVE_A)) for x in v]
  assert all(a[k + 1] <= a[k] for k in range(12))                       # firmer with speed, monotonic
  assert max(a[k] - a[k + 1] for k in range(12)) <= 0.16                # no step: at most 0.16 m/s^2 per km/h
  assert a[12] == -1.35 and a[0] == a[1] == a[2] == -0.32             # owner 11-20: -0.32 held from 2 km/h to the stop
  d_end = _sa.end_distance()
  assert 7.0 < d_end < 10.0                                             # what the Altis really needs from 12 km/h
  v0, d = 40 / 3.6, 60.0
  need = _sa._need(v0, d)
  assert abs(need - (v0 ** 2 - _sa.END_V ** 2) / (2 * (d - _sa.STOP_GAP - d_end))) < 1e-9
  assert need > _sa._need_plain(v0, d) * 0.9                            # the room for the end is paid for in the mid


def test_altis_short_stop_gets_a_little_more_between_10_and_5_kph_only(monkeypatch):
  monkeypatch.setattr(_sa, 'SLOPE_ENABLED', False)   # the pre-slope law, kept for SLOPE_ENABLED = False
  monkeypatch.setattr(_sa, 'STOP_GAP', 3.8)
  # owner 2026-11-20: "if it is going to stop too close, add a little force at 10 to 5 km/h"
  line = lambda kph: float(np.interp(kph / 3.6, _sa.END_CURVE_V, _sa.END_CURVE_A))
  fits = lambda kph: _sa.STOP_GAP + kph / 3.6 * _sa.SHORT_LAG + _sa.end_distance(kph / 3.6)
  for kph in (10.0, 8.0, 6.0, 5.0):
    assert _sa._short_extra(kph / 3.6, fits(kph), line(kph)) == 0.0                 # a stop that fits: the plain line
    assert _sa._short_extra(kph / 3.6, fits(kph) + 2.0, line(kph)) == 0.0           # one ending far: the plain line
    e = _sa._short_extra(kph / 3.6, fits(kph) - 0.35 * _sa.end_distance(kph / 3.6), line(kph))
    assert 0.05 < e <= _sa.SHORT_MAX + 1e-9                                         # short: a little more, capped
  for kph in (3.0, 2.0, 1.0, 0.5, 12.0, 15.0):
    assert _sa._short_extra(kph / 3.6, _sa.STOP_GAP + 0.1, line(kph)) == 0.0        # never in the last 3 km/h, nor above 12
  # in the controller: built and taken back at SHORT_RATE, gone by 3 km/h
  sa = StopApproach(); sa.phase = 'release'; sa.a = line(9.0); sa.lead_moving = False
  outs = []
  v, d = 9.0 / 3.6, fits(9.0) - 1.5
  for _ in range(400):
    out = sa.update(v, lead(d), -3.0, False)
    outs.append((v, out, sa.short))
    v = max(v + out * sa.dt, 0.0); d -= v * sa.dt
    if v <= 0.31:
      break
  sh = np.array([o[2] for o in outs]); vv = np.array([o[0] for o in outs])
  assert sh.max() > 0.1 and np.all(np.abs(np.diff(sh)) <= _sa.SHORT_RATE * sa.dt + 1e-9)
  assert np.all(sh[vv <= 3.0 / 3.6] < 1e-9)
  assert np.all(np.array([o[1] for o in outs])[vv <= 2.0 / 3.6] >= -0.32 - 1e-9)    # the end is still -0.32


def test_altis_one_slope_worked_back_from_the_stop_point():
  # owner 2026-11-20: "3.5 m first, smooth second; work back what speed the car must have at what distance"
  a = [float(np.interp(k / 3.6, _sa.SLOPE_V, _sa.SLOPE_A)) for k in range(0, 61)]
  assert all(a[k + 1] <= a[k] for k in range(60))                               # one slope, firmer with speed
  assert a[0] == -0.32 and a[1] == -0.40 and abs(a[12] - -1.40) < 1e-9          # eased all the way to a light end
  assert max(a[k] - a[k + 1] for k in range(60)) <= 0.21                        # no step in it
  d = [_sa.slope_distance(k / 3.6) for k in range(0, 61, 5)]
  assert all(d[k + 1] > d[k] for k in range(len(d) - 1))                         # distance -> speed is one-to-one
  on = lambda kph: _sa.STOP_GAP + kph / 3.6 * _sa.SLOPE_LAG + _sa.slope_distance(kph / 3.6)
  for kph in (40.0, 20.0, 10.0, 5.0):
    assert abs(_sa.slope_k(kph / 3.6, on(kph)) - 1.0) < 1e-6                     # on the reference: the plain slope
    assert _sa.slope_k(kph / 3.6, on(kph) - 1.0) > 1.0 > _sa.slope_k(kph / 3.6, on(kph) + 1.0)  # too fast firmer / too slow lighter
  # an ideal car (gives what is asked) started on the slope ends at the stop point without firming after the first build
  monkey = {k: getattr(_sa, k) for k in ('SLOPE_DELIVER_GAIN', 'SLOPE_DELIVER_OFF')}
  _sa.SLOPE_DELIVER_GAIN, _sa.SLOPE_DELIVER_OFF = [1.0] * 7, [0.0] * 7
  try:
    sa = StopApproach(); v = 30.0 / 3.6; d = on(30.0) + 25.0; outs = []
    for _ in range(4000):
      out = sa.update(v, lead(d), 0.0, False)
      outs.append((v, out, sa.phase))
      v = max(v + min(out, 0.0) * sa.dt, 0.0); d -= v * sa.dt
      if v <= 0.31:
        break
    assert abs(d - _sa.STOP_GAP) < 0.6, d
    rel = [o[1] for o in outs if o[2] == 'release']
    i_peak = int(np.argmin(rel))
    assert all(rel[k + 1] >= rel[k] - 0.02 for k in range(i_peak, len(rel) - 1))  # after the build it only eases
    assert min(o[1] for o in outs if o[0] <= 2.0 / 3.6) >= -0.52 - 1e-9           # the end is the plain line, nothing added
  finally:
    for k, val in monkey.items():
      setattr(_sa, k, val)


def test_altis_slope_mid_may_reach_2_6_but_builds_softly():
  # owner 2026-11-21: mid decel allowed to -2.6, the first part light; the rate is what must stay gentle
  sa = StopApproach(); v = 50.0 / 3.6; outs = []
  d = _sa.STOP_GAP + 0.55 * _sa.slope_distance(v)              # far too fast for the distance: the slope scales up
  for _ in range(200):
    out = sa.update(v, lead(d), 0.0, False)
    outs.append(out)
    v = max(v + min(out, 0.0) * sa.dt, 0.0); d -= v * sa.dt
    if v < 20.0 / 3.6:
      break
  o = np.array(outs)
  assert o.min() >= _sa.SLOPE_A_MAX - 1e-9 and o.min() < -2.3                  # deeper than the plain slope, never past -2.6
  step = -np.diff(o)
  assert step.max() <= max(_sa.SLOPE_J_BUILD) * sa.dt + 1e-9                    # built at the jerk limit, no step
  first = step[(o[:-1] > -0.5) & (step > 0)]
  assert len(first) and first.max() <= 1.2 * sa.dt + 1e-9                       # the first touch softer still


def test_altis_slope_end_only_eases_and_is_kept_to_the_wheel_stop():
  # owner 2026-11-21 (route 000001e2 08:42:59): eased to -0.66, re-applied to -0.88 before the end. And the 121
  # bookmarked stops: bad ones re-firm ~0.6 inside the last 2 s, perfect ones <= 0.2
  for d0, kph in ((14.0, 20.0), (9.0, 12.0), (20.0, 20.0)):
    sa = StopApproach(); sa.phase = 'release'; sa.a = float(np.interp(kph / 3.6, _sa.SLOPE_V, _sa.SLOPE_A))
    v, d, outs = kph / 3.6, d0, []
    for k in range(3000):
      out = sa.update(v, lead(d), 0.0, False)
      outs.append((v, out))
      # a car that answers unevenly (stalls for a moment at 3 km/h, as the Altis does after a release)
      a_car = 0.0 if 3.0 / 3.6 < v < 3.4 / 3.6 and k % 3 else min(out, 0.0)
      v = max(v + a_car * sa.dt, 0.0); d -= v * sa.dt
      if v <= 0.02:
        break
    low = np.array([o[1] for o in outs if o[0] < _sa.SLOPE_EASED_V])
    assert (np.maximum.accumulate(low) - low).max() <= 0.2 + 1e-9, (d0, kph)   # once easing, no re-firming
    assert low[-1] >= -0.40 and low[-1] <= -0.10                               # a light end, and still a brake
    slow = [o[1] for o in outs if o[0] <= 0.3]
    assert len(slow) and min(slow) >= -0.42 and max(slow) <= -0.10             # kept below 0.3 m/s too (no hand-back dip)

