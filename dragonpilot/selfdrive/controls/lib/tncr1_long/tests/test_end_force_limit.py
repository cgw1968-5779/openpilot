# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
import numpy as np
import pytest
import dragonpilot.selfdrive.controls.lib.tncr1_long.end_force_limit as EFL
from openpilot.common.realtime import DT_CTRL
from dragonpilot.selfdrive.controls.lib.tncr1_long.end_force_limit import EndForceLimit, HOLD_RATE, POST_T, \
  SETTLE_MIN, SETTLE_MAX, V_LINE, END_ACCEL, CEIL_BP, CEIL_A, \
  END_RELEASE_RATE, END_FIRM_RATE, END_GAP_MIN


@pytest.fixture(autouse=True)
def _no_feel_for_the_older_tests(monkeypatch, request):
  # the tests written before FEEL_* leave a_ego at its default (-1.0 = "slowing"); the feel has its own tests below
  if 'feel' not in request.node.name:
    monkeypatch.setattr(EFL, 'FEEL_KI', 0.0)



def ceil(kph):
  return float(np.interp(kph / 3.6, CEIL_BP, CEIL_A))


def test_line_from_2kph_to_minus_030():
  e = EndForceLimit()
  assert e.update(-0.92, 2.8 / 3.6, False, True) == ceil(2.8)    # above the 2 km/h line: the ceiling (-0.37, owner 10-22)
  assert e.update(-0.80, 2.0 / 3.6, False, True) == ceil(2.0)    # tncr: the ceiling (-0.60) caps it; a_line = -0.60
  out = e.update(-0.70, 1.0 / 3.6, False, True)                  # 18:51: -0.70 at 1.4 km/h asked -> ceiling -0.38
  assert abs(out - max(ceil(1.0), END_ACCEL + (ceil(2.0) - END_ACCEL) * 0.5)) < 1e-9
  assert e.update(-0.46, 0.01, False, True) >= END_ACCEL - 0.01 * 0.5 / V_LINE - 1e-9  # at the stop: ~-0.34
  assert e.update(-0.46, 0.0, False, True) == END_ACCEL


def test_lighter_passes_and_never_firmer_than_030_at_zero():
  e = EndForceLimit()
  e.update(-0.25, 2.5 / 3.6, False, True)
  assert e.update(-0.25, 1.0 / 3.6, False, True) == -0.25
  assert e.update(-2.5, 0.0, False, True) == END_ACCEL            # no exceptions


def test_standstill_continues_into_the_hold_without_a_step():
  e = EndForceLimit()
  e.update(-0.6, 2.0 / 3.6, False, True)
  assert EndForceLimit().update(-0.6, 0.0, False, True) == END_ACCEL
  assert e.update(-0.6, 0.0, False, True, -0.5) == END_ACCEL      # at the stop the line is back on END_ACCEL
  # owner 2026-11-09: the instant of the stop and the body's settle stay on END_ACCEL (the hold asks -1.2 at once)
  for _ in range(int(SETTLE_MIN / DT_CTRL) - 1):
    assert e.update(-1.2, 0.0, True, True, -0.5) == END_ACCEL      # still settling (body moving)
  out = e.update(-1.2, 0.0, True, True, 0.0)                      # settled after SETTLE_MIN: firms up at HOLD_RATE
  assert abs(out - (END_ACCEL - HOLD_RATE * DT_CTRL)) < 1e-9
  for _ in range(300):
    out = e.update(-1.2, 0.0, True, True)
  assert out == -1.2 and e.out is None
  assert e.update(0.5, 0.0, True, True) == 0.5                    # launch passes


def test_settle_ends_at_settle_max_even_if_the_body_still_moves():
  e = EndForceLimit()
  e.update(-0.6, 2.0 / 3.6, False, True)
  e.update(-0.6, 0.0, False, True, -0.5)
  outs = [e.update(-1.2, 0.0, True, True, -0.5) for _ in range(int(SETTLE_MAX / DT_CTRL) + 20)]
  n = int(SETTLE_MAX / DT_CTRL)
  assert all(o == END_ACCEL for o in outs[:n - 1])
  assert outs[-1] < END_ACCEL and abs((outs[-2] - outs[-1]) - HOLD_RATE * DT_CTRL) < 1e-9


def test_engaged_below_the_line():
  e = EndForceLimit()
  ceil15 = ceil(1.5)                                                      # tncr: the fixed ceiling caps -0.9 at 1.5 km/h
  assert abs(e.update(-0.9, 1.5 / 3.6, False, True) - ceil15) < 1e-9     # and the line is anchored there
  out = e.update(-0.9, 0.75 / 3.6, False, True)
  assert abs(out - max(ceil(0.75), END_ACCEL + (ceil15 - END_ACCEL) * 0.5)) < 1e-9
  assert e.update(-0.9, 0.0, False, True) == END_ACCEL


def test_stall_lets_the_line_go_and_reanchors():
  e = EndForceLimit()
  e.update(-0.6, 2.0 / 3.6, False, True, -0.5)
  assert e.update(-0.8, 1.5 / 3.6, False, True, -0.5) > -0.8      # on the line
  for _ in range(30):                                              # rolling at 1.5 km/h without slowing for 0.3 s
    out = e.update(-0.8, 1.5 / 3.6, False, True, 0.0)
  assert out == ceil(1.5)                                          # a stall is not the stop: the request passes - up to the
  e.update(-0.8, 1.4 / 3.6, False, True, -0.4)                     # ceiling (tncr: -0.49 at 1.5 km/h); then re-anchored there
  out = e.update(-0.8, 0.7 / 3.6, False, True, -0.4)
  assert abs(out - max(ceil(0.7), END_ACCEL + (ceil(1.5) - END_ACCEL) * 0.7 / 1.5)) < 1e-6
  assert e.update(-0.8, 0.0, False, True, -0.4) == END_ACCEL


def test_fixed_ceiling_from_5kph_caps_everything():  # tncr 2026-10-15 evening: -2.0 @5 -> -0.34 @0, no exceptions
  e = EndForceLimit()
  assert e.update(-2.7, 6.0 / 3.6, False, True) == -2.7                       # above 5 km/h: untouched
  # owner: 5 km/h may still need a firm brake; the last 1-0 km/h must be below -0.6, best -0.34
  # 2026-10-22 owner: 3 -0.37, 2 -0.37, 1 -0.36, 0 -0.34 (below 2 km/h the line from the 2 km/h anchor: 1 km/h -0.355)
  # the table itself (each point on its own: from one frame to the next the cut now comes off as a ramp, see below)
  for kph, want in ((5.0, -2.0), (4.0, -1.185), (3.0, -0.37), (2.0, -0.37), (1.0, -0.36), (0.0, END_ACCEL)):
    out = EndForceLimit().update(-2.7, kph / 3.6, False, True)
    assert abs(out - want) < 0.005, (kph, out)
  e2 = EndForceLimit()
  assert e2.update(-0.65, 5.0 / 3.6, False, True) == -0.65                    # an ordinary end (END curve) never touches it
  assert e2.update(-0.60, 3.5 / 3.6, False, True) == -0.60                    # -0.78 ceiling at 3.5 km/h
  assert EndForceLimit().update(-0.60, 3.0 / 3.6, False, True) == -0.37      # owner line from 3 km/h
  assert -0.60 < e2.update(-0.60, 3.0 / 3.6, False, True) < -0.37             # reached from 3.5 km/h: as a ramp
  e3 = EndForceLimit()                                                        # into the standstill hold without a step
  e3.update(-2.7, 1.0 / 3.6, False, True)
  assert e3.update(-2.7, 0.0, False, True, -0.5) == END_ACCEL
  assert e3.update(-1.2, 0.0, True, True) == END_ACCEL           # the stop itself: END_ACCEL (settle, owner 11-09)


def _arrive(e, a0, v0, decel, lead_d=None, lead_v=None, req=None):
  """a stop at a steady decel from v0 with a firm request; returns the outputs while moving"""
  v, d, outs, req = v0, lead_d, [], a0 if req is None else req
  e.update(a0, v0, False, True, -decel, lead_d=d, lead_v=lead_v)
  outs.append(e.sent)
  while v > 0.0:
    v = max(v - decel * DT_CTRL, 0.0)
    if d is not None:
      d -= v * DT_CTRL
    outs.append(e.update(req, v, False, True, -decel, lead_d=d, lead_v=lead_v))
  return outs


def test_hot_arrival_is_released_as_a_ramp_back_onto_the_end_value():
  # tncr 2026-10-28, route bb 09:17:46: arriving at 5.4 km/h with -2.26 the table cut it to -0.38 in 0.4 s, the brake let
  # go and the creep rolled the car toward the lead. Now the cut comes off as a ramp, just fast enough to be on
  # END_ACCEL as the wheels stop (no step, no jolt at the stop).
  outs = _arrive(EndForceLimit(), -2.26, 6.0 / 3.6, 2.0)
  steps = np.diff(outs)
  assert np.all(steps >= -1e-9)                                     # only ever lighter on the way in
  assert max(steps) < 0.1                                           # no step (the table alone dropped 1.6 in a frame)
  assert abs(outs[-1] - END_ACCEL) < 0.02                           # on the end value at the stop
  assert outs[1] < -2.0                                             # still firm right after crossing 5 km/h


def test_ordinary_stop_is_unchanged():
  # an ordinary end arrives at ~-0.6..-0.7 and is taken to -0.37 by the table slower than END_RELEASE_RATE anyway
  a, b = EndForceLimit(), EndForceLimit()
  v = 5.0 / 3.6
  for i in range(200):
    req = -0.7 + 0.33 * min(i / 50.0, 1.0)
    old = float(np.interp(v, CEIL_BP, CEIL_A))
    out = b.update(req, v, False, True, -0.6, lead_d=6.0, lead_v=0.0)
    if v >= V_LINE:
      assert abs(out - max(req, old)) < 1e-9
    v = max(v - 0.6 * DT_CTRL, 0.0)
    if v == 0.0:
      break
  assert END_RELEASE_RATE >= 0.74                                   # route bb 09:21:53: the table's own release, 0.74 m/s^3


def test_creep_toward_a_stopped_lead_is_held_gradually():
  # rolling at 2 km/h, not slowing, 2.6 m behind a stopped lead, the plan asking -0.8: the table would cut it to -0.37;
  # the need to stop END_GAP_MIN short keeps (and gradually builds) what the state needs instead
  e = EndForceLimit()
  v, d = 2.0 / 3.6, 2.6
  e.update(-0.37, v, False, True, 0.1, lead_d=d, lead_v=0.0)
  outs = []
  for _ in range(100):
    d -= v * DT_CTRL
    outs.append(e.update(-0.8, v, False, True, 0.1, lead_d=d, lead_v=0.0))
  assert min(outs) < -0.45                                          # firmer than the table's -0.37
  assert min(outs) >= -0.8                                          # never firmer than the plan
  assert np.all(np.diff(outs) >= -END_FIRM_RATE * DT_CTRL - 1e-9)   # built gradually
  assert d > END_GAP_MIN - 0.5


def test_rebound_right_after_the_stop_keeps_the_hold():
  # dpcr 2026-11-08, route f3 23:58:49.6: stopped on -0.35, the rebound + creep rolled the car at 0.45-1.2 km/h and the
  # end ceiling put the request back to -0.35 -> 0.7 m creep. Now the hold keeps firming through it
  e = EndForceLimit()
  e.update(-0.6, 2.0 / 3.6, False, True)
  e.update(-0.6, 0.0, False, True, -0.5)
  for _ in range(30):                                   # 0.3 s standing: settling on END_ACCEL (owner 11-09)
    out = e.update(-1.2, 0.0, True, True)
    assert out == END_ACCEL
  held = out
  for _ in range(30):                                   # rolls again at 1 km/h, standstill flag off: firms at once
    out2 = e.update(-1.2, 1.0 / 3.6, False, True, 0.9)
    assert out2 <= held + 1e-9                          # never back toward the end ceiling
    held = out2
  assert abs(out2 - (END_ACCEL - 30 * HOLD_RATE * DT_CTRL)) < 1e-6
  # a real launch (request >= 0) ends it at once
  assert e.update(0.4, 1.0 / 3.6, False, True) == 0.4
  # after POST_T a roll is a roll again (the ceiling applies)
  e = EndForceLimit()
  e.update(-0.6, 0.0, True, True)
  for _ in range(int(POST_T / DT_CTRL) + 5):
    e.update(-1.2, 0.0, True, True)
  assert e.update(-1.2, 1.0 / 3.6, False, True, 0.2) >= -0.37


def _feel_stop(real_of, v0=3.4 / 3.6, accel=-0.37, **kw):
  """a stop from v0 with the request `accel`; the car's real decel is real_of(sent). Returns rows (v, sent, real)"""
  e = EFL.EndForceLimit(); v, a, rows = v0, real_of(accel), []
  for _ in range(1500):
    out = e.update(accel, v, False, True, a, **kw)
    a += (real_of(out) - a) * 0.01 / 0.3          # the car answers with a 0.3 s lag
    rows.append((v, out, a)); v = max(v + a * 0.01, 0.0)
    if v <= 0.0:
      break
  return e, np.array(rows)


def test_feel_eases_the_end_when_the_car_slows_harder_than_the_feel_line():
  # route 0000000a 22:38:12: -0.37 asked, the Cross on -0.50 down to the wheel stop
  e, r = _feel_stop(lambda s: s - 0.13)
  assert r[0, 1] == pytest.approx(-0.37, abs=0.01)
  assert (np.diff(r[r[:, 2] < -0.30, 1]) >= -1e-9).all()                  # only eases while the car slows this hard
  assert r[:, 1].min() >= -0.37 - 1e-6                                    # and nothing is ever firmer than the end value
  assert np.diff(r[:, 1]).max() <= EFL.FEEL_RATE * 0.01 + 1e-9            # slowly
  assert EFL.FEEL_FLOOR + 1e-9 >= r[:, 1].max() > -0.30                   # eased, never past the floor
  assert r[-1, 2] > -0.42                                                 # the car is softer at the wheel stop than -0.50
  # ... and the instant of the stop and the settle stay on what was eased (no step back to the end value)
  sent = r[-1, 1]
  assert e.update(-0.37, 0.0, True, True, 0.3) == pytest.approx(sent, abs=1e-6)
  assert e.update(-1.2, 0.0, True, True, 0.3) == pytest.approx(sent, abs=1e-6)


def test_feel_leaves_a_soft_stop_alone():
  # route 00000005 09:04:45: -0.37 asked, the car on -0.10 .. -0.20 - nothing to ease
  _, r = _feel_stop(lambda s: s + 0.22)
  assert np.allclose(r[r[:, 0] > 2.0 / 3.6, 1], -0.37, atol=1e-6)
  assert r[:, 1].max() <= -0.33


def test_feel_never_goes_under_what_the_room_needs():
  # 2.6 m behind a standing lead at 3 km/h on a car that slows hard: the end keeps what still stops END_GAP_MIN short
  e = EFL.EndForceLimit(); v, d, a = 3.0 / 3.6, 2.6, -0.6
  for _ in range(1500):
    need = EFL.end_need(v, d, 0.0)
    out = e.update(-0.9, v, False, True, a, lead_d=d, lead_v=0.0)
    assert out <= -min(need, 0.9) + 1e-6
    a += (out - 0.13 - a) * 0.01 / 0.3; v = max(v + a * 0.01, 0.0); d -= v * 0.01
    if v <= 0.0:
      break
  assert d > EFL.END_GAP_MIN - 0.05


def test_feel_is_forgotten_for_the_next_stop():
  e, _ = _feel_stop(lambda s: s - 0.13)
  assert e.feel > 0.0
  e.update(-0.5, 6.0 / 3.6, False, True, -0.5)
  assert e.feel == 0.0


def test_feel_gives_the_ease_back_when_the_car_stops_slowing():
  # the hybrid's creep eats a light request (route 00000159: -0.30 gave ~0): eased first, then the car hardly slows -
  # the ease comes back slowly, never past the end value
  e = EFL.EndForceLimit(); v = 3.0 / 3.6; outs = []
  for _ in range(60):
    outs.append(e.update(-0.37, v, False, True, -0.60))
  assert outs[-1] > -0.30
  for _ in range(200):
    outs.append(e.update(-0.37, v, False, True, -0.02))
  assert outs[-1] == pytest.approx(-0.37, abs=1e-6)
  assert (-np.diff(outs)).max() <= EFL.FEEL_RATE * 0.01 + 1e-9
