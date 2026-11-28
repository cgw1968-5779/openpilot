import json

import numpy as np
import pytest

import dragonpilot.selfdrive.controls.lib.curve_bias_learn as CL
from dragonpilot.selfdrive.controls.lib.curve_bias import CURVE_OUTWARD_FRAC_RANGE, CURVE_OUTWARD_V_BP

DT = 0.01
RIGHT, LEFT = 0.002, -0.002   # 1/m (R 500 m); positive = right


def curve(lr, v_kph, inside, t=5.0, k=RIGHT, pressed=0.0, blinker=False, lane_changing=False, lines=True, active=True):
  """one curve at v_kph taken `inside` metres toward the inside (offset: + = car left of the lane centre), then a straight"""
  side = 1.0 if k > 0 else -1.0
  n = int(t / DT)
  for i in range(n):
    lr.update(v_kph / 3.6, k, (-side * inside, 0.0) if lines else None, active, i < pressed * n, blinker, lane_changing)
  for _ in range(200):
    lr.update(v_kph / 3.6, 0.0, (0.0, 0.0), active, False, False, False)


def test_a_curve_taken_inside_moves_that_side_outward_at_the_speed_points_around_it():
  lr = CL.CurveBiasLearner(DT, None)
  curve(lr, 65.0, 0.10)
  step = CL.LEARN_GAIN * (0.10 - CL.LEARN_DB)
  assert lr.delta['right'][2] == pytest.approx(0.5 * step, rel=0.05) and lr.delta['right'][3] == pytest.approx(0.5 * step, rel=0.05)   # 60 and 70 km/h
  assert np.count_nonzero(lr.delta['right']) == 2 and not lr.delta['left'].any()
  assert lr.extra(65.0 / 3.6, RIGHT) == pytest.approx(0.5 * step, rel=0.05)
  assert lr.extra(65.0 / 3.6, LEFT) == 0.0 and lr.extra(65.0 / 3.6, 0.0003) == 0.0      # the other side; a straight


def test_outside_moves_it_inward_and_left_curves_are_their_own():
  lr = CL.CurveBiasLearner(DT, None)
  curve(lr, 90.0, -0.12, k=LEFT)
  assert lr.delta['left'][4] == pytest.approx(CL.LEARN_GAIN * -(0.12 - CL.LEARN_DB), rel=0.05) and not lr.delta['right'].any()


def test_centred_within_the_dead_band_learns_nothing_and_one_curve_cannot_claim_much():
  lr = CL.CurveBiasLearner(DT, None)
  curve(lr, 70.0, 0.02)
  assert not lr.delta['right'].any() and lr.last[3] == 0.0
  curve(lr, 70.0, 1.5)                                   # a merge / a worn line
  assert lr.delta['right'].max() == pytest.approx(CL.LEARN_STEP_MAX)


@pytest.mark.parametrize("kw", [dict(t=2.5), dict(pressed=0.2), dict(blinker=True), dict(lane_changing=True),
                                dict(lines=False), dict(active=False), dict(k=0.0004)])
def test_what_is_not_learned_from(kw):
  # too short, the driver steering, a blinker, a lane change / lane choice, lines not trusted, lateral off, a straight
  lr = CL.CurveBiasLearner(DT, None)
  curve(lr, 70.0, 0.15, **kw)
  assert not lr.delta['right'].any() and not lr.delta['left'].any()


def test_slow_curves_are_not_learned_from():
  lr = CL.CurveBiasLearner(DT, None)
  curve(lr, 35.0, 0.15)
  assert not lr.delta['right'].any()


def test_the_turn_in_is_left_out():
  lr = CL.CurveBiasLearner(DT, None)
  n = int(5.0 / DT)
  for i in range(n):   # 0.5 m inside during the first 0.9 s only, centred after
    lr.update(70.0 / 3.6, RIGHT, (-0.5 if i * DT < 0.9 else 0.0, 0.0), True, False, False, False)
  lr.update(70.0 / 3.6, 0.0, (0.0, 0.0), True, False, False, False)
  for _ in range(200):
    lr.update(70.0 / 3.6, 0.0, (0.0, 0.0), True, False, False, False)
  assert not lr.delta['right'].any()


@pytest.mark.parametrize("s", [0.5, 1.5, 4.0])
def test_closed_loop_it_settles_without_oscillating(s):
  # the car sits 0.15 m inside with the table alone; the learned fraction moves it out by s metres per unit
  lr = CL.CurveBiasLearner(DT, None); hist = []
  for _ in range(250):
    inside = 0.15 - s * lr.extra(70.0 / 3.6, RIGHT)
    hist.append(inside)
    curve(lr, 70.0, inside, t=4.0)
  hist = np.array(hist)
  assert (np.diff(hist) <= 1e-9).all()                    # monotonic: never past the centre and back
  assert hist.min() >= CL.LEARN_DB - 1e-6
  reach = 0.15 - s * CL.LEARN_RANGE[1]                    # what the allowed range can reach
  assert hist[-1] <= max(CL.LEARN_DB, reach) + 0.01


def test_the_learned_part_stays_in_its_range():
  lr = CL.CurveBiasLearner(DT, None)
  for _ in range(60):
    curve(lr, 70.0, 0.3, t=3.5)
  assert lr.delta['right'].max() == pytest.approx(CL.LEARN_RANGE[1])
  for _ in range(120):
    curve(lr, 70.0, -0.3, t=3.5)
  assert lr.delta['right'].min() == pytest.approx(CL.LEARN_RANGE[0])


def test_kept_across_drives(tmp_path):
  p = str(tmp_path / "learn.json")
  lr = CL.CurveBiasLearner(DT, None)
  curve(lr, 65.0, 0.10)
  CL._write(p, json.dumps({'v_bp': [float(x) for x in CURVE_OUTWARD_V_BP],
                           'delta': {s: [float(x) for x in lr.delta[s]] for s in ('left', 'right')},
                           'count': {s: [float(x) for x in lr.count[s]] for s in ('left', 'right')}}))
  lr2 = CL.CurveBiasLearner(DT, p)
  assert np.allclose(lr2.delta['right'], lr.delta['right']) and np.allclose(lr2.count['right'], lr.count['right'])
  with open(p, "w") as f:
    f.write("{broken")
  assert not CL.CurveBiasLearner(DT, p).delta['right'].any()        # a damaged file: from zero, no crash
  with open(p, "w") as f:
    json.dump({'v_bp': [1.0, 2.0], 'delta': {'left': [9, 9], 'right': [9, 9]}}, f)
  assert not CL.CurveBiasLearner(DT, p).delta['right'].any()        # another table: from zero


def test_the_file_is_written_between_curves_not_in_one(tmp_path, monkeypatch):
  wrote = []
  monkeypatch.setattr(CL, '_write', lambda path, data: wrote.append(json.loads(data)))
  monkeypatch.setattr(CL.threading, 'Thread', lambda target, args, daemon: type('T', (), {'start': lambda self: target(*args)})())
  lr = CL.CurveBiasLearner(DT, str(tmp_path / "learn.json"))
  curve(lr, 65.0, 0.10)
  assert len(wrote) == 1 and wrote[0]['delta']['right'][2] > 0.0 and not lr.dirty


def test_curve_bias_takes_the_learned_part_inside_its_range():
  # the learned part moves the turn-in lag: CURVE_IN_T seconds per unit of fraction, inside the fraction's range
  from dragonpilot.selfdrive.controls.lib.curve_bias import CurveTurnIn, CURVE_IN_T
  v, ti = 65.0 / 3.6, CurveTurnIn(DT)
  base = ti.lag(0.002, v)
  assert base == pytest.approx(0.22 * CURVE_IN_T)
  assert ti.lag(0.002, v, 0.03) == pytest.approx(base + 0.03 * CURVE_IN_T)
  assert ti.lag(0.002, v, -0.03) == pytest.approx(base - 0.03 * CURVE_IN_T)
  assert ti.lag(0.002, v, 5.0) == pytest.approx(CURVE_OUTWARD_FRAC_RANGE[1] * CURVE_IN_T)
  assert ti.lag(0.0003, v, 0.05) == 0.0 and ti.update(0.0003, v, 0.05) == 0.0003          # a straight stays untouched
