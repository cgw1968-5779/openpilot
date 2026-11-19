# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import numpy as np
import pytest

from dragonpilot.selfdrive.controls.lib import lane_centering
from dragonpilot.selfdrive.controls.lib.lane_centering import LaneCentering, LINE_T_LOOK, LINE_W_MAX, LINE_PRED_T

DT = 0.01
X = [0., 2., 4., 6., 8., 10., 12., 15., 18., 21., 25., 30., 35., 40., 45., 50., 55., 60., 70., 80.]


@pytest.fixture(autouse=True)
def _centre_on_the_lines(monkeypatch):
  # dppb: the module holds the Altis 5 cm right of the line centre (LINE_CENTRE_SHIFT); these tests check the loop
  # itself around the plain line centre
  monkeypatch.setattr(lane_centering, 'LINE_CENTRE_SHIFT', 0.0)


def _model(offset=0.0, curv=0.0, prob=0.9):
  """lane centre at y = offset + curv / 2 * x^2 (model y: positive = right)"""
  centre = np.array([offset + 0.5 * curv * x ** 2 for x in X])
  left = SimpleNamespace(x=X, y=list(centre - 1.65))
  right = SimpleNamespace(x=X, y=list(centre + 1.65))
  return SimpleNamespace(laneLines=[left, left, right, right], laneLineProbs=[prob, prob, prob, prob])


def _run(lc, md, v, k_model, seconds, lane_changing=False):
  out = 0.0
  for _ in range(int(seconds / DT)):
    out = lc.update(md, v, True, False, 0.0, k_model, lane_changing)
  return out


def test_takes_over_with_confident_lines_and_pursues_the_centre():
  lc = LaneCentering(DT)
  v = 100 / 3.6
  out = _run(lc, _model(offset=0.2), v, 0.0, 3.0)  # car 0.2 m LEFT of centre, model goes straight
  assert lc.line_w == LINE_W_MAX
  look = v * LINE_T_LOOK
  # dpcr: pursuit from the pose LINE_PRED_T ahead on the arc of the last command (the car already turning right)
  xd, k = v * lane_centering.line_pred_t(v), lc.k_prev   # 100 km/h: LINE_PRED_T_HIGH (dppb 11-15)
  dy = 0.2 - k * xd ** 2 / 2 - k * xd * look
  pursuit = 2 * dy / (look ** 2 + dy ** 2)  # steers right, pure pursuit
  assert 0.0 < pursuit < 2 * 0.2 / (look ** 2 + 0.04)  # less than from the current pose: no overshoot push
  assert abs(out - (LINE_W_MAX * pursuit + (1 - LINE_W_MAX) * lc.correction)) < 1e-5


def test_follows_the_lane_curvature_not_the_model():
  lc = LaneCentering(DT)
  v = 100 / 3.6
  out = _run(lc, _model(curv=1 / 1000), v, 1 / 800, 4.0)  # model asks tighter than the lane (4 s: the command history settles)
  # 100 km/h: the pursuit's curvature carries the outward fraction (owner 11-08 route ff)
  from dragonpilot.selfdrive.controls.lib.lane_centering import LINE_OUT_FRAC_V
  k_lane = (1 - LINE_OUT_FRAC_V[-1]) / 1000
  assert abs(out - (LINE_W_MAX * (k_lane - 1 / 800) + (1 - LINE_W_MAX) * lc.correction)) < 2e-5


def test_no_outward_fraction_at_or_below_85_kph():
  lc = LaneCentering(DT)
  v = 84 / 3.6
  out = _run(lc, _model(curv=1 / 1000), v, 1 / 800, 4.0)
  assert abs(out - (LINE_W_MAX * (1 / 1000 - 1 / 800) + (1 - LINE_W_MAX) * lc.correction)) < 2e-5


def test_line_mode_from_walking_speed_and_not_without_model_curvature():
  # dppb 2026-11-19 (owner: "below 30 km/h hold the centre, no swaying"; then "up to 60"): fades in 1 -> 3 km/h
  # (11-11 20 -> 25, 11-10 30 -> 40, was 70 -> 80) and carries the whole command below 60 km/h
  from dragonpilot.selfdrive.controls.lib.lane_centering import LINE_W_MAX_LOW
  lc = LaneCentering(DT)
  _run(lc, _model(offset=0.2), 0.8 / 3.6, 0.0, 3.0)
  assert lc.line_w == 0.0
  lc_half = LaneCentering(DT)
  _run(lc_half, _model(offset=0.2), 2.0 / 3.6, 0.0, 3.0)
  assert abs(lc_half.line_w - 0.5 * LINE_W_MAX_LOW) < 1e-6
  for kph in (5, 15, 27, 45, 58):
    lc_full = LaneCentering(DT)
    _run(lc_full, _model(offset=0.2), kph / 3.6, 0.0, 3.0)
    assert lc_full.line_w == LINE_W_MAX_LOW == 1.0, kph
  lc2 = LaneCentering(DT)
  for _ in range(300):
    out = lc2.update(_model(offset=0.2), 100 / 3.6, True, False)
  assert lc2.line_w == 0.0 and out == lc2.correction


def test_low_speed_model_swing_stays_out_of_the_command():
  # 11-19 16:55 (30 -> 1 km/h on a straight lane): the model's action swung -5.6 / +1.8 / -3.2 / +6.4 e-3 while the
  # lines did not move. With the lines trusted the command is the line centre's, whatever the model asks
  lc = LaneCentering(DT)
  v = 15 / 3.6
  _run(lc, _model(), v, 0.0, 2.0)
  for k_model in (-5.6e-3, 1.8e-3, -3.2e-3, 6.4e-3):
    out = _run(lc, _model(), v, k_model, 1.0)
    assert lc.line_w == 1.0
    assert abs(k_model + out) < 0.5e-3, (k_model, out)       # command = model + out ~ 0 (straight lane, car centred)


def test_low_speed_turn_the_lines_do_not_show_goes_back_to_the_model():
  # a junction: the model turns (R 30 m) while the lines ahead run straight -> the model's command, within 0.5 s
  from dragonpilot.selfdrive.controls.lib.lane_centering import LINE_MAX_DIFF_K
  lc = LaneCentering(DT)
  v = 15 / 3.6
  _run(lc, _model(), v, 0.0, 2.0)
  assert lc.line_w == 1.0
  out = _run(lc, _model(), v, 1 / 30, 0.6)
  assert 1 / 30 > LINE_MAX_DIFF_K and lc.line_w == 0.0 and abs(out) < 1e-3


def test_hands_back_on_weak_lines_lane_change_or_far_target():
  v = 100 / 3.6
  for md, k_model, lane_changing in ((_model(prob=0.6), 0.0, False), (_model(), 0.0, True), (_model(curv=1 / 300), 0.0, False)):
    lc = LaneCentering(DT)
    _run(lc, _model(), v, 0.0, 3.0)
    assert lc.line_w == LINE_W_MAX
    _run(lc, md, v, k_model, 0.6, lane_changing)
    assert lc.line_w == 0.0


def test_weight_rises_in_two_seconds():
  lc = LaneCentering(DT)
  _run(lc, _model(), 100 / 3.6, 0.0, 1.0)
  assert 0.45 < lc.line_w < 0.55


def test_weight_capped_at_line_w_max():
  lc = LaneCentering(DT)
  _run(lc, _model(), 100 / 3.6, 0.0, 5.0)
  assert LINE_W_MAX == 0.85 and lc.line_w == LINE_W_MAX  # dpcr 0.85 (tncr1 0.7)


def test_centre_fits_made_once_per_model_frame(monkeypatch):
  import dragonpilot.selfdrive.controls.lib.lane_centering as lcm
  calls = []
  real = np.polyfit
  monkeypatch.setattr(lcm.np, 'polyfit', lambda *a, **k: calls.append(1) or real(*a, **k))
  lc = LaneCentering(DT)
  md = _model(offset=0.2)
  for _ in range(5):  # one 20 Hz model frame = five 100 Hz cycles
    lc.update(md, 100 / 3.6, True, False, 0.0, 0.0, False)
  assert len(calls) == 2  # the near fit (measure) and the line-mode fit, once each
  lc.update(_model(offset=0.1), 100 / 3.6, True, False, 0.0, 0.0, False)  # a new frame refits
  assert len(calls) == 4



def test_constant_command_history_is_the_old_arc():
  from dragonpilot.selfdrive.controls.lib.lane_centering import line_pred_t
  lc = LaneCentering(DT)
  v, k = 100 / 3.6, 1 / 900
  PT = line_pred_t(v)
  for _ in range(200):
    lc._push_command(k)
  y, psi = lc.predicted_pose(v)
  xd = v * PT
  assert abs(psi - k * xd) < 1e-4 * k * xd + 1e-9
  assert abs(y - k * xd * xd / 2) < 0.02 * k * xd * xd / 2


def test_command_history_sees_a_correction_already_sent():
  # the car was sent a right correction in the last 0.3 s: the pose after the delay is further right than the arc of
  # the (now straight) last command says, so the pursuit asks less
  lc = LaneCentering(DT)
  v = 100 / 3.6
  n = int(round(LINE_PRED_T / DT))
  for i in range(n):
    lc._push_command(2e-4 if i >= n - 30 else 0.0)
  lc._push_command(0.0)
  y, psi = lc.predicted_pose(v)
  assert y > 0.0 and psi > 0.0


def test_dppb_holds_the_car_5_cm_right_of_the_line_centre(monkeypatch):
  # LINE_CENTRE_SHIFT 0.05: a car already 5 cm right of the line centre gets no correction, one on the centre is
  # steered right (+ curvature)
  monkeypatch.setattr(lane_centering, 'LINE_CENTRE_SHIFT', 0.05)
  v = 28.0
  lc = LaneCentering(0.01)
  on_target = _run(lc, _model(offset=-0.05), v, 0.0, 6.0)
  lc = LaneCentering(0.01)
  on_centre = _run(lc, _model(offset=0.0), v, 0.0, 6.0)
  assert abs(on_target) < 2e-5
  assert on_centre > 1e-5


def test_low_speed_integral_held_through_a_short_line_drop_out():
  # dppb 11-11: below 70 km/h the integral survives a drop-out of up to I_HOLD_T, then eases out; from 70 it resets
  from dragonpilot.selfdrive.controls.lib.lane_centering import I_HOLD_T
  for kph, held in ((40, True), (75, False)):
    lc = LaneCentering(DT)
    for _ in range(800):
      lc.update(_model(offset=0.2), kph / 3.6, True, False)
    i0 = lc.integral
    assert i0 > 0.0
    for _ in range(int(I_HOLD_T / DT) - 5):
      lc.update(None, kph / 3.6, True, False)
    assert (lc.integral == i0) if held else (lc.integral == 0.0)
    lc.update(None, kph / 3.6, True, True)  # the driver steers: gone at any speed
    assert lc.integral == 0.0


def test_curves_below_70_kph_are_centred_too_except_near_the_steering_limit():
  # dppb 2026-11-19 (owner: "the big 50-60 km/h curves run to the inside; steer from the lines up to 60"): the 11-11
  # straights-only gate is replaced by a steering-limit gate - the 11-11 takeover (59 km/h, R 65 m, 3.1-4.1 m/s^2) came
  # from commanding less than the model where the EPS was already at its limit (~2.7 m/s^2)
  from dragonpilot.selfdrive.controls.lib.lane_centering import LINE_W_MAX_LOW, LOW_CURVE_ACCEL
  v = 55 / 3.6
  curve = LaneCentering(DT)
  _run(curve, _model(offset=0.2, curv=1 / 300), v, 1 / 300, 4.0)       # R 300 m, 0.78 m/s^2
  assert curve.line_w == LINE_W_MAX_LOW and curve._straight_gate(v) == 1.0
  tight = LaneCentering(DT)
  k = LOW_CURVE_ACCEL[1] / v ** 2 * 1.05                                # the model asks more than the limit gate
  _run(tight, _model(offset=0.2, curv=k), v, k, 4.0)
  assert tight.line_w == 0.0 and tight.correction == 0.0 and tight.line_i == 0.0
  fast = LaneCentering(DT)
  _run(fast, _model(offset=0.0, curv=1 / 1000), 100 / 3.6, 1 / 1000, 4.0)   # above 70 km/h: as before
  assert fast.line_w == LINE_W_MAX and fast.line_i == 0.0


def test_curve_trim_takes_off_what_the_near_lines_say_is_left_over():
  # with the far lines pursued the 55-64 km/h curves still ran 24-49 cm inside (sim of 11-18 21:09 / 21:16 / 21:33):
  # a slow integral of the near offset is added to the line target - in a curve, from 30-40 km/h, bounded, and it
  # unwinds faster than it builds
  from dragonpilot.selfdrive.controls.lib.lane_centering import LINE_I_LIMIT
  v = 55 / 3.6
  lc = LaneCentering(DT)
  _run(lc, _model(offset=0.3, curv=1 / 250), v, 1 / 250, 3.0)          # car 0.3 m LEFT of the centre in a right curve
  built = lc.line_i
  assert 0.0 < built <= LINE_I_LIMIT                                     # pushes right
  _run(lc, _model(offset=0.3, curv=1 / 250), v, 1 / 250, 20.0)
  assert lc.line_i == LINE_I_LIMIT
  _run(lc, _model(offset=-0.3, curv=1 / 250), v, 1 / 250, 1.0)         # back across the centre: unwinds 6 x faster
  assert LINE_I_LIMIT - lc.line_i > 3.0 * (built / 3.0)                # in 1 s more than 3 x what 1 s built
  straight = LaneCentering(DT)
  _run(straight, _model(offset=0.3), v, 0.0, 5.0)
  assert straight.line_i == 0.0                                          # not on a straight
  slow = LaneCentering(DT)
  _run(slow, _model(offset=0.3, curv=1 / 250), 25 / 3.6, 1 / 250, 5.0)
  assert slow.line_i == 0.0                                              # not below 30 km/h
