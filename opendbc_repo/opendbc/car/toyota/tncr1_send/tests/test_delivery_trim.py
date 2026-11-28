# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
import numpy as np
import pytest
from opendbc.car.toyota.tncr1_send.delivery_trim import DeliveryTrim, STEADY_V, RELEASE_EXTRA, HARD_CEIL


def settle(t, cmd, v, n=100):
  out = cmd
  for _ in range(n):
    out = t.apply(cmd, v, False, True)
  return out


def test_steady_part_is_off_the_pid_already_closes_it():
  # 10-15 late: the steady residual was double-counted with the stock PID (plan-vs-delivered) -> zeros for now.
  assert settle(DeliveryTrim(), -1.0, 11 / 3.6) == -1.0
  assert settle(DeliveryTrim(), -1.0, 20 / 3.6) == -1.0
  assert settle(DeliveryTrim(), -1.0, 30 / 3.6) == -1.0            # 30 km/h: matched, untouched
  assert settle(DeliveryTrim(), -1.0, 4 / 3.6) == -1.0             # below 6 km/h: nothing (end rules own it)
  assert settle(DeliveryTrim(), -0.2, 11 / 3.6) == -0.2            # a light touch is untouched


def test_release_adds_a_little_more_while_the_command_rises():
  t = DeliveryTrim()
  settle(t, -1.2, 12 / 3.6)
  outs = [t.apply(-1.2 + 0.02 * i, 12 / 3.6, False, True) for i in range(1, 40)]   # releasing at 0.67 m/s^3
  trims = [o - (-1.2 + 0.02 * i) for i, o in enumerate(outs, 1)]
  assert min(trims) <= -(STEADY_V[2] + RELEASE_EXTRA) + 0.03 and min(trims) >= -(STEADY_V[2] + RELEASE_EXTRA) - 1e-9


def test_hard_brake_at_speed_is_lifted_but_stays_hard():
  assert abs(settle(DeliveryTrim(), -2.5, 30 / 3.6) - (-2.5 + 0.25)) < 1e-9
  assert abs(settle(DeliveryTrim(), -3.0, 40 / 3.6) - (-3.0 + 0.35)) < 1e-9
  assert settle(DeliveryTrim(), -1.6, 30 / 3.6) >= -1.6 and settle(DeliveryTrim(), -1.6, 30 / 3.6) <= HARD_CEIL
  assert settle(DeliveryTrim(), -2.5, 10 / 3.6) == -2.5            # below 15 km/h no lift


def test_trim_moves_at_its_rate_and_resets_at_standstill():
  t = DeliveryTrim()
  first = t.apply(-2.5, 30 / 3.6, False, True)
  assert abs(first - (-2.5 + 0.03)) < 1e-9                          # 1.0 m/s^3 * 0.03 s
  settle(t, -2.5, 30 / 3.6)
  assert t.apply(-2.5, 0.0, True, True) == -2.5 and t.trim == 0.0
  assert t.apply(-2.5, 30 / 3.6, False, False) == -2.5


# --- after the driver's gas (GAS_*): the PCM's own extra brake is taken out of the command ---
import opendbc.car.toyota.tncr1_send.delivery_trim as DTM


def _after_gas(cmd, extra, neutral=650.0, v=3.0 / 3.6, t=2.0, gas=True, d=None):
  """long control off (gas pressed or not) for 1 s, then `cmd` for t seconds while the PCM asks its usual
  compensation + `extra` on top of what it was sent. Returns the commands sent"""
  d = d or DeliveryTrim()
  for _ in range(33):
    d.reset(gas)
  outs, sent = [], cmd
  for _ in range(int(t / d.dt)):
    net = sent + DTM.GAS_NET_K * neutral + DTM.GAS_NET_C + extra
    sent = d.apply(cmd, v, False, True, net, neutral)
    outs.append(sent)
  return d, outs


def test_after_gas_the_pcm_extra_is_taken_out():
  # 0000000a 11-24 22:55:04: cmd -0.25 after the gas at 3 km/h, the PCM asked -0.80 where -0.50 is its usual
  d, outs = _after_gas(-0.25, -0.30)
  assert outs[0] == pytest.approx(-0.25, abs=0.035)                   # no step
  assert max(np.diff(outs)) <= DTM.GAS_RATE * d.dt + 1e-9
  assert outs[-1] == pytest.approx(DTM.GAS_FLOOR, abs=1e-6)           # eased, the brake still on
  # 000000ec: the PCM added 0.14 - eased by that less the dead band
  _, outs = _after_gas(-0.30, -0.14)
  assert outs[-1] == pytest.approx(-0.30 + 0.14 - DTM.GAS_DB, abs=0.01)


def test_after_gas_nothing_when_the_pcm_adds_nothing_or_without_gas():
  _, outs = _after_gas(-0.25, 0.0)
  assert max(outs) == pytest.approx(-0.25, abs=1e-6)
  _, outs = _after_gas(-0.25, +0.10)                                   # the PCM asks less: never made firmer
  assert min(outs) == pytest.approx(-0.25, abs=1e-6) and max(outs) == pytest.approx(-0.25, abs=1e-6)
  _, outs = _after_gas(-0.25, -0.30, gas=False)                        # engaged without the gas: an ordinary brake
  assert max(outs) == pytest.approx(-0.25, abs=1e-6)
  _, outs = _after_gas(-0.25, -0.30, v=12.0 / 3.6)                     # above GAS_V
  assert max(outs) == pytest.approx(-0.25, abs=1e-6)
  d = DeliveryTrim()                                                   # no PCM signals: as before
  d.reset(True)
  assert d.apply(-0.25, 3.0 / 3.6, False, True) == -0.25


def test_after_gas_a_firm_command_is_not_eased_and_it_ends():
  _, outs = _after_gas(-1.0, -0.30)                                    # the room is short: the brake as asked
  assert max(outs) == pytest.approx(-1.0, abs=1e-6)
  _, outs = _after_gas(-0.80, -0.30)
  assert -0.80 < max(outs) < -0.60                                     # half of it
  _, outs = _after_gas(-0.25, -0.30, t=DTM.GAS_T + 1.0)                # GAS_T after the gas it is over
  assert outs[-1] == pytest.approx(-0.25, abs=1e-6)
  assert min(np.diff(outs)) >= -DTM.GAS_RATE * 0.03 - 1e-9             # ... given back as a ramp


def _creep_run(cmd_fn, v0, secs=8.0, lost=0.2):
  """toy car: the first `lost` of a positive command is lost (11-28 measurement), 0.3 s lag; returns (t, cmd, sent, a, v)"""
  from opendbc.car.toyota.tncr1_send.delivery_trim import DeliveryTrim
  dt, tr, a, v, out = 0.03, DeliveryTrim(0.03), 0.0, v0, []
  for k in range(int(secs / dt)):
    cmd = cmd_fn(k * dt)
    sent = tr.apply(cmd, v, False, True, a_ego=a)
    want = max(sent - lost, 0.0) if sent > 0.0 else sent
    a += (want - a) * dt / 0.3
    v = max(v + a * dt, 0.0)
    out.append((k * dt, cmd, sent, a, v))
  return out


def test_small_gas_at_creep_speed_is_made_up():
  # 11-28 14:29:58: +0.2 asked at 5 km/h, the car gave +0.05: now it gets what was asked, without running past it
  out = _creep_run(lambda t: 0.2, 5.0 / 3.6)
  assert max(s for _, _, s, _, _ in out) <= 0.2 + 0.25 + 1e-9
  assert 0.15 < out[int(3.0 / 0.03)][3] < 0.26 and max(a for _, _, _, a, _ in out) < 0.30
  none = _creep_run(lambda t: 0.2, 5.0 / 3.6, lost=0.0)   # a car that gives what is asked: (almost) nothing added
  assert max(s - c for _, c, s, _, _ in none) < 0.05 and none[-1][2] - none[-1][1] < 0.01


def test_creep_gas_never_touches_a_brake_or_a_firm_gas():
  from opendbc.car.toyota.tncr1_send.delivery_trim import DeliveryTrim
  # gas made up, then the plan turns to braking: continuous through zero, and the brake command is passed as it is
  out = _creep_run(lambda t: 0.2 if t < 4.0 else max(0.2 - 0.5 * (t - 4.0), -0.4), 5.0 / 3.6)
  for (_, c0, s0, _, _), (_, c1, s1, _, _) in zip(out, out[1:], strict=False):
    assert s1 - s0 > 3.5 * min(c1 - c0, 0.0) - 0.5 * 0.03 - 1e-9   # no step: the added gas leaves in proportion to the plan's own move
  assert all(abs(s - c) < 1e-9 for _, c, s, _, _ in out if c <= 0.0)
  tr = DeliveryTrim(0.03)
  for _ in range(200):   # a firm gas, standstill speeds and above 15 km/h: nothing
    assert tr.apply(0.8, 5.0 / 3.6, False, True, a_ego=0.0) == 0.8
    assert tr.apply(0.2, 2.0 / 3.6, False, True, a_ego=0.0) == 0.2
    assert tr.apply(0.2, 20.0 / 3.6, False, True, a_ego=0.0) == 0.2
    assert tr.apply(0.2, 5.0 / 3.6, False, True) == 0.2   # no measurement: nothing
