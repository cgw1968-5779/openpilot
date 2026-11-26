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
