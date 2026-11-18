# dppb (owner 2026-11-18): the brake is let off only in the last instant of a stop - below 1 km/h the PCM's request is
# walked to -0.30 (never firmer), nothing changes above 1 km/h, the car is held again once it is at rest
from types import SimpleNamespace

from openpilot.common.realtime import DT_CTRL
from dragonpilot.selfdrive.controls.lib import final_ease as F


def cs(v, a=-0.4, standstill=False, gas=False):
  return SimpleNamespace(vEgo=v / 3.6, aEgo=a, standstill=standstill, gasPressed=gas)


def stop(extra=-0.25, a=-0.4, cmd=-0.40, v0=3.0):
  # a stop slowing at `a` from v0 km/h with the other layers asking `cmd`; the PCM adds `extra` beyond our command
  fe = F.FinalEase(DT_CTRL)
  rows, v, sent = [], v0, [cmd] * 6
  while v > 0.55:
    net = sent[-6] + extra
    out = fe.update(cmd, -0.3, cs(v, a), True, (net, 0.0))
    sent.append(out)
    rows.append((v, out, net))
    v += a * 3.6 * DT_CTRL
  return fe, rows, sent, extra


def test_nothing_changes_above_1_kph():
  _, rows, _, _ = stop()
  assert all(out == -0.40 for v, out, _ in rows if v > 1.0)


def test_eases_in_the_last_instant_and_never_firms():
  _, rows, _, _ = stop()
  eased = [(v, out) for v, out, _ in rows if out != -0.40]
  assert eased and eased[0][0] <= 1.0 and eased[0][0] > 0.6
  outs = [out for _, out, _ in rows]
  assert all(b >= a - 1e-9 for a, b in zip(outs, outs[1:]))   # never firmer while moving
  assert all(out >= -0.40 for out in outs)                     # never firmer than the other layers


def test_pcm_request_reaches_minus_030_at_the_stop():
  fe, rows, sent, extra = stop(extra=-0.25)
  assert sent[-6] + extra >= F.FE_NET - 0.02
  assert max(out for _, out, _ in rows) <= F.FE_CMD_MAX


def test_firms_back_at_once_at_the_standstill_flag():
  fe, _, sent, extra = stop()
  outs = [fe.update(-0.30, 0.0, cs(0.0, 0.0, standstill=True), True, (sent[-6] + extra, 0.0)) for _ in range(40)]
  assert outs[0] < sent[-1]                                     # firming starts at the flag (no wait)
  assert all(b <= a + 1e-9 for a, b in zip(outs, outs[1:]))
  assert max(a - b for a, b in zip(outs, outs[1:])) <= F.FE_PARK_RATE * DT_CTRL + 1e-9
  assert outs[-1] <= F.FE_PARK + 1e-6                           # at least the park level within ~0.1 s


def test_pcm_request_not_left_lighter_than_minus_030():
  # the PCM's extra shrinks after the ease (11-18 08:00 the request ended at -0.21): the command firms back to hold -0.30
  fe = F.FinalEase(DT_CTRL)
  sent, v = [-0.40] * 6, 0.9
  extra = -0.35
  for k in range(60):
    if k == 20:
      extra = -0.15
    net = sent[-6] + extra
    sent.append(fe.update(-0.40, -0.3, cs(v, -0.4), True, (net, 0.0)))
  assert abs(sent[-6] + extra - F.FE_NET) < 0.03


def test_only_one_ease_per_stop():
  fe = F.FinalEase(DT_CTRL)
  fe.update(-0.40, -0.3, cs(0.8, -0.4), True, (-0.6, 0.0))
  for _ in range(40):
    fe.update(-0.40, -0.3, cs(0.8, 0.3), True, (-0.6, 0.0))   # creeping on: handed back
  for _ in range(100):
    out = fe.update(-0.40, -0.3, cs(0.8, -0.4), True, (-0.6, 0.0))
  assert out == -0.40 and fe.uses == 1


def test_hands_back_when_the_car_creeps_on():
  fe = F.FinalEase(DT_CTRL)
  out = fe.update(-0.40, -0.3, cs(0.9, -0.4), True, (-0.6, 0.0))
  for _ in range(40):
    out = fe.update(-0.40, -0.3, cs(0.9, -0.3), True, (-0.6, 0.0))
  assert out > -0.40
  for _ in range(100):
    out = fe.update(-0.40, -0.3, cs(1.0, 0.2), True, (-0.6, 0.0))
  assert abs(out + 0.40) < 1e-6 and not fe.on


def test_off_for_gas_urgent_and_inactive():
  for kw in (dict(gas=True),):
    fe = F.FinalEase(DT_CTRL)
    assert fe.update(-0.40, -0.3, cs(0.8, -0.4, **kw), True, (-0.6, 0.0)) == -0.40
  fe = F.FinalEase(DT_CTRL)
  assert fe.update(-0.40, -2.0, cs(0.8, -0.4), True, (-0.6, 0.0)) == -0.40
  fe = F.FinalEase(DT_CTRL)
  assert fe.update(-0.40, -0.3, cs(0.8, -0.4), False, (-0.6, 0.0)) == -0.40


def test_never_arms_without_the_pcm_request():
  fe = F.FinalEase(DT_CTRL)
  assert all(fe.update(-0.40, -0.3, cs(v / 10, -0.4), True, None) == -0.40 for v in range(9, 0, -1))
