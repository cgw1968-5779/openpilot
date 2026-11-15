# dppb (owner 2026-11-15): the creep stop after a low-speed gas release - soft (the PCM's request held at -0.7..-0.85)
# but it MUST stop: never lighter than -0.7, never lets the car speed up again, hands back to the planner when short
from types import SimpleNamespace

from openpilot.common.realtime import DT_CTRL
from dragonpilot.selfdrive.controls.lib import creep_stop as C

MASS = 1500.0


def cs(v=2.0 / 3.6, a=-0.5, gas=False, standstill=False):
  return SimpleNamespace(vEgo=v, aEgo=a, gasPressed=gas, standstill=standstill)


def lead(d=6.0, vl=0.0, status=True):
  return SimpleNamespace(dRel=d, vLead=vl, status=status)


def released(extra=-1.3):
  # the PCM adds `extra` beyond what we send; returns (layer, step) with step(...) -> (command, PCM request)
  c = C.CreepStop(DT_CTRL, MASS)
  c.update(0.0, cs(gas=True), lead(), 0.0, False, (0.0, 900.0))
  hist = [0.0] * (C.CS_NET_DELAY + 1)   # the PCM answers our command CS_NET_DELAY frames later

  def step(car=None, ld=None, planner=-0.2, a_target=-0.2):
    net = hist[-C.CS_NET_DELAY] + extra
    cmd = c.update(planner, car or cs(), ld or lead(), a_target, True, (net, 900.0))
    hist.append(cmd)
    return cmd, net
  return c, step


def test_never_armed_without_the_gas():
  c = C.CreepStop(DT_CTRL, MASS)
  for _ in range(100):
    assert c.update(-0.4, cs(), lead(), -0.4, True, (-1.5, 900.0)) == -0.4
  assert not c.on


def test_not_without_the_pcm_values_or_a_stopped_lead():
  c = C.CreepStop(DT_CTRL, MASS)
  c.update(0.0, cs(gas=True), lead(), 0.0, False, None)
  assert c.update(-0.3, cs(), lead(), -0.3, True, (-1.3, 900.0)) == -0.3
  c, step = released()
  assert step(ld=lead(vl=3.0))[0] == -0.2 and not c.on


NEAR = lead(d=3.8)   # close enough that the one stop starts at once


def test_far_lead_creeps_on_first_then_one_stop():
  c, step = released(extra=-1.3)
  nets = [step()[1] for _ in range(100)]
  assert not c.brake and abs(nets[-1] - C.CS_APPROACH_NET) < 0.05   # the PCM's own bite taken off, the car creeps on
  for _ in range(100):
    net = step(ld=NEAR)[1]
  assert c.brake and net <= -C.CS_NET_MIN + 1e-6


def test_pcm_request_held_in_the_soft_band():
  c, step = released(extra=-1.3)
  out = [step(ld=NEAR) for _ in range(200)]
  net = out[-1][1]
  assert -C.CS_NET_FIRM - 1e-6 <= net <= -C.CS_NET_MIN + 1e-6   # the PCM's own -1.3 bite is taken off
  assert out[-1][0] > 0.0                                       # by a positive command


def test_onset_no_sudden_bite():
  c, step = released(extra=0.0)
  nets = [step()[1] for _ in range(100)]
  steps = [b - a for a, b in zip([0.0] + nets, nets)]
  assert min(steps) >= -C.CS_ONSET_J * DT_CTRL - 1e-6


def test_speeding_up_again_firms_at_once():
  c, step = released(extra=-1.3)
  for _ in range(100):
    step(ld=NEAR)
  before = step(ld=NEAR)[1]
  for _ in range(30):
    after = step(car=cs(a=+0.2), ld=NEAR)[1]
  assert after < before - 0.3


def test_never_lighter_than_the_band_even_if_slowing_hard():
  c, step = released(extra=-1.3)
  nets = [step(car=cs(a=-1.5), ld=NEAR)[1] for _ in range(300)]
  assert max(nets[60:]) <= C.CS_END_NET + 1e-6      # even the eased end never lighter than CS_END_NET
  assert min(nets) <= -C.CS_NET_MIN                 # and the stop itself was at least the band


def test_hands_back_when_too_close_or_urgent():
  c, step = released()
  for _ in range(50):
    step()
  cmd, _ = step(ld=lead(d=2.6), planner=-1.2)
  assert not c.on and cmd <= -1.2 + 1e-9
  c, step = released()
  for _ in range(50):
    step()
  cmd, _ = step(planner=-2.0, a_target=-2.0)
  assert not c.on and cmd <= -2.0 + 1e-9


def test_into_the_hold_at_standstill():
  c, step = released()
  for _ in range(100):
    step()
  cmd, _ = step(car=cs(v=0.0, a=0.0, standstill=True), planner=-0.1)
  assert not c.on and cmd <= C.CS_HOLD_A + 1e-9


def test_ext_roundtrip():
  ext = SimpleNamespace(pcmValid=False, pcmAccelNet=0.0, pcmNeutralForce=0.0)
  C.set_pcm_ext(ext, SimpleNamespace(pcm_accel_net=-1.2, pcm_neutral_force=800.0))
  assert C.pcm_from_ext(ext) == (-1.2, 800.0)
  C.set_pcm_ext(ext, SimpleNamespace())
  assert C.pcm_from_ext(ext) is None


def test_force_decided_at_the_start_not_added_at_the_end():
  # owner 11-15: a firmer force to reach the stop position is decided when the stop starts, not added near the end
  c, step = released(extra=0.0)
  step(car=cs(v=4.0 / 3.6, a=0.0), ld=lead(d=4.6))
  assert c.brake
  level0 = c.level
  for k in range(150):
    v = max(4.0 / 3.6 - 0.9 * k * DT_CTRL, 0.05)
    step(car=cs(v=v, a=-0.9), ld=lead(d=4.6 - 0.3 * k * DT_CTRL))
  assert c.level == level0
