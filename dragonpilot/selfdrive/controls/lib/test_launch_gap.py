# dppb (owner 2026-11-05): follow a lead that starts moving within ~0.3 s; the MPC takes the gap we stopped at as enough
from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib import long_mpc as lm

DT = 0.05


def mpc():
  return SimpleNamespace(dt=DT, launch_t=0.0, launch_offset=0.0)


def lead(d=3.5, v=0.0, status=True):
  return SimpleNamespace(status=status, dRel=d, vLead=v)


def step(m, ld, v_ego=0.0):
  return lm.LongitudinalMpc.launch_gap(m, ld, v_ego)


def test_radar_noise_spikes_do_not_start():
  m = mpc()
  for k in range(400):
    v = 0.6 if k % 20 == 0 else (0.3 if k % 20 == 1 else 0.05)  # single spikes, as recorded with both stopped
    assert step(m, lead(v=v)) == 0.0


def test_lead_moving_away_gives_the_gap():
  m = mpc()
  outs = [step(m, lead(v=0.3)) for _ in range(5)]
  assert outs[1] == 0.0                                     # not after one frame
  k = next(i for i, o in enumerate(outs) if o > 0.0)
  assert (k + 1) * DT <= lm.LAUNCH_T + DT + 1e-9            # within LAUNCH_T
  assert abs(outs[k] - (lm.STOP_DISTANCE + lm.STOP_EXTRA - 3.5 + lm.LAUNCH_EXTRA)) < 1e-9


def test_fades_once_moving_and_faster_if_the_lead_stops():
  m = mpc()
  for _ in range(5):
    step(m, lead(v=0.5))
  o0 = m.launch_offset
  assert step(m, lead(v=0.5)) == o0                         # still stopped: kept
  o1 = step(m, lead(d=4.0, v=1.0), v_ego=0.5)
  assert abs((o0 - o1) - lm.LAUNCH_FADE * DT) < 1e-9        # moving: fades slowly
  o2 = step(m, lead(d=4.0, v=0.0), v_ego=0.5)
  assert abs((o1 - o2) - lm.LAUNCH_FADE_STOPPED * DT) < 1e-9  # the lead stopped again: fades fast
  for _ in range(200):
    step(m, lead(d=4.0, v=0.0), v_ego=0.2)
  assert m.launch_offset == 0.0 and m.launch_t == 0.0


def test_far_lead_or_no_lead_untouched():
  m = mpc()
  for _ in range(20):
    assert step(m, lead(d=8.0, v=0.5)) == 0.0               # already beyond STOP_DISTANCE
  m = mpc()
  for _ in range(20):
    assert step(m, lead(v=0.5, status=False)) == 0.0


# 11-19 (stop control): the MPC's own stop target behind a stopped lead is STOP_GAP (it was 5.75 m: the crawl stops at ~6 m)
def smpc():
  return SimpleNamespace(dt=DT, stop_offset=0.0)


def shift(m, ld):
  return lm.LongitudinalMpc.stop_shift(m, ld)


def test_stopped_lead_moves_the_stop_target_to_the_stop_gap():
  m = smpc()
  for _ in range(60):
    out = shift(m, lead(d=20.0, v=0.0))
  assert abs(out - lm.STOP_SHIFT) < 1e-6
  assert abs(lm.STOP_DISTANCE + lm.STOP_EXTRA - lm.STOP_SHIFT - lm.STOP_GAP) < 1e-6


def test_stop_shift_is_rate_limited_and_gone_behind_a_moving_lead():
  m = smpc()
  assert 0.0 < shift(m, lead(v=0.0)) <= lm.STOP_SHIFT_RATE * DT + 1e-9
  for _ in range(60):
    shift(m, lead(v=0.0))
  for _ in range(60):
    out = shift(m, lead(v=lm.FOLLOW_VLEAD[1] + 1.0))
  assert out == 0.0
  assert shift(m, lead(status=False)) == 0.0
