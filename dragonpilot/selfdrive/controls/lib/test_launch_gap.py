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


# 11-18 owner: at crawl speed the MPC's own stop target is ~3.5 m (22:03: 6.0 / 5.9 m on the 5.75 m MPC target)
def cmpc():
  return SimpleNamespace(dt=DT, creep_w=0.0, creep_offset=0.0)


def creep(m, ld, v_ego):
  return lm.LongitudinalMpc.creep_gap(m, ld, v_ego)


def test_crawl_behind_a_queue_gets_the_creep_stop_target():
  m = cmpc()
  for _ in range(100):
    creep(m, lead(d=6.0, v=0.8), 0.8)                       # following a creeping lead at ~3 km/h
  assert abs(m.creep_offset - lm.CREEP_FLOOR) < 1e-9
  assert abs((lm.STOP_DISTANCE + lm.STOP_EXTRA - lm.CREEP_FLOOR) - lm.CREEP_GAP) < 1e-9
  for _ in range(100):
    creep(m, lead(d=5.0, v=0.0), 0.5)                       # the lead stopped, we brake to the stop: kept
  assert abs(m.creep_offset - lm.CREEP_FLOOR) < 1e-9
  o = m.creep_offset
  for _ in range(100):
    creep(m, lead(d=3.6, v=0.0), 0.0)                       # both stopped: held
  assert m.creep_offset == o


def test_normal_stop_from_speed_untouched():
  m = cmpc()
  v = 30 / 3.6
  while v > 0.0:
    assert creep(m, lead(d=4.0 + 2 * v, v=0.0), v) == 0.0   # the lead was stopped when we came below 10 km/h
    v -= 0.1


def test_faster_crawl_gets_less_and_lost_lead_none():
  m = cmpc()
  for _ in range(100):
    creep(m, lead(d=8.0, v=1.3), 1.3)                       # ~4.7 km/h
  assert 0.0 < m.creep_offset < lm.CREEP_FLOOR
  for _ in range(100):
    creep(m, lead(d=8.0, v=5.0), 5.0)                       # out of the queue: fades
  assert m.creep_offset == 0.0
  m = cmpc()
  creep(m, lead(d=6.0, v=0.8), 0.8)
  assert creep(m, lead(status=False), 0.8) == 0.0
