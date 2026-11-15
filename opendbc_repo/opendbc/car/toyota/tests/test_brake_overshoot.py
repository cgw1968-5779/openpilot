from opendbc.car.toyota import brake_overshoot as B

DT = 0.03


def test_no_relief_without_overshoot_or_outside_the_window():
  r = B.BrakeOvershootRelief(DT)
  for _ in range(100):
    assert r.update(True, 18 / 3.6, -1.5, -1.75) == 0.0     # overshoot 0.25 < deadband
    assert r.update(True, 35 / 3.6, -2.0, -2.8) == 0.0      # above 28 km/h
    assert r.update(True, 10 / 3.6, -2.0, -2.8) == 0.0      # below 12 km/h
    assert r.update(True, 18 / 3.6, -0.3, -1.0) == 0.0      # light command
    assert r.update(True, 18 / 3.6, -1.5, -1.2) == 0.0      # car brakes less than asked: never firmer


def test_small_relief_for_a_heavy_overshoot_rate_limited_and_capped():
  r = B.BrakeOvershootRelief(DT)
  out = [r.update(True, 18 / 3.6, -1.9, -2.5) for _ in range(100)]
  assert abs(out[-1] - B.KP * (0.6 - 0.3)) < 1e-6
  assert out[-1] < 0.6 - 0.3                                   # still at least the planner's decel
  assert max(abs(b - a) for a, b in zip([0.0] + out, out)) <= B.RATE * DT + 1e-9
  out = [r.update(True, 18 / 3.6, -2.0, -4.0) for _ in range(100)]
  assert out[-1] == B.MAX_RELIEF


def test_gone_when_the_overshoot_is_and_reset_when_inactive():
  r = B.BrakeOvershootRelief(DT)
  for _ in range(100):
    r.update(True, 18 / 3.6, -1.9, -2.5)
  for _ in range(20):
    x = r.update(True, 18 / 3.6, -1.9, -1.9)
  assert x == 0.0
  r.update(True, 18 / 3.6, -1.9, -2.5)
  assert r.update(False, 18 / 3.6, -1.9, -2.5) == 0.0


# GasReleaseRelief (owner 2026-11-15: creeping with the gas, the stop on release is too heavy)
def pcm(r, base, extra, gas=False, standstill=False, v=0.6, n=1, active=True):
  # the PCM adds `extra` to whatever we send; returns the relief of each call
  out = []
  for _ in range(n):
    sent = base + r.relief
    out.append(r.update(active, gas, standstill, v, base, sent, sent + extra))
  return out


def test_gas_release_cancels_the_pcm_extra_brake():
  r = B.GasReleaseRelief(DT)
  pcm(r, 0.0, 0.0, gas=True)
  out = pcm(r, -0.1, -1.3, n=50)
  assert abs(out[-1] - (1.3 - B.GR_DB)) < 1e-6
  assert max(b - a for a, b in zip([0.0] + out, out)) <= B.GR_UP * DT + 1e-9


def test_no_relief_without_a_gas_release_or_for_a_positive_extra():
  r = B.GasReleaseRelief(DT)
  assert max(pcm(r, -0.3, -1.3, n=50)) == 0.0                 # a stop without the gas: untouched
  pcm(r, 0.0, 0.0, gas=True)
  assert max(pcm(r, -0.1, +1.0, n=50)) == 0.0                 # the PCM pushes: never followed


def test_capped_and_not_above_10_kph():
  r = B.GasReleaseRelief(DT)
  pcm(r, 0.0, 0.0, gas=True)
  assert pcm(r, -0.1, -3.0, n=100)[-1] == B.GR_MAX
  r = B.GasReleaseRelief(DT)
  pcm(r, 0.0, 0.0, gas=True, v=12 / 3.6)
  assert max(pcm(r, -0.1, -1.3, n=50, v=12 / 3.6)) == 0.0


def test_withdrawn_smoothly_for_an_urgent_command_and_at_standstill():
  r = B.GasReleaseRelief(DT)
  pcm(r, 0.0, 0.0, gas=True)
  pcm(r, -0.1, -1.3, n=50)
  out = pcm(r, -1.5, -1.3, n=100)
  assert out[-1] == 0.0 and max(a - b for a, b in zip([1.2] + out, out)) <= B.GR_URGENT_DOWN * DT + 1e-9
  r = B.GasReleaseRelief(DT)
  pcm(r, 0.0, 0.0, gas=True)
  pcm(r, -0.1, -1.3, n=50)
  out = pcm(r, -0.1, -1.3, n=100, standstill=True)
  assert out[-1] == 0.0 and not r.armed
  assert max(a - b for a, b in zip([1.2] + out, out)) <= B.GR_STILL_DOWN * DT + 1e-9
  assert max(pcm(r, -0.1, -1.3, n=20)) == 0.0                 # disarmed until the gas is pressed again


def test_disengaged_resets():
  r = B.GasReleaseRelief(DT)
  pcm(r, 0.0, 0.0, gas=True)
  pcm(r, -0.1, -1.3, n=50)
  assert pcm(r, -0.1, -1.3, active=False)[-1] == 0.0
