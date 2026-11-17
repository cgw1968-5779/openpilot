import numpy as np

from dragonpilot.selfdrive.controls.lib import low_speed_lat as L

KMH = 1 / 3.6


def test_kp_scaled_only_at_low_speed():
  speeds = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
  kps = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, 0.8]
  out = L.low_speed_kp(speeds, kps)
  assert out[:3] == [k * L.KP_SCALE for k in kps[:3]]       # <= 10 km/h
  assert out[6:] == kps[6:]                                  # >= 20 km/h: stock
  assert all(o <= k for o, k in zip(out, kps))


def test_curvature_smoothed_at_low_speed_stock_above():
  f = L.LowSpeedCurvature(0.01)
  rng = np.random.default_rng(0)
  noisy = 0.002 + 0.002 * rng.standard_normal(500)
  out = [f.update(k, 8 * KMH, True) for k in noisy]
  assert np.std(np.diff(out[100:])) < 0.2 * np.std(np.diff(noisy[100:]))   # jumps smoothed
  assert abs(np.mean(out[200:]) - 0.002) < 3e-4                             # the path (mean) kept
  f = L.LowSpeedCurvature(0.01)
  assert all(f.update(k, 30 * KMH, True) == k for k in noisy[:50])          # above 25 km/h: stock
  assert f.update(0.01, 8 * KMH, False) == 0.01                             # inactive: passthrough
