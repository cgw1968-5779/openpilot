"""
dragonpilot (dppb, Corolla Altis; owner 2026-11-17): calm the steering at very low speed without moving the model's path.

Owner: "at very low speed it still sways left and right a lot - can torque friction tuning or anything else that does
not change the model's path stop it?" -> "do A and B".
11-15 + 11-17 rlogs (lat active, no driver torque): steering rate RMS 28 / 42 / 52 / 41 / 16 / 7 deg/s at 1-5 / 5-10 /
10-15 / 15-25 / 25-40 / 40-60 km/h, the model's desired curvature jumping frame to frame 23 / 13 / 9 / 4 / 1.6 / 0.7e-4
1/m (rms), and the stock torque controller's proportional term RMS 6.5 / 5.6 / 3.3 lateral-accel units below 15 km/h
(KP_INTERP 250 at 1 m/s .. 2 at 15 m/s), i.e. the torque often clipped at full one way and then the other. Friction
compensation is small down here (the lateral-accel error is tiny at low v), so it is not the cause. dp's lane
centering is already off below 20-25 km/h.

A) the desired curvature handed to the lateral controller is low-passed (tau LP_TAU) below LP_V[0], faded out by LP_V[1]:
   the model's path is kept (the mean is unchanged, it lags by ~LP_TAU), only the frame-to-frame jumps are smoothed.
B) the torque controller's proportional gain is scaled by KP_SCALE at and below KP_V[0], back to stock at KP_V[1].
Low-speed closed-loop sim (fitted Altis steering plant, 169 low-speed stretches, 946 s): torque at its limit 7.8% ->
3.8% of the time, steering rate 12.3 -> 11.2 deg/s, path deviation from the 1 s mean path 0.671 -> 0.678 m rms (the sim
under-reproduces the recorded wobble, 18.5 deg/s, so the road is the judge).
"""
import numpy as np

LP_TAU = 0.35                  # s
LP_V = [15.0 / 3.6, 25.0 / 3.6]  # m/s: full smoothing below the first, none above the second
KP_SCALE = 0.4
KP_V = [10.0 / 3.6, 20.0 / 3.6]  # m/s


def low_speed_kp(speeds, kps):
  """B: the torque controller's KP table with the low-speed points scaled (speeds in m/s)."""
  return [float(k * np.interp(v, KP_V, [KP_SCALE, 1.0])) for v, k in zip(speeds, kps)]


class LowSpeedCurvature:
  """A: low-pass of the desired curvature at very low speed."""
  def __init__(self, dt: float):
    self.dt = dt
    self.k = 0.0

  def update(self, desired_curvature: float, v_ego: float, active: bool) -> float:
    if not active:
      self.k = desired_curvature
      return desired_curvature
    self.k += (desired_curvature - self.k) * self.dt / (LP_TAU + self.dt)
    w = float(np.interp(v_ego, LP_V, [1.0, 0.0]))
    if w <= 0.0:
      self.k = desired_curvature  # above the fade: stock, and the filter restarts from here
      return desired_curvature
    return w * self.k + (1.0 - w) * desired_curvature
