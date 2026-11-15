"""
dragonpilot (dppb, Corolla Altis): a very small relief when the Altis brakes harder than asked at 28-12 km/h.

Owner 2026-11-08: "normal mid/end braking on dppb is good; only when a lead slows from about 40-30 km/h, the brake at
around 20-15 km/h is a little too heavy. Without touching normal mid/end braking, make that slightly heavy brake just
a little lighter - almost the smallest step."

dppb rlogs 11-04..11-08 (engaged, moving lead slowing): at 25-16 km/h the car delivers 0.4-0.7 m/s^2 MORE than the
command (11-04 13:19:32 cmd -1.88 / aEgo -2.47, 13:15:07 -2.18 / -2.72); the TSS2 PID only integrates (ki 0.25 above
18 km/h), too slowly to catch it. Normal stops overshoot <= 0.3 there or not at all.

So the existing send-side loop gets a small proportional term with no memory (gone the moment the overshoot is):
relief = KP x (overshoot - DEADBAND), only while braking firmer than CMD_MAX, faded in 12-15 km/h and out 22-28 km/h.
Overshoot = command - filtered aEgo (the controller's own aEgo filter). It only ever makes the command lighter by at
most MAX_RELIEF, and only while the car is ALREADY decelerating more than the planner asked (by > DEADBAND), so the
delivered decel stays at least the planner's. Replay over 11 dppb drives: 7 events, max 0.16 (13:12 / 13:15 / 13:19
/ 13:24 0.10-0.16); normal stops 0-0.04.

Owner 2026-11-08 after driving it: "20-15 a little lighter worked, almost perfect - try one more tiny step lighter".
KP 0.4 -> 0.5 (deadband, window, cap and rate unchanged). Same replay: 13:12:43 0.16 -> 0.20 (the cap), 13:15:07 0.10 ->
0.12, 13:19:32 0.11 -> 0.14, 13:24:42 0.12 -> 0.15, 13:26:13 0.08 -> 0.10; normal stops 11-05 09:22:55 / 11-08 08:32:50
stay 0.04, the rest 0.01-0.06. relief = 0.5 x (overshoot - 0.3) is always smaller than the overshoot, so the car still
decelerates at least as much as the planner asked.

Owner 2026-11-15 (11:26 bookmark): "after a stop, creeping up with the gas to adjust the gap: when the gas is released
the stop is still too heavy - it should be like the smooth 11:27 three-bookmark stop". 34 low-speed gas releases
(Oct-Nov rlogs): the command we send is ~0..-0.5, but about 0.25 s after the release the PCM's own net request
(PCM_CRUISE ACCEL_NET) sits a roughly constant amount below it - mostly -0.5..-1.3 behind a stopped close lead (11:26:13
sent -0.08, ACCEL_NET -1.36, aEgo -1.55), ~0 or even positive with a moving / no lead - and stays there until the car
stops. Within one release it adds to whatever we send (11:26:19: command -0.11..-0.52, extra steady ~-1.0).
GasReleaseRelief (same idea as the relief above - the car brakes more than asked - extended to this case): after the
driver releases the gas below GR_V_MAX, until standstill, the extra the PCM adds (ACCEL_NET - what we sent) is sent back
on top, so the PCM delivers what the planner / stop end layers ask for, as in a stop without the gas. Never more than
GR_MAX, up at most GR_UP; while the command asks for more than GR_URGENT (planner: short stop) it is withdrawn at
GR_URGENT_DOWN; at standstill at GR_STILL_DOWN (the hold is the PCM's again). A positive extra is never followed.
Replay of the 34 releases: 11:26:13 aEgo -1.55 -> -0.45, 11:26:19 -1.51 -> -0.86; releases with a moving lead or none
(extra >= 0) unchanged. Not active in any stop without the gas.
"""
import numpy as np

KP = 0.5           # 2026-11-08: was 0.4 (owner: one more tiny step lighter at 20-15 km/h)
DEADBAND = 0.3      # m/s^2: overshoot below this is untouched (normal stops)
CMD_MAX = -0.5      # m/s^2: only while the command brakes firmer than this
MAX_RELIEF = 0.2    # m/s^2
RATE = 0.5          # m/s^3: the relief itself moves at most this fast (aEgo noise must not be felt)
V_BP = [12. / 3.6, 15. / 3.6, 22. / 3.6, 28. / 3.6]   # m/s
V_W = [0.0, 1.0, 1.0, 0.0]


def relief_target(v_ego: float, accel_cmd: float, a_ego_filtered: float) -> float:
  if accel_cmd > CMD_MAX:
    return 0.0
  w = float(np.interp(v_ego, V_BP, V_W))
  if w == 0.0:
    return 0.0
  over = accel_cmd - a_ego_filtered   # > 0: the car decelerates more than asked
  return float(np.clip(KP * (over - DEADBAND) * w, 0.0, MAX_RELIEF))


class BrakeOvershootRelief:
  def __init__(self, dt: float):
    self.step = RATE * dt
    self.relief = 0.0

  def update(self, active: bool, v_ego: float, accel_cmd: float, a_ego_filtered: float) -> float:
    target = relief_target(v_ego, accel_cmd, a_ego_filtered) if active else 0.0
    if not active:
      self.relief = 0.0
    self.relief = float(np.clip(target, self.relief - self.step, self.relief + self.step))
    return self.relief


GR_V_MAX = 10. / 3.6   # m/s
GR_DB = 0.1            # m/s^2: extra below this is left alone
GR_MAX = 1.5           # m/s^2
GR_UP = 6.0            # m/s^3
GR_DOWN = 1.0          # m/s^3
GR_URGENT = -1.0       # m/s^2
GR_URGENT_DOWN = 2.0   # m/s^3
GR_STILL_DOWN = 3.0    # m/s^3


class GasReleaseRelief:
  def __init__(self, dt: float):
    self.dt = dt
    self.relief = 0.0
    self.armed = False  # the gas was pressed (engaged, slow) and this release has not reached standstill yet

  def update(self, active: bool, gas: bool, standstill: bool, v_ego: float, accel_cmd: float, sent_prev: float,
             accel_net: float) -> float:
    if gas:
      self.armed = v_ego < GR_V_MAX
      self.relief = 0.0
      return 0.0
    if not active or v_ego >= GR_V_MAX:
      self.armed = False
    if not self.armed:
      self.relief = 0.0
      return 0.0
    if standstill:
      target, down = 0.0, GR_STILL_DOWN
    elif accel_cmd < GR_URGENT:
      target, down = 0.0, GR_URGENT_DOWN
    else:
      extra = accel_net - sent_prev  # what the PCM adds beyond what we sent (relief included)
      target, down = float(np.clip(-extra - GR_DB, 0.0, GR_MAX)), GR_DOWN
    self.relief = float(np.clip(target, self.relief - down * self.dt, self.relief + GR_UP * self.dt))
    if standstill and self.relief == 0.0:
      self.armed = False
    return self.relief
