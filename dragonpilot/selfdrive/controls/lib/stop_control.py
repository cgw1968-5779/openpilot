"""
dragonpilot (dppb, Corolla Altis Hybrid): StopControl - one controller owns the stop behind a stopped lead.

Owner 2026-11-18/19 (hard targets): every stop ends STOP_GAP (3.5 m, radar dRel) behind the lead within +-0.25 m, no
jolt at the instant of the stop, no two-stage stop (stopped, then moving on) unless the lead itself moved, and the lead
is followed whenever it moves. "Remove the cause at the root, do not patch."

What the stop scorecard showed (152 dppb stops behind a stopped lead, rlogs 11-02..11-18, /mnt/e/Claude/claude_work/
stopdb): 27 % ended within 3.25-3.75 m (range 2.4-6.0 m; crawl stops ~6 m), 5 % rolled on after the stop, the felt
decel just before the true stop was -0.35 (median), and only 4 % met all three targets. Causes, all measured there:
 1. nothing closed the loop on the remaining distance: the MPC aimed at 5.75 m and six open-loop end layers (soft end,
    stop end level, end hold, engage hold, creep gap, final ease) let the car coast past it; where it ended depended on
    the approach speed, the braking level and the car's response (real / commanded decel 0.75-0.98 at 6-12 km/h,
    0.77-1.29 at 1.6-6 km/h, 0.31-1.30 below 1.6 km/h from stop to stop);
 2. below ~8 km/h the car answers a command late: 0.33 s at 3-8 km/h, 0.53 s (0.30-0.70) below 3 km/h (pure delay).
    What is felt in the last half second was commanded at 1.5-2 km/h: easing "in the last instant" cannot work;
 3. the wheel speeds read 0 below ~0.6 km/h, 0.4-0.5 s (0.15-0.9 s) before the car really stops: the hold was firmed
    at that flag while the car still rolled (jolt), or eased blind (roll-on);
 4. the PCM's own request (ACCEL_NET) is not the felt decel (it reads +0.65 at a zero command at 6-12 km/h);
 5. release overshoot: a brake command eased quickly at low speed leaves the brake under-delivering, and below ~1.6
    km/h the car then does not slow at all - it creeps on at ~1 km/h for 2-4 s, about 1 m (every "too near" stop:
    18 of 151, e.g. command -1.06 at 3 km/h -> -0.40 at 1.6 km/h, real decel 0.06). Fitted on the 151 stops: it
    happens where command - 0.3 x (what it was eased by in the last 1 s) < 0.10 (15 of 18 caught, 3 false alarms).
    Slow easing to a light command is fine (-0.55 -> -0.30 over a second: real / commanded ~1.0).

What it does (replaces those layers for a stop behind a slow or stopped radar lead):
 - the stop point is STOP_GAP behind where the lead comes to rest (a lead still rolling slower than ST_LEAD_SLOW is
   taken to brake at ST_LEAD_DECEL from now: a queue creeping on a few metres is followed and stopped behind in one
   go). Remaining distance s from a complementary filter (odometry + radar, ST_RADAR_TC); speed = wheel speed, below
   the wheel-speed floor the IMU-integrated speed (bias learned against the wheel acceleration);
 - the state is predicted to when a command sent now is felt (ST_DELAY by speed), by running the commands already
   sent but not yet felt through the nominal car (x this stop's gain k);
 - the decel profile into the point, by speed: the last ST_V_END is a constant light decel ST_A_END; above it the
   decel eases into that at a constant slow rate ST_JERK_END (a^2 = a_end^2 + 2 j (v - v_end): no release overshoot,
   and nothing changes fast where the car answers half a second late) from a mid decel a_mid; a_mid is solved every
   cycle so that the profile ends exactly at the point from the predicted (s, v) - a car that brakes more or less
   than commanded is corrected by the loop while there is still speed to do it. Where the easing part alone is too
   long the rate goes up to ST_JERK_END_MAX; in a late stop beyond that, up to ST_JERK_HARD with a firmer end
   (ST_HARD_END: a fast easing must not end light, see 5);
 - command = profile decel / (nominal gain at this speed x k), k = felt (IMU) / expected decel of this stop; changed
   at most ST_JERK_UP firmer, eased at most ST_EASE below 5 km/h; below ST_V_LOW never less brake than ST_CMD_MIN;
 - past the point (the brake under-delivered) the decel is firmed by how far past it really is (ST_PAST_GAIN beyond
   ST_PAST_FREE), at any speed: the car must stop;
 - closing up: farther than ST_CRAWL_S from the point and slower than the speed the lightest profile allows there
   (at most the crawl speed), the brake is released / light gas (also how it follows a lead that rolled on a little);
 - the hold starts ST_HOLD_DELAY after the estimated speed reaches zero (not at the wheel-speed flag): the command
   firms at ST_HOLD_RATE to ST_HOLD and longcontrol's own standstill ramp carries on; latched until the lead moves
   (or has rolled on more than ST_HOP_MOVED, leaving more than ST_HOP_GAP), the car is driven off, or the gas is pressed.
Active behind a lead slower than ST_LEAD_SLOW within ST_D_ON, below ST_V_ON, from when the lightest profile no longer
fits (or, behind a stopped lead, once the planner brakes ST_A_ON, and always within ST_S_OWN below ST_V_OWN). The
planner (whose own stop target is the same STOP_GAP) drives until then, and again when the lead moves off or there is
room again behind a moving lead. A lost lead is bridged for ST_LOST_T on odometry. The creep stop (after the driver
crept up with the gas) keeps its own layer and overrides this one.

Closed-loop campaign (x86 MPC + this controller + the measured car incl. the release overshoot, 133 recorded stop
starts, car response drawn at random per stop): 97 % end at 3.25-3.75 m (dppb before: 37 %), 65 % also with a felt
decel <= 0.30 at the stop and no roll-on (before: 13 %; 88 % at <= 0.36). Scenarios (lead braking to a stop, queue
creeping 1 / 3 / 8 m, lead rolling on 0.5 m, drive-off, no IMU): 78 stops, 97 % in range, drive-off follows in 1.4 s.
"""
import math

import numpy as np

STOP_GAP = 3.5            # m (owner): radar dRel at rest
ST_AIM = 0.05             # m: aim offset of the stop point (calibrated in the closed-loop campaign)
ST_JERK_END = 0.25        # m/s^3: the decel eases into ST_A_END at this constant rate (see "release overshoot" above) ...
ST_JERK_END_MAX = 0.40    # m/s^3: ... up to this with the same light end when the point would otherwise be passed ...
ST_JERK_HARD = 1.2        # m/s^3: ... and up to this in a late stop, with the firmer end of ST_HARD_END
ST_HARD_END = (0.15, 0.30)   # m/s^2, s: end decel >= [0] + [1] x easing rate (stopdb: stalls where cmd - 0.3 x drop in 1 s < 0.1)
ST_A_END = 0.27           # m/s^2: felt decel of the last ST_V_END, down to the stop
ST_A_END_RANGE = (0.25, 0.285)  # m/s^2: in the last ST_V_END the decel is trimmed by the remaining distance within this
ST_CMD_MIN = 0.25         # m/s^2: below ST_V_LOW a braking command is never lighter than this (lighter ones let the car creep on)
ST_V_LOW = 3.0 / 3.6      # m/s
ST_EASE_V = [5.0 / 3.6, 8.0 / 3.6]   # m/s
ST_EASE = [0.5, 3.0]      # m/s^3: fastest easing of a braking command by speed (ST_JERK_DOWN above)
ST_V_END = 0.8 / 3.6      # m/s
ST_A_MAX = 3.0            # m/s^2: most it asks (a firmer planner brake is passed through)
ST_PAST_GAIN = 1.5        # m/s^2 per m past the stop point ...
ST_PAST_FREE = 0.10       # m: ... beyond this
ST_RADAR_TC = 0.5         # s: complementary filter, radar corrects the odometry-propagated distance
ST_DELAY_V = [2.0 / 3.6, 5.0 / 3.6, 8.0 / 3.6, 12.0 / 3.6]   # m/s
ST_DELAY_T = [0.50, 0.38, 0.25, 0.12]                        # s: command -> felt decel (stopdb)
ST_PIPE_T = 0.70          # s of sent commands kept
ST_V_ON = 25.0 / 3.6      # m/s
ST_LEAD_V = 0.5           # m/s: the lead counts as stopped below this ...
ST_LEAD_T = 0.3           # s: ... for this long
ST_LEAD_SLOW = 2.5        # m/s: a lead slower than this (for ST_LEAD_T) may be about to stop: its stop point is aimed at
ST_LEAD_DECEL = 1.0       # m/s^2: the decel a rolling lead is taken to stop with
ST_GO_MARGIN = 0.5        # m of spare room at which a moving lead is handed back to the planner
ST_LEAD_GO_V = 0.35       # m/s: the lead moves again above this ...
ST_LEAD_GO_T = 0.15       # s: ... for this long (launch_gap's numbers)
ST_D_ON = 45.0            # m
ST_A_ON = 0.25            # m/s^2
ST_S_OWN = 6.0            # m: within this of the stop point and below ST_V_OWN the crawl up to it is also owned
ST_V_OWN = 10.0 / 3.6     # m/s
ST_V_CRAWL = 3.0 / 3.6    # m/s: crawl speed while closing up
ST_CRAWL_S = 0.25         # m: closing up only while farther than this from the point
ST_CRAWL_K = 1.0          # 1/s: accel per m/s below the crawl speed ...
ST_CRAWL_A = 0.25         # m/s^2: ... at most this (a brake release / light gas)
ST_JERK_UP = 2.5          # m/s^3 firmer
ST_JERK_DOWN = 3.0        # m/s^3 lighter
ST_V_FLOOR = 0.7 / 3.6    # m/s: the wheel speeds are not trusted below this
ST_V_REST = 0.03          # m/s: the estimated speed counts as at rest below this
ST_HOLD_DELAY = 0.10      # s after the estimated stop before the hold firms
ST_HOLD = -0.55           # m/s^2: command once at rest (then longcontrol's standstill ramp)
ST_HOLD_RATE = 3.0        # m/s^3
ST_STOP_T_MAX = 1.5       # s after the wheel-speed flag: at rest at the latest
ST_LOST_T = 0.6           # s a lost lead is bridged
ST_HOP_MOVED = 0.20       # m the lead must have moved on since we came to rest ...
ST_HOP_GAP = 0.25         # m ... leaving us more than this beyond STOP_GAP ...
ST_HOP_T = 0.5            # s ... for this long: hand back so the car closes up (the owner allows a move after the lead moved)
ST_BIAS_TC = 3.0          # s: IMU accel bias against the wheel acceleration (learned at ST_BIAS_V)
ST_BIAS_V = (1.5, 12.0)   # m/s
# nominal real / commanded decel by speed (stopdb, from the wheel speeds: the IMU reads the pitch rebound as decel and
# overstated the low-speed spread). From stop to stop (10-90 %): 0.31-1.30 at 0.8-1.6 km/h (25-75 %: 0.81-1.17),
# 0.77-1.29 at 1.6-3, 0.78-1.13 at 3-6, 0.75-0.98 at 6-12 km/h
ST_GAIN_V = [1.2 / 3.6, 2.3 / 3.6, 4.5 / 3.6, 9.0 / 3.6, 18.0 / 3.6]
ST_GAIN = [1.05, 1.01, 0.96, 0.83, 0.95]
ST_K_TC = 0.50            # s: filter of k = felt decel (IMU) / what the command sent ST_DELAY ago gives on the nominal car
ST_K_RANGE = (0.75, 1.2)   # limits of k
ST_K_MIN_EXP = 0.25       # m/s^2: k is only updated while the expected decel is at least this ...
ST_K_V_MIN = 1.5 / 3.6    # m/s: ... and above this speed (the command changes slowly down there now, the delay matters little)


def _rise(v: float, jerk: float, a_end: float) -> float:
  return math.sqrt(a_end * a_end + 2.0 * jerk * max(v - ST_V_END, 0.0))


def profile_decel(v: float, a_mid: float, jerk: float = ST_JERK_END, a_end: float = ST_A_END) -> float:
  """felt decel of the stop profile at speed v: a_end up to ST_V_END, above it the decel that eases into a_end at the
  constant rate jerk (a^2 = a_end^2 + 2 jerk (v - ST_V_END)), at most a_mid"""
  return min(_rise(v, jerk, a_end), max(a_mid, a_end))


def profile_distance(v: float, a_mid: float, jerk: float = ST_JERK_END, a_end: float = ST_A_END) -> float:
  """distance the profile needs from speed v to rest"""
  if v <= 0.0:
    return 0.0
  a_mid = max(a_mid, a_end)
  v1 = ST_V_END
  if v <= v1:
    return v * v / (2.0 * a_end)
  d = v1 * v1 / (2.0 * a_end)
  a_top = min(_rise(v, jerk, a_end), a_mid)
  d += ((v1 - a_end * a_end / (2.0 * jerk)) * (a_top - a_end) + (a_top ** 3 - a_end ** 3) / (6.0 * jerk)) / jerk
  v_m = v1 + (a_mid * a_mid - a_end * a_end) / (2.0 * jerk)      # where the easing part starts
  if v > v_m:
    d += (v * v - v_m * v_m) / (2.0 * a_mid)
  return d


def hard_end(jerk: float) -> float:
  """the end decel that goes with an easing rate: a brake eased faster under-delivers more (release overshoot), so a
  fast easing has to end firmer or the car creeps on"""
  return max(ST_A_END, ST_HARD_END[0] + ST_HARD_END[1] * jerk)


def solve_decel(s: float, v: float) -> tuple[float, float, float]:
  """(decel now, easing rate, end decel) of the profile that ends exactly s ahead from speed v. The mid decel is
  solved first (easing at ST_JERK_END); where the easing part alone is already too long the easing rate is raised, up
  to ST_JERK_END_MAX with the same light end, beyond it (a late stop) up to ST_JERK_HARD with a firmer end"""
  if profile_distance(v, ST_A_END) <= s:
    return ST_A_END, ST_JERK_END, ST_A_END
  if profile_distance(v, ST_A_MAX) <= s:
    lo, hi = ST_A_END, ST_A_MAX
    for _ in range(14):
      mid = 0.5 * (lo + hi)
      if profile_distance(v, mid) > s:
        lo = mid
      else:
        hi = mid
    return profile_decel(v, hi), ST_JERK_END, ST_A_END
  def dist(j):
    return profile_distance(v, ST_A_MAX, j, hard_end(j))

  if dist(ST_JERK_HARD) >= s:
    j = ST_JERK_HARD
  else:
    lo, hi = ST_JERK_END, ST_JERK_HARD
    for _ in range(12):
      mid = 0.5 * (lo + hi)
      if dist(mid) > s:
        lo = mid
      else:
        hi = mid
    j = hi
  return profile_decel(v, ST_A_MAX, j, hard_end(j)), j, hard_end(j)


def profile_speed(s: float) -> float:
  """the speed (at most the crawl speed) from which the lightest profile ends exactly s ahead"""
  if s <= 0.0:
    return 0.0
  if profile_distance(ST_V_CRAWL, ST_A_END) <= s:
    return ST_V_CRAWL
  lo, hi = 0.0, ST_V_CRAWL
  for _ in range(12):
    mid = 0.5 * (lo + hi)
    if profile_distance(mid, ST_A_END) > s:
      hi = mid
    else:
      lo = mid
  return lo


class StopControl:
  def __init__(self, dt: float):
    self.dt = dt
    self.active = False
    self.held = False        # at rest behind the lead, latched
    self.cmd = 0.0
    self.s = 0.0             # filtered remaining distance to the stop point
    self.have_s = False
    self.lead_t = 0.0        # s the lead has been stopped
    self.slow_t = 0.0        # s the lead has been slow
    self.go_t = 0.0          # s the lead has been moving again
    self.lost_t = 0.0
    self.v_est = 0.0
    self.a_last = -0.2
    self.bias = 0.0          # IMU accel bias
    self.have_bias = False
    self.flag_t = 0.0        # s since the wheel-speed standstill flag in this stop
    self.rest_t = 0.0
    self.hop_t = 0.0
    self.gap_at_rest = float('nan')   # radar gap when we came to rest
    self.k = 1.0             # felt / nominal decel of this stop
    self.a_end = ST_A_END    # end decel of the profile in use
    self.pipe = [0.0] * int(round(ST_PIPE_T / dt))   # the commands of the last ST_PIPE_T (newest last)

  def reset(self):
    self.active = self.held = False
    self.have_s = False
    self.flag_t = self.hop_t = self.rest_t = 0.0

  def override(self, cmd: float):
    """another layer has the stop (the creep stop): stand down, keep the record of what was really sent"""
    self.reset()
    self.pipe[-1] = self.cmd = cmd

  def _speed(self, CS, imu_ax):
    """wheel speed above the floor; below it the IMU-integrated speed (the last wheel decel when there is no IMU)"""
    v = CS.vEgo
    if imu_ax is not None and ST_BIAS_V[0] < v < ST_BIAS_V[1]:
      e = imu_ax - CS.aEgo
      self.bias = e if not self.have_bias else self.bias + (e - self.bias) * self.dt / ST_BIAS_TC
      self.have_bias = True
    if v > ST_V_FLOOR and not CS.standstill:
      self.v_est = v
      self.a_last = min(CS.aEgo, 0.0)
      self.flag_t = 0.0
    else:
      a = (imu_ax - self.bias) if (imu_ax is not None and self.have_bias) else self.a_last
      self.v_est = max(self.v_est + min(a, 0.0) * self.dt, 0.0)
      if CS.standstill:
        self.flag_t += self.dt
        if self.flag_t >= ST_STOP_T_MAX:
          self.v_est = 0.0
    return self.v_est

  def _predict(self, v: float):
    """(s, v) when a command sent now is felt: the commands still on their way run through the nominal car x k"""
    n = min(int(round(float(np.interp(v, ST_DELAY_V, ST_DELAY_T)) / self.dt)), len(self.pipe))
    s_p, v_p = self.s, v
    for c in self.pipe[len(self.pipe) - n:] if n else ():
      a = c * (self._gain(v_p) if c < 0.0 else 1.0)
      v_n = max(v_p + a * self.dt, 0.0)
      s_p -= 0.5 * (v_p + v_n) * self.dt
      v_p = v_n
    return s_p, v_p

  def _gain(self, v: float) -> float:
    """felt / commanded decel at speed v: the nominal car x this stop's k """
    return float(np.interp(v, ST_GAIN_V, ST_GAIN)) * self.k

  def _send(self, cmd: float) -> float:
    self.cmd = cmd
    self.pipe.append(cmd)
    self.pipe.pop(0)
    return cmd

  def update(self, output_accel: float, a_target: float, CS, lead, active: bool, imu_ax=None) -> float:
    v = self._speed(CS, imu_ax)
    has_lead = lead is not None and lead.status
    if has_lead:
      self.lost_t = 0.0
      self.lead_t = self.lead_t + self.dt if abs(lead.vLead) < ST_LEAD_V else 0.0
      self.slow_t = self.slow_t + self.dt if lead.vLead < ST_LEAD_SLOW else 0.0
      self.go_t = self.go_t + self.dt if lead.vLead > ST_LEAD_GO_V else 0.0
      # the stop point is STOP_GAP behind where the lead comes to rest: a lead still rolling is taken to brake at
      # ST_LEAD_DECEL from now (a queue creeping on a few metres; a lead that keeps going moves the point on with it)
      s_meas = lead.dRel - STOP_GAP - ST_AIM + max(lead.vLead, 0.0) ** 2 / (2.0 * ST_LEAD_DECEL)
      if not self.have_s:
        self.s, self.have_s = s_meas, True
      else:
        self.s += -v * self.dt + (s_meas - (self.s - v * self.dt)) * self.dt / ST_RADAR_TC
    else:
      self.lost_t += self.dt
      self.lead_t = self.slow_t = 0.0
      if self.have_s:
        self.s -= v * self.dt
      if self.lost_t > ST_LOST_T:
        self.have_s = False

    gas = getattr(CS, 'gasPressed', False)
    going = has_lead and self.go_t >= ST_LEAD_GO_T
    if not active or gas or not self.have_s or (going and self.held):
      self.reset()
      return self._send(output_accel)

    if self.held:
      if not CS.standstill and v > ST_V_FLOOR:
        self.reset()               # the car is driven off: hand back
        return self._send(output_accel)
      # the lead rolled on after our stop (too slowly for its speed to show): let the car close up again
      moved = has_lead and lead.dRel - self.gap_at_rest > ST_HOP_MOVED and lead.dRel > STOP_GAP + ST_HOP_GAP
      self.hop_t = self.hop_t + self.dt if moved else 0.0
      if self.hop_t >= ST_HOP_T:
        self.reset()
        return self._send(output_accel)
      # at rest: never lighter than the hold; longcontrol's own standstill ramp goes on below it
      return self._send(min(output_accel, max(ST_HOLD, self.cmd - ST_HOLD_RATE * self.dt)))

    # this stop's plant gain: what the car did (IMU) against what the command sent one delay ago gives on the nominal car
    delay_n = min(int(round(float(np.interp(v, ST_DELAY_V, ST_DELAY_T)) / self.dt)), len(self.pipe))
    if self.active and imu_ax is not None and self.have_bias and v > ST_K_V_MIN and delay_n > 0:
      expected = max(-self.pipe[-delay_n], 0.0) * float(np.interp(v, ST_GAIN_V, ST_GAIN))
      if expected >= ST_K_MIN_EXP:
        felt = max(-(imu_ax - self.bias), 0.0)
        self.k += (float(np.clip(felt / expected, *ST_K_RANGE)) - self.k) * self.dt / ST_K_TC

    s_p, v_p = self._predict(v)
    need, jerk, a_end = solve_decel(s_p, v_p)
    if v_p > ST_V_END:
      self.a_end = a_end           # the end level this stop is heading for (kept through the last ST_V_END)

    if self.active and going and need <= ST_A_END and jerk <= ST_JERK_END and s_p > profile_distance(v_p, ST_A_END) + ST_GO_MARGIN:
      self.reset()                 # the lead moves on and there is room again: the planner follows it
      return self._send(output_accel)

    if not self.active:
      near = has_lead and self.slow_t >= ST_LEAD_T and v < ST_V_ON and lead.dRel < ST_D_ON
      own = self.lead_t >= ST_LEAD_T and self.s < ST_S_OWN and v < ST_V_OWN
      tight = s_p < profile_distance(v_p, ST_A_END)      # the lightest profile no longer fits: time to brake
      if not near or v <= ST_V_REST or not (tight or (not going and (a_target <= -ST_A_ON or own))):
        return self._send(output_accel)
      self.active, self.rest_t, self.k, self.a_end = True, 0.0, 1.0, ST_A_END
      self.cmd = output_accel

    if v <= ST_V_REST:
      # the estimated speed reached zero: at rest - hold ST_HOLD_DELAY later (the estimate runs a little ahead of the
      # car; a hold that reaches the car before it has stopped is the jolt)
      self.rest_t += self.dt
      if self.rest_t < ST_HOLD_DELAY:
        return self._send(self.cmd)
      self.held = True
      self.gap_at_rest = lead.dRel if has_lead else float('nan')
      return self._send(min(output_accel, max(ST_HOLD, min(self.cmd, 0.0) - ST_HOLD_RATE * self.dt)))

    lo = self.cmd - ST_JERK_UP * self.dt
    hi = self.cmd + ST_JERK_DOWN * self.dt
    ease = max(float(np.interp(v, ST_EASE_V, ST_EASE)), 1.3 * jerk)
    hi_brake = self.cmd + min(ST_JERK_DOWN, ease) * self.dt if self.cmd < 0.0 else hi
    # closing up: still far from the point and slower than the crawl speed -> release the brake / light gas
    v_ref = profile_speed(s_p)
    if s_p > ST_CRAWL_S and v_p < v_ref - 0.05:
      return self._send(float(np.clip(min(ST_CRAWL_K * (v_ref - v_p), ST_CRAWL_A), lo, hi)))

    if v_p <= ST_V_END:
      # the last ST_V_END: the decel that ends at the point, kept inside the no-jolt band (the car's response below
      # 1.6 km/h is 0.31-1.30 x the command from stop to stop); past the point a little more, never a "must stop" bite
      need = float(np.clip(v_p * v_p / (2.0 * max(s_p, 0.01)), ST_A_END_RANGE[0], max(ST_A_END_RANGE[1], self.a_end)))
    # past the point (the brake under-delivered: the car creeps on): firmer by how far it really is past, at any speed -
    # nothing else would stop it short of the lead. A few cm past changes nothing (no last-instant correction)
    need = min(need + ST_PAST_GAIN * max(-s_p - ST_PAST_FREE, 0.0), ST_A_MAX)
    goal = -need / self._gain(v_p)
    if v_p < ST_V_LOW:
      goal = min(goal, -ST_CMD_MIN / self._gain(v_p))            # never less brake delivered than what holds the creep
    if a_target < -ST_A_MAX:
      goal = min(goal, a_target)                                   # an urgent planner brake is never lightened
    return self._send(float(np.clip(goal, lo, max(hi_brake, lo))))
