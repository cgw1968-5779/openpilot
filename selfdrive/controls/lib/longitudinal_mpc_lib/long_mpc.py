#!/usr/bin/env python3
import os
import time
import numpy as np
from cereal import log
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
# WARNING: imports outside of constants will not trigger a rebuild
from openpilot.selfdrive.modeld.constants import index_function
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU

if __name__ == '__main__':  # generating code
  from acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython

from casadi import SX, vertcat

MODEL_NAME = 'long'
LONG_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LONG_MPC_DIR, "c_generated_code")
JSON_FILE = os.path.join(LONG_MPC_DIR, "acados_ocp_long.json")

LongitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource
MPC_SOURCES = (LongitudinalPlanSource.lead0, LongitudinalPlanSource.lead1, LongitudinalPlanSource.cruise)

X_DIM = 3
U_DIM = 1
PARAM_DIM = 6
COST_E_DIM = 5
COST_DIM = COST_E_DIM + 1
CONSTR_DIM = 4

X_EGO_OBSTACLE_COST = 3.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.
A_CHANGE_COST = 200.
DANGER_ZONE_COST = 100.
CRASH_DISTANCE = .25
LEAD_DANGER_FACTOR = 0.75
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'

# Fewer timestamps don't hurt performance and lead to
# much better convergence of the MPC with low iterations
N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]

T_IDXS = np.array(T_IDXS_LST)
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.])
COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.0
CRUISE_MIN_ACCEL = -1.2
CRUISE_MAX_ACCEL = 1.6
MIN_X_LEAD_FACTOR = 0.5

# dragonpilot (dppb, owner 2026-11-05): "start following 0.3 s after the lead moves; if the planner does not allow it,
# change the planner". Stopped closer than STOP_DISTANCE (the stop end leaves the Altis ~3.5 m out), the MPC waited until
# the lead had pulled out to STOP_DISTANCE: 1.1 s after the lead started (21:53:00), car moving ~2.2 s after it. So while
# we are stopped and the radar lead has been moving away (vLead above LAUNCH_V for LAUNCH_T: radar noise with both
# stopped is <= 0.33 m/s for 99.9 %, single spikes to 0.6), the gap we stopped at is taken as the stop distance (the lead
# obstacle moves out by the shortfall), so the MPC follows the lead's own motion at once. Nothing changes while the lead
# is stopped. Once we move the offset fades at LAUNCH_FADE (the lead pulls away faster), and at LAUNCH_FADE_STOPPED if
# the lead stops again, so a queue that stops again is not followed closer.
LAUNCH_V = 0.25             # m/s
LAUNCH_T = 0.15             # s
LAUNCH_FADE = 0.5           # m/s
LAUNCH_FADE_STOPPED = 2.0   # m/s
LAUNCH_EGO_V = 0.3          # m/s: "we are stopped"
LAUNCH_EXTRA = 0.5          # m beyond the gap we stopped at, so a lead creeping off slowly is followed at once
# dragonpilot (dppb, owner 2026-11-05): "the distance I stopped at behind the truck (09:19:43, radar 4.00 m) is the
# standard stop position; calibrate every stop to it". STOP_DISTANCE is compiled into the acados model, so the MPC is
# given every real lead STOP_EXTRA closer than measured: its stop target (and its following distance) moves out by
# STOP_EXTRA without a rebuild. With the stop end's gaps at 4.0 m (longcontrol), closed-loop sim of 17 engaged stops
# (11-04/11-05, x86 MPC, Altis low-speed map): median 3.32 -> 3.94 m, 3 -> 9 within 3.5-4.5 m; the three gentle stops
# that already end far (lead stopping with us) end ~0.5 m farther (owner chose to road test it).
# 11-12 owner: "4 m is still too far, 3.5 m is right; and follow 0.5-1 m closer below 15 km/h" -> STOP_EXTRA 0 with the
# stop end gaps 3.5 m. Real stops since 11-06 (37): median 4.08 m; sim 17 stops 3.85 -> 3.40 (real est ~3.6 m). The same
# offset sets the following distance: steady following 3-15 km/h (7.7-11.0 m) comes 0.7 m closer, every speed alike.
# 11-15 owner: "try to bring the stop to 3.25 m" -> STOP_EXTRA -0.25 (the lead is given 0.25 m farther than measured)
# with the stop end gaps 3.25 m. Real stops 11-15 (21 behind a stopped lead): median 3.64 m.
STOP_EXTRA = -0.25          # m (11-12..11-15: 0.0; 11-05..11-12: 0.7)
# 11-13 owner: "drop yesterday's closer following below 15 km/h; instead, below 30 km/h - except when the speed has been
# coming down continuously from above 31 km/h - follow 1 m closer, with all the smooth-stop measures as before". The stop
# target stays STOP_EXTRA (3.5 m stop end); only a MOVING lead gets a following offset, blended in with the lead's own
# speed so a lead that stops hands the MPC the unchanged stop target, and every change is rate limited:
#  - normal: the 11-05..11-12 following distance (lead seen FOLLOW_EXTRA closer);
#  - low (<= 30 km/h, not on a continuous slow-down from > 31 km/h): LOW_CLOSER nearer than that. The slow-down from
#    > 31 km/h ends once we stop, or speed back up LOW_RISE above the lowest speed since; > 31 km/h ends low.
# 11-13 owner (later): "raise the switch point to 39 km/h" -> low at <= 39 km/h, slow-downs from > 40 km/h excluded.
FOLLOW_EXTRA = 0.7          # m
LOW_CLOSER = 1.0            # m
LOW_V_ON = 39.0 / 3.6       # m/s (11-13 first version: 30)
LOW_V_OFF = 40.0 / 3.6      # m/s (11-13 first version: 31)
LOW_RISE = 1.0              # m/s
FOLLOW_VLEAD = (0.3, 4.0)   # m/s: lead at/below the first is stopped (stop target), at/above the second is moving
FOLLOW_RATE = 0.25          # m/s: offset change rate
# 11-15 owner: "at 10:32 the MPC kept its following distance while the lead braked, so it braked too hard" + "cut the
# mid braking for a hard-braking lead to -2.54 (the 11:37 three-bookmark template peak command); go to -3.5 only when
# -2.54 would still hit the lead". The MPC's accel floor (a runtime parameter, no new layer) is MID_MIN; it opens to
# ACCEL_MIN while braking at MID_MIN (from the current accel after BRAKE_REACT, reached at MID_JERK) would bring us
# within CRASH_GAP of a lead that keeps its current braking until it stops, and closes again once we have stopped or that gap is over CLEAR_GAP.
# 11-18 owner: "open it once -2.54 would stop within 3 m" -> CRASH_GAP 0.5 -> 3.0 m (CLEAR_GAP 2.5 -> 5.0 m, same 2 m
# hysteresis). With 0.5 m, sudden stopped leads at 40-60 km/h ended 0.8-2 m behind with a stop-go-stop; at 3 m they end
# 3.4-4.1 m with one stop, and the 11-15 10:32 replay ends 1.7-2.2 m instead of 0.5-1.2 m. It now also closes only once the
# plan itself no longer needs more than MID_MIN, so closing can never cut the braking (a 50 km/h / 55 m stop opened at the
# onset and dropped -2.91 -> -2.54 in 0.25 s when it closed on the gap alone).
# Fading the following time out while the lead brakes was tried too and dropped: it lightens the onset (10:32 sim -0.85
# instead of -1.54 at 1.5 s), so the mid had to go to -3.5 anyway.
MID_MIN = -2.54             # m/s^2
MID_JERK = 1.8              # m/s^3 (11:37 template peak command rate)
BRAKE_REACT = 0.5           # s
CRASH_GAP = 3.0             # m (11-15: 0.5)
CLEAR_GAP = 5.0             # m (11-15: 2.5)
CHECK_TAU = 0.3             # 1/s^2: decay of the lead braking assumed by the check (MPC uses 1.5)
CHECK_DT = 0.05             # s
CHECK_T = np.arange(1, 201) * CHECK_DT  # 10 s

# 11-18 owner (route 000001d1): "the mid brake slows down a little early - keep the soft onset as it is, but when no hard
# brake is needed start braking ~0.2-0.3 s later". While closing in on a slower lead that is still far, the lead obstacle
# is moved out by APPROACH_DELAY_T * (vEgo - vLead): during an approach the gap shrinks at vEgo - vLead, so the MPC sees
# the situation later and starts its own (unchanged, soft) build-up later, then makes up the room with a slightly firmer
# mid. Zero when not closing in (steady following, T_FOLLOW unchanged); faded out as the real dRel drops from
# APPROACH_FADE_D[1] to [0] (the mid's catch-up is done well before the end: stop distance kept); faded out as the brake
# the lead needs from the real dRel goes from APPROACH_NEED[0] to [1] (a hard brake is never delayed; the owner's normal
# firm mid is -2.0..-2.4, so a need below 2.0 is a normal stop). Not in the FCW check.
# Sim (stopped lead seen from 120 m, 30-70 km/h; aggressive = the owner's setting): -0.3 reached 0.1-0.45 s later, -0.6
# 0-0.45 s later, peak -2.00..-2.55 (was -2.00..-2.52), build-up rate unchanged; stop aggressive 3.69-4.04 m (was
# 3.79-4.56), relaxed 4.15-4.41 (4.44-4.80), standard 3.50-3.85 (3.81-4.15). Lead braking, cut-in, sudden stopped leads,
# steady following and the PRE=3 stop set identical (45/60 +0.07, 60/85 +0.19 m). Route 000001d1 replays (aggressive):
# 21:01:57 -0.3 0.15 s later, peak -2.32 -> -2.50; 21:03:27 onset 0.21 s later.
APPROACH_DELAY_T = 0.5           # s of closing-speed travel (MPC onset moves ~0.2-0.4 s: it also reacts to the lead appearing)
APPROACH_FADE_D = [45.0, 90.0]   # m (real dRel): none closer than the first, full beyond the second
APPROACH_NEED = [2.0, 3.0]       # m/s^2
APPROACH_S0 = 4.0                # m: the standard stop gap
APPROACH_LAG = 0.5               # s of travel before a firmer brake takes effect

def get_jerk_factor(personality=log.LongitudinalPersonality.standard):
  if personality==log.LongitudinalPersonality.relaxed:
    return 1.0
  elif personality==log.LongitudinalPersonality.standard:
    return 1.0
  elif personality==log.LongitudinalPersonality.aggressive:
    return 0.5
  else:
    raise NotImplementedError("Longitudinal personality not supported")


def get_T_FOLLOW(personality=log.LongitudinalPersonality.standard):
  if personality==log.LongitudinalPersonality.relaxed:
    return 1.75
  elif personality==log.LongitudinalPersonality.standard:
    return 1.45
  elif personality==log.LongitudinalPersonality.aggressive:
    return 1.25
  else:
    raise NotImplementedError("Longitudinal personality not supported")

def get_stopped_equivalence_factor(v_lead):
  return (v_lead**2) / (2 * COMFORT_BRAKE)

def get_safe_obstacle_distance(v_ego, t_follow):
  return (v_ego**2) / (2 * COMFORT_BRAKE) + t_follow * v_ego + STOP_DISTANCE

def gen_long_model():
  model = AcadosModel()
  model.name = MODEL_NAME

  # states
  x_ego, v_ego, a_ego = SX.sym('x_ego'), SX.sym('v_ego'), SX.sym('a_ego')
  model.x = vertcat(x_ego, v_ego, a_ego)

  # controls
  j_ego = SX.sym('j_ego')
  model.u = vertcat(j_ego)

  # xdot
  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  a_ego_dot = SX.sym('a_ego_dot')
  model.xdot = vertcat(x_ego_dot, v_ego_dot, a_ego_dot)

  # live parameters
  a_min = SX.sym('a_min')
  a_max = SX.sym('a_max')
  x_obstacle = SX.sym('x_obstacle')
  a_prev = SX.sym('a_prev')
  lead_t_follow = SX.sym('lead_t_follow')
  lead_danger_factor = SX.sym('lead_danger_factor')
  model.p = vertcat(a_min, a_max, x_obstacle, a_prev, lead_t_follow, lead_danger_factor)

  # dynamics model
  f_expl = vertcat(v_ego, a_ego, j_ego)
  model.f_impl_expr = model.xdot - f_expl
  model.f_expl_expr = f_expl
  return model

def gen_long_ocp():
  ocp = AcadosOcp()
  ocp.model = gen_long_model()

  Tf = T_IDXS[-1]

  # set dimensions
  ocp.dims.N = N

  # set cost module
  ocp.cost.cost_type = 'NONLINEAR_LS'
  ocp.cost.cost_type_e = 'NONLINEAR_LS'

  QR = np.zeros((COST_DIM, COST_DIM))
  Q = np.zeros((COST_E_DIM, COST_E_DIM))

  ocp.cost.W = QR
  ocp.cost.W_e = Q

  x_ego, v_ego, a_ego = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
  j_ego = ocp.model.u[0]

  a_min, a_max = ocp.model.p[0], ocp.model.p[1]
  x_obstacle = ocp.model.p[2]
  a_prev = ocp.model.p[3]
  lead_t_follow = ocp.model.p[4]
  lead_danger_factor = ocp.model.p[5]

  ocp.cost.yref = np.zeros((COST_DIM, ))
  ocp.cost.yref_e = np.zeros((COST_E_DIM, ))

  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow)

  # The main cost in normal operation is how close you are to the "desired" distance
  # from an obstacle at every timestep. This obstacle can be a lead car
  # or other object. In e2e mode we can use x_position targets as a cost
  # instead.
  costs = [((x_obstacle - x_ego) - (desired_dist_comfort)) / (v_ego + 10.),
           x_ego,
           v_ego,
           a_ego,
           a_ego - a_prev,
           j_ego]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])

  # Constraints on speed, acceleration and desired distance to
  # the obstacle, which is treated as a slack constraint so it
  # behaves like an asymmetrical cost.
  constraints = vertcat(v_ego,
                        (a_ego - a_min),
                        (a_max - a_ego),
                        ((x_obstacle - x_ego) - lead_danger_factor * (desired_dist_comfort)) / (v_ego + 10.))
  ocp.model.con_h_expr = constraints

  x0 = np.zeros(X_DIM)
  ocp.constraints.x0 = x0
  ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0, get_T_FOLLOW(), LEAD_DANGER_FACTOR])


  # We put all constraint cost weights to 0 and only set them at runtime
  cost_weights = np.zeros(CONSTR_DIM)
  ocp.cost.zl = cost_weights
  ocp.cost.Zl = cost_weights
  ocp.cost.Zu = cost_weights
  ocp.cost.zu = cost_weights

  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4*np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)

  # The HPIPM solver can give decent solutions even when it is stopped early
  # Which is critical for our purpose where compute time is strictly bounded
  # We use HPIPM in the SPEED_ABS mode, which ensures fastest runtime. This
  # does not cause issues since the problem is well bounded.
  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = 1

  # More iterations take too much time and less lead to inaccurate convergence in
  # some situations. Ideally we would run just 1 iteration to ensure fixed runtime.
  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3

  # set prediction horizon
  ocp.solver_options.tf = Tf
  ocp.solver_options.shooting_nodes = T_IDXS

  ocp.code_export_directory = EXPORT_DIR
  return ocp


class LongitudinalMpc:
  def __init__(self, dt=DT_MDL):
    self.dt = dt
    self.launch_t = 0.0
    self.launch_offset = 0.0
    self.follow_offset = None
    self.from_high = False
    self.low = False
    self.v_min = 0.0
    self.mid_open = False
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.cruise_profile = None  # dragonpilot hook (cruise_decel: the owner's set-0 speed curve, None = stock)
    self.reset()
    self.source = LongitudinalPlanSource.cruise

  def reset(self):
    self.solver.reset()

    self.x_sol = np.zeros((N+1, X_DIM))
    self.u_sol = np.zeros((N, 1))
    self.v_solution = np.zeros(N+1)
    self.a_solution = np.zeros(N+1)
    self.j_solution = np.zeros(N)
    self.a_prev = np.array(self.a_solution)
    self.yref = np.zeros((N+1, COST_DIM))

    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params = np.zeros((N+1, PARAM_DIM))
    for i in range(N+1):
      self.solver.set(i, 'x', np.zeros(X_DIM))

    self.last_cloudlog_t = 0
    self.status = False
    self.crash_cnt = 0.0
    self.solution_status = 0
    # timers
    self.solve_time = 0.0
    self.time_qp_solution = 0.0
    self.time_linearization = 0.0
    self.time_integrator = 0.0
    self.x0 = np.zeros(X_DIM)
    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      # TODO don't hardcode A_CHANGE_COST idx
      # reduce the cost on (a-a_prev) later in the horizon.
      W[4,4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
      self.solver.cost_set(i, 'W', W)
    # Setting the slice without the copy make the array not contiguous,
    # causing issues with the C interface.
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))

    # Set L2 slack cost on lower bound constraints
    Zl = np.array(constraint_cost_weights)
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def set_weights(self, prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard):
    jerk_factor = get_jerk_factor(personality)
    a_change_cost = A_CHANGE_COST if prev_accel_constraint else 0
    cost_weights = [X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST, jerk_factor * a_change_cost, jerk_factor * J_EGO_COST]
    constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]
    self.set_cost_weights(cost_weights, constraint_cost_weights)

  def set_cur_state(self, v, a):
    v_prev = self.x0[1]
    self.x0[1] = v
    self.x0[2] = a
    if abs(v_prev - v) > 2.:  # probably only helps if v < v_prev
      for i in range(N+1):
        self.solver.set(i, 'x', self.x0)

  @staticmethod
  def extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau):
    a_lead_traj = a_lead * np.exp(-a_lead_tau * (T_IDXS**2)/2.)
    v_lead_traj = np.clip(v_lead + np.cumsum(T_DIFFS * a_lead_traj), 0.0, 1e8)
    x_lead_traj = x_lead + np.cumsum(T_DIFFS * v_lead_traj)
    lead_xv = np.column_stack((x_lead_traj, v_lead_traj))
    return lead_xv

  def process_lead(self, lead):
    v_ego = self.x0[1]
    if lead is not None and lead.status:
      x_lead = lead.dRel - STOP_EXTRA  # dragonpilot hook
      v_lead = lead.vLead
      a_lead = lead.aLeadK
      a_lead_tau = lead.aLeadTau
    else:
      # Fake a fast lead car, so mpc can keep running in the same mode
      x_lead = 50.0
      v_lead = v_ego + 10.0
      a_lead = 0.0
      a_lead_tau = _LEAD_ACCEL_TAU

    # MPC will not converge if immediate crash is expected
    # Clip lead distance to what is still possible to brake for
    min_x_lead = MIN_X_LEAD_FACTOR * (v_ego + v_lead) * (v_ego - v_lead) / (-ACCEL_MIN * 2)
    x_lead = np.clip(x_lead, min_x_lead, 1e8)
    v_lead = np.clip(v_lead, 0.0, 1e8)
    a_lead = np.clip(a_lead, -10., 5.)
    lead_xv = self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)
    return lead_xv

  def launch_gap(self, lead, v_ego):
    if not lead.status:
      self.launch_t, self.launch_offset = 0.0, 0.0
      return 0.0
    if v_ego < LAUNCH_EGO_V and self.launch_offset == 0.0:
      self.launch_t = self.launch_t + self.dt if lead.vLead > LAUNCH_V else 0.0
      if self.launch_t >= LAUNCH_T and lead.dRel - STOP_EXTRA < STOP_DISTANCE:
        self.launch_offset = STOP_DISTANCE - (lead.dRel - STOP_EXTRA) + LAUNCH_EXTRA
    elif self.launch_offset > 0.0:
      fade = LAUNCH_FADE_STOPPED if lead.vLead < LAUNCH_V else (LAUNCH_FADE if v_ego >= LAUNCH_EGO_V else 0.0)
      self.launch_offset = max(self.launch_offset - fade * self.dt, 0.0)
      if self.launch_offset == 0.0:
        self.launch_t = 0.0
    return self.launch_offset

  def follow_gap(self, lead, v_ego):
    if v_ego > LOW_V_OFF:
      self.from_high, self.low, self.v_min = True, False, v_ego
    elif self.from_high:
      self.v_min = min(self.v_min, v_ego)
      if v_ego < LAUNCH_EGO_V or v_ego > self.v_min + LOW_RISE:
        self.from_high = False
    if v_ego <= LOW_V_ON and not self.from_high:
      self.low = True
    if not lead.status:
      self.follow_offset = None
      return 0.0
    w = float(np.clip((lead.vLead - FOLLOW_VLEAD[0]) / (FOLLOW_VLEAD[1] - FOLLOW_VLEAD[0]), 0.0, 1.0))
    target = w * ((LOW_CLOSER if self.low else 0.0) - FOLLOW_EXTRA)
    if self.follow_offset is None:
      self.follow_offset = target
    else:
      step = FOLLOW_RATE * self.dt
      self.follow_offset = float(np.clip(target, self.follow_offset - step, self.follow_offset + step))
    return self.follow_offset

  def mid_floor_gap(self, lead, v_ego):
    # least gap to the lead if it keeps its braking and we brake at MID_MIN from now
    if not lead.status:
      return 1e3
    a_l = min(float(np.clip(lead.aLeadK, -10., 5.)), 0.0) * np.exp(-CHECK_TAU * CHECK_T**2 / 2.)
    x_l = lead.dRel + np.cumsum(CHECK_DT * np.clip(lead.vLead + np.cumsum(CHECK_DT * a_l), 0.0, 1e8))
    a0 = min(float(self.x0[2]), 0.0)
    a_e = np.where(CHECK_T < BRAKE_REACT, a0, np.maximum(a0 - MID_JERK * (CHECK_T - BRAKE_REACT), MID_MIN))
    x_e = np.cumsum(CHECK_DT * np.clip(v_ego + np.cumsum(CHECK_DT * a_e), 0.0, 1e8))
    return float(np.min(x_l - x_e))

  def mid_floor(self, radarstate, v_ego):
    gap = min(self.mid_floor_gap(radarstate.leadOne, v_ego), self.mid_floor_gap(radarstate.leadTwo, v_ego))
    if gap < CRASH_GAP:
      self.mid_open = True
    elif v_ego < 0.1 or (gap > CLEAR_GAP and float(np.min(self.a_solution)) >= MID_MIN):
      self.mid_open = False
    return ACCEL_MIN if self.mid_open else MID_MIN

  @staticmethod
  def approach_delay(lead, v_ego):
    if not lead.status:
      return 0.0
    dv = max(v_ego - lead.vLead, 0.0)
    need = max(-lead.aLeadK, 0.0) + dv**2 / (2. * max(lead.dRel - APPROACH_S0 - v_ego * APPROACH_LAG, 1.0))
    w = float(np.interp(lead.dRel, APPROACH_FADE_D, [0.0, 1.0]) * np.interp(need, APPROACH_NEED, [1.0, 0.0]))
    return APPROACH_DELAY_T * dv * w

  def update(self, radarstate, v_cruise, personality=log.LongitudinalPersonality.standard):
    t_follow = get_T_FOLLOW(personality)
    v_ego = self.x0[1]
    self.status = radarstate.leadOne.status or radarstate.leadTwo.status

    lead_xv_0 = self.process_lead(radarstate.leadOne)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)
    lead_xv_0[:, 0] += self.launch_gap(radarstate.leadOne, v_ego)
    follow = self.follow_gap(radarstate.leadOne, v_ego)  # dragonpilot hook
    lead_xv_0[:, 0] += follow
    lead_xv_1[:, 0] += follow
    a_floor = self.mid_floor(radarstate, v_ego)  # dragonpilot hook

    # To estimate a safe distance from a moving lead, we calculate how much stopping
    # distance that lead needs as a minimum. We can add that to the current distance
    # and then treat that as a stopped car/obstacle at this new distance.
    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1])
    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1])
    lead_0_obstacle += self.approach_delay(radarstate.leadOne, v_ego)  # dragonpilot hook
    lead_1_obstacle += self.approach_delay(radarstate.leadTwo, v_ego)  # dragonpilot hook

    # Fake an obstacle for cruise, this ensures smooth acceleration to set speed
    # when the leads are no factor.
    v_lower = v_ego + (T_IDXS * CRUISE_MIN_ACCEL * 1.05) if self.cruise_profile is None else self.cruise_profile(v_ego, self.x0[2], T_IDXS)  # dragonpilot hook
    # TODO does this make sense when max_a is negative?
    v_upper = v_ego + (T_IDXS * CRUISE_MAX_ACCEL * 1.05)
    v_cruise_clipped = np.clip(v_cruise * np.ones(N+1), v_lower, v_upper)
    cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, t_follow)

    x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])
    self.source = MPC_SOURCES[np.argmin(x_obstacles[0])]

    self.yref[:,:] = 0.0
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params[:,0] = a_floor
    self.params[:,1] = ACCEL_MAX
    self.params[:,2] = np.min(x_obstacles, axis=1)
    self.params[:,3] = np.copy(self.a_prev)
    self.params[:,4] = t_follow
    self.params[:,5] = LEAD_DANGER_FACTOR

    self.run()
    if (np.any(lead_xv_0[FCW_IDXS,0] - self.x_sol[FCW_IDXS,0] < CRASH_DISTANCE) and
            radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

  def run(self):
    for i in range(N+1):
      self.solver.set(i, 'p', self.params[i])
    self.solver.constraints_set(0, "lbx", self.x0)
    self.solver.constraints_set(0, "ubx", self.x0)

    self.solution_status = self.solver.solve()
    self.solve_time = float(self.solver.get_stats('time_tot')[0])
    self.time_qp_solution = float(self.solver.get_stats('time_qp')[0])
    self.time_linearization = float(self.solver.get_stats('time_lin')[0])
    self.time_integrator = float(self.solver.get_stats('time_sim')[0])

    for i in range(N+1):
      self.x_sol[i] = self.solver.get(i, 'x')
    for i in range(N):
      self.u_sol[i] = self.solver.get(i, 'u')

    self.v_solution = self.x_sol[:,1]
    self.a_solution = self.x_sol[:,2]
    self.j_solution = self.u_sol[:,0]

    self.a_prev = np.interp(T_IDXS + self.dt, T_IDXS, self.a_solution)

    t = time.monotonic()
    if self.solution_status != 0:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning(f"Long mpc reset, solution_status: {self.solution_status}")
      self.reset()


if __name__ == "__main__":
  ocp = gen_long_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
