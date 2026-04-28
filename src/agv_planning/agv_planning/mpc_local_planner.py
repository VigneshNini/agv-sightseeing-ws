#!/usr/bin/env python3
"""MPC Local Planner — Model Predictive Control using CasADi NLP.

Formulation:
    State    x_k = [px, py, psi, v]          (position, heading, speed)
    Control  u_k = [delta, a]                (steering angle, acceleration)
    Horizon  N = 20 steps at dt = 0.1 s

    min  sum_k  w_pos*||pos_k - ref_k||^2 + w_v*(v_k - v_ref)^2
               + w_d*delta_k^2 + w_a*a_k^2 + w_ddelta*(delta_k - delta_{k-1})^2
               + w_obs * max(0, r_safe - dist(pos_k, obs_j))^2

    s.t. x_{k+1} = f(x_k, u_k)   (bicycle kinematics via RK4)
         v_min <= v_k <= v_max
         |delta_k| <= delta_max
         a_min <= a_k <= a_max
         |delta_k - delta_{k-1}| <= ddelta_max
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from agv_msgs.msg import VehicleCommand, ObstacleArray

try:
    import casadi as ca
    CASADI_AVAILABLE = True
except ImportError:
    CASADI_AVAILABLE = False


def _bicycle_rk4(L, dt):
    """Return a CasADi RK4 integrator for Ackermann bicycle model.

    State: [px, py, psi, v]
    Input: [delta, a]
    """
    x = ca.MX.sym('x', 4)
    u = ca.MX.sym('u', 2)

    def ode(xs, us):
        px, py, psi, v = xs[0], xs[1], xs[2], xs[3]
        delta, a = us[0], us[1]
        dpx = v * ca.cos(psi)
        dpy = v * ca.sin(psi)
        dpsi = v / L * ca.tan(delta)
        dv = a
        return ca.vertcat(dpx, dpy, dpsi, dv)

    k1 = ode(x, u)
    k2 = ode(x + dt / 2 * k1, u)
    k3 = ode(x + dt / 2 * k2, u)
    k4 = ode(x + dt * k3, u)
    x_next = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return ca.Function('F', [x, u], [x_next])


class MPCLocalPlanner(Node):
    def __init__(self):
        super().__init__('mpc_local_planner')
        # --- Parameters ---
        self.declare_parameter('horizon_steps', 20)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('max_speed', 2.5)
        self.declare_parameter('min_speed', 0.0)
        self.declare_parameter('max_steering', 0.6)
        self.declare_parameter('max_steering_rate', 0.3)
        self.declare_parameter('max_accel', 1.5)
        self.declare_parameter('max_decel', 2.0)
        self.declare_parameter('weight_tracking', 10.0)
        self.declare_parameter('weight_speed', 1.0)
        self.declare_parameter('weight_steering', 0.5)
        self.declare_parameter('weight_accel', 0.1)
        self.declare_parameter('weight_steer_rate', 2.0)
        self.declare_parameter('weight_obstacle', 50.0)
        self.declare_parameter('obstacle_safe_radius', 1.5)
        self.declare_parameter('wheelbase', 1.2)

        self.N = self.get_parameter('horizon_steps').value
        self.dt = self.get_parameter('dt').value
        self.v_max = self.get_parameter('max_speed').value
        self.v_min = self.get_parameter('min_speed').value
        self.d_max = self.get_parameter('max_steering').value
        self.dd_max = self.get_parameter('max_steering_rate').value * self.dt
        self.a_max = self.get_parameter('max_accel').value
        self.a_min = -self.get_parameter('max_decel').value
        self.L = self.get_parameter('wheelbase').value

        self.W = {
            'pos': self.get_parameter('weight_tracking').value,
            'v':   self.get_parameter('weight_speed').value,
            'd':   self.get_parameter('weight_steering').value,
            'a':   self.get_parameter('weight_accel').value,
            'dd':  self.get_parameter('weight_steer_rate').value,
            'obs': self.get_parameter('weight_obstacle').value,
        }
        self.r_safe = self.get_parameter('obstacle_safe_radius').value

        # --- State ---
        self.robot_state = np.zeros(4)   # px, py, psi, v
        self.ref_path = None
        self.obstacles = []
        self.prev_delta = 0.0
        self.prev_a = 0.0

        # --- Build CasADi solver once ---
        if CASADI_AVAILABLE:
            self._build_solver()
        else:
            self.get_logger().warn('CasADi not installed — falling back to sampling MPC')
            self.solver = None

        # --- ROS I/O ---
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/agv/pose', self._pose_cb, 10)
        self.path_sub = self.create_subscription(
            Path, '/agv/planned_path', self._path_cb, 10)
        self.obs_sub = self.create_subscription(
            ObstacleArray, '/agv/tracked_obstacles', self._obs_cb, 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/agv/vehicle_command', 10)
        self.create_timer(self.dt, self._control_step)
        self.get_logger().info(
            f'MPC Local Planner initialized (CasADi={"ON" if CASADI_AVAILABLE else "OFF"}, N={self.N}, dt={self.dt})')

    # ------------------------------------------------------------------ #
    #  CasADi NLP builder                                                 #
    # ------------------------------------------------------------------ #
    def _build_solver(self):
        """Construct a parametric NLP solved at each control step.

        Decision variables:  X  (4 x N+1)  States
                             U  (2 x N)     Controls [delta, a]
        Parameters:          p  = [x0(4), ref(3*N), obs_flat]
        """
        N, dt, L = self.N, self.dt, self.L
        F = _bicycle_rk4(L, dt)

        opti = ca.Opti()

        X = opti.variable(4, N + 1)     # states
        U = opti.variable(2, N)         # controls

        # Parameters: initial state + reference trajectory + obstacles
        # ref: Nx3 (px,py,v_ref per step), obs: MAX_OBS x 2 (px,py)
        MAX_OBS = 10
        P_x0  = opti.parameter(4)
        P_ref = opti.parameter(3, N)    # px_ref, py_ref, v_ref
        P_obs = opti.parameter(2, MAX_OBS)   # obstacle positions

        self._MAX_OBS = MAX_OBS

        # ----- Objective -----
        cost = 0
        for k in range(N):
            pos_err = X[:2, k] - P_ref[:2, k]
            v_err   = X[3, k] - P_ref[2, k]
            cost += self.W['pos'] * ca.dot(pos_err, pos_err)
            cost += self.W['v']   * v_err**2
            cost += self.W['d']   * U[0, k]**2
            cost += self.W['a']   * U[1, k]**2
            if k > 0:
                dd = U[0, k] - U[0, k - 1]
                cost += self.W['dd'] * dd**2
            # Obstacle avoidance cost (soft constraint)
            for j in range(MAX_OBS):
                dx = X[0, k] - P_obs[0, j]
                dy = X[1, k] - P_obs[1, j]
                dist2 = dx**2 + dy**2 + 1e-6
                cost += self.W['obs'] * ca.fmax(0, self.r_safe**2 - dist2)**2

        opti.minimize(cost)

        # ----- Dynamics constraints -----
        opti.subject_to(X[:, 0] == P_x0)
        for k in range(N):
            opti.subject_to(X[:, k + 1] == F(X[:, k], U[:, k]))

        # ----- Control bounds -----
        opti.subject_to(opti.bounded(self.v_min, X[3, :], self.v_max))
        opti.subject_to(opti.bounded(-self.d_max, U[0, :], self.d_max))
        opti.subject_to(opti.bounded(self.a_min, U[1, :], self.a_max))
        for k in range(1, N):
            opti.subject_to(ca.fabs(U[0, k] - U[0, k - 1]) <= self.dd_max)

        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 100,
            'ipopt.tol': 1e-4,
        }
        opti.solver('ipopt', opts)

        self._opti = opti
        self._X = X
        self._U = U
        self._P_x0 = P_x0
        self._P_ref = P_ref
        self._P_obs = P_obs
        self.get_logger().info('CasADi NLP solver built successfully')

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #
    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y ** 2 + q.z ** 2))
        self.robot_state = np.array([p.position.x, p.position.y, yaw,
                                     self.robot_state[3]])

    def _path_cb(self, msg: Path):
        self.ref_path = msg.poses

    def _obs_cb(self, msg: ObstacleArray):
        self.obstacles = msg.obstacles

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #
    def _nearest_ref_idx(self):
        if not self.ref_path:
            return 0
        sx, sy = self.robot_state[0], self.robot_state[1]
        best_d, best_i = float('inf'), 0
        for i, p in enumerate(self.ref_path):
            dx = p.pose.position.x - sx
            dy = p.pose.position.y - sy
            d = dx * dx + dy * dy
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def _build_ref(self, start_idx: int):
        """Build N-step reference matrix [px_ref, py_ref, v_ref]."""
        ref = np.zeros((3, self.N))
        path_len = len(self.ref_path)
        for k in range(self.N):
            idx = min(start_idx + k + 1, path_len - 1)
            ref[0, k] = self.ref_path[idx].pose.position.x
            ref[1, k] = self.ref_path[idx].pose.position.y
            ref[2, k] = self.v_max * 0.6
        return ref

    def _build_obs_param(self):
        M = self._MAX_OBS
        obs_arr = np.zeros((2, M))
        for j, obs in enumerate(self.obstacles[:M]):
            obs_arr[0, j] = obs.position.x
            obs_arr[1, j] = obs.position.y
        return obs_arr

    # ------------------------------------------------------------------ #
    #  Main control step                                                   #
    # ------------------------------------------------------------------ #
    def _control_step(self):
        if self.ref_path is None or len(self.ref_path) < 2:
            return

        ref_idx = self._nearest_ref_idx()
        if ref_idx >= len(self.ref_path) - 1:
            self._publish(0.0, 0.0, brake=1.0)
            return

        if CASADI_AVAILABLE and hasattr(self, '_opti'):
            delta, accel = self._solve_casadi(ref_idx)
        else:
            delta, accel = self._solve_sampling(ref_idx)

        v_next = float(np.clip(
            self.robot_state[3] + accel * self.dt,
            self.v_min, self.v_max))
        self.robot_state[3] = v_next
        self.prev_delta = delta
        self.prev_a = accel
        self._publish(v_next, delta)

    def _solve_casadi(self, ref_idx: int):
        try:
            self._opti.set_value(self._P_x0, self.robot_state)
            self._opti.set_value(self._P_ref, self._build_ref(ref_idx))
            self._opti.set_value(self._P_obs, self._build_obs_param())
            # Warm-start
            self._opti.set_initial(self._U[0, :], self.prev_delta)
            self._opti.set_initial(self._U[1, :], 0.0)
            sol = self._opti.solve()
            U_opt = sol.value(self._U)
            return float(U_opt[0, 0]), float(U_opt[1, 0])
        except Exception as e:
            self.get_logger().warn(f'CasADi solve failed: {e} — using prev control')
            return self.prev_delta, 0.0

    def _solve_sampling(self, ref_idx: int):
        """Fallback: coarse grid search over (delta, a) pairs."""
        best_cost, best_d, best_a = float('inf'), 0.0, 0.0
        ref = self._build_ref_np(ref_idx)
        for d in np.linspace(-self.d_max, self.d_max, 7):
            if abs(d - self.prev_delta) > self.dd_max * 10:
                continue
            for a in np.linspace(self.a_min, self.a_max, 5):
                cost = self._rollout_cost(d, a, ref)
                if cost < best_cost:
                    best_cost, best_d, best_a = cost, d, a
        return best_d, best_a

    def _build_ref_np(self, start_idx: int):
        path_len = len(self.ref_path)
        ref = []
        for k in range(self.N):
            idx = min(start_idx + k + 1, path_len - 1)
            ref.append((self.ref_path[idx].pose.position.x,
                        self.ref_path[idx].pose.position.y,
                        self.v_max * 0.6))
        return ref

    def _rollout_cost(self, delta: float, a: float, ref: list) -> float:
        s = self.robot_state.copy()
        cost = 0.0
        for k, (rx, ry, rv) in enumerate(ref):
            # Euler step
            v = float(np.clip(s[3] + a * self.dt, self.v_min, self.v_max))
            s[0] += v * math.cos(s[2]) * self.dt
            s[1] += v * math.sin(s[2]) * self.dt
            s[2] += v / self.L * math.tan(delta) * self.dt
            s[3] = v
            cost += self.W['pos'] * ((s[0] - rx) ** 2 + (s[1] - ry) ** 2)
            cost += self.W['v'] * (v - rv) ** 2
            cost += self.W['d'] * delta ** 2
            for obs in self.obstacles:
                dx, dy = s[0] - obs.position.x, s[1] - obs.position.y
                dist = math.sqrt(dx * dx + dy * dy + 1e-6)
                if dist < self.r_safe:
                    cost += self.W['obs'] * (self.r_safe - dist) ** 2
        return cost

    def _publish(self, speed: float, steering: float, brake: float = 0.0):
        cmd = VehicleCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.speed = float(speed)
        cmd.steering_angle = float(np.clip(steering, -self.d_max, self.d_max))
        cmd.brake = float(brake)
        cmd.mode = 'auto'
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MPCLocalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
