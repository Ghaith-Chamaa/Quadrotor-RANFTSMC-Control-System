"""
AXIS-SPECIFIC RANFTSMC Controller
Flexible control with independent parameter tuning per axis

STRATEGY:
1. All axes (XYZ): Axis-specific direct velocity command approach
   - Full adaptive RANFTSMC with direct velocity commands
   - Separate parameters for X, Y, Z axes for independent tuning
2. Axis-specific architecture: Direct velocity for all three axes
"""

import numpy as np
from typing import Tuple, Dict
from Params import QuadrotorParams, ControllerParams, SafetyParams


class RANFTSMController:
    """
    Robust Adaptive NFTSM Controller - AXIS-SPECIFIC VERSION

    FEATURES:
    - All axes (XYZ): Full adaptive RANFTSMC with direct velocity commands
    - Axis-specific parameters for independent tuning of X, Y, and Z
    - Flexible architecture supporting different control characteristics per axis
    """

    def __init__(self, quad_params: QuadrotorParams, ctrl_params: ControllerParams,
                 safety_params: SafetyParams = None, debug=False):
        self.qp = quad_params
        self.cp = ctrl_params
        self.sp = safety_params if safety_params is not None else SafetyParams()
        self.debug = debug

        self.a_hat_pos = np.zeros((3, 3))
        self.a_hat_att = np.zeros((3, 3))

        self.prev_state = None
        self.prev_time = None
        self.prev_cmd = {'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'yaw_rate': 0.0}
        self.alpha_smooth = 0.1
        self.iteration = 0

        print(f"\n{'='*60}")
        print(f"UNIFIED RANFTSMC CONTROLLER INITIALIZED")
        print(f"{'='*60}")
        print(f"All axes (XYZ): RANFTSMC with axis-specific parameters")
        print(f"  X-axis: bn={ctrl_params.bn_x}, cn={ctrl_params.cn_x}, hn={ctrl_params.hn_x}")
        print(f"  Y-axis: bn={ctrl_params.bn_y}, cn={ctrl_params.cn_y}, hn={ctrl_params.hn_y}")
        print(f"  Z-axis: bn={ctrl_params.bn_z}, cn={ctrl_params.cn_z}, hn={ctrl_params.hn_z}")
        print(f"{'='*60}\n")

    def _sign_smooth(self, x: float, epsilon: float) -> float:
        """Smooth sign function"""
        return np.tanh(x / epsilon)

    def position_control(self, state: np.ndarray, desired: Dict, dt: float) -> Dict[str, float]:
        """
        Position control - Axis-specific direct velocity implementation
        X, Y, Z axes: Separate parameters for independent tuning
        """
        pos = state[0:3]
        vel = state[3:6]
        pos_d = desired['pos']
        vel_d = desired['vel']
        acc_d = desired['acc']

        e_pos = pos - pos_d
        e_vel = vel - vel_d
        v_cmd = np.zeros(3)

        for i in range(3):
            if i == 0:
                bn = self.cp.bn_x
                bn_plus1 = self.cp.bn_plus1_x
                cn = self.cp.cn_x
                beta_n = self.cp.beta_n_x
                alpha_n = self.cp.alpha_n_x
                hn = self.cp.hn_x
                mu0n = self.cp.mu0n_x
                mu1n = self.cp.mu1n_x
                mu2n = self.cp.mu2n_x
                rho = -self.qp.Kx / self.qp.m
            elif i == 1:
                bn = self.cp.bn_y
                bn_plus1 = self.cp.bn_plus1_y
                cn = self.cp.cn_y
                beta_n = self.cp.beta_n_y
                alpha_n = self.cp.alpha_n_y
                hn = self.cp.hn_y
                mu0n = self.cp.mu0n_y
                mu1n = self.cp.mu1n_y
                mu2n = self.cp.mu2n_y
                rho = -self.qp.Ky / self.qp.m
            else:
                bn = self.cp.bn_z
                bn_plus1 = self.cp.bn_plus1_z
                cn = self.cp.cn_z
                beta_n = self.cp.beta_n_z
                alpha_n = self.cp.alpha_n_z
                hn = self.cp.hn_z
                mu0n = self.cp.mu0n_z
                mu1n = self.cp.mu1n_z
                mu2n = self.cp.mu2n_z
                rho = -self.qp.Kz / self.qp.m

            # Sliding surface (equation 26)
            sigma = (e_pos[i] +
                     bn * np.abs(e_pos[i])**alpha_n * np.sign(e_pos[i]) +
                     bn_plus1 * np.abs(e_vel[i])**beta_n * np.sign(e_vel[i]))

            # Adaptive parameter updates
            if dt > 0:
                self.a_hat_pos[i, 0] += mu0n * np.abs(sigma) * np.abs(e_vel[i])**(beta_n - 1) * dt
                self.a_hat_pos[i, 1] += mu1n * np.abs(sigma) * np.abs(e_pos[i]) * np.abs(e_vel[i])**(beta_n - 1) * dt
                self.a_hat_pos[i, 2] += mu2n * np.abs(sigma) * np.abs(e_vel[i])**beta_n * dt

            # Saturate adaptive parameters
            self.a_hat_pos[i] = np.clip(self.a_hat_pos[i], 0, 15.0)

            # Equivalent control (direct velocity)
            v_eq = (vel_d[i] +
                   (acc_d[i] - rho * vel[i] -
                   (1.0 / (beta_n * bn_plus1)) *
                   np.abs(e_vel[i])**(2 - beta_n) *
                   (1 + alpha_n * bn * np.abs(e_pos[i])**(alpha_n - 1)) *
                   self._sign_smooth(e_vel[i], self.cp.epsilon)) * dt)

            # Adaptive switching gain
            K = (self.a_hat_pos[i, 0] +
                 self.a_hat_pos[i, 1] * np.abs(e_pos[i]) +
                 self.a_hat_pos[i, 2] * np.abs(e_vel[i]) +
                 hn)

            # Switching control
            v_sw = (-cn * sigma - K * self._sign_smooth(sigma, self.cp.epsilon)) * 0.5

            v_cmd[i] = v_eq + v_sw

        # Apply velocity scaling
        v_cmd[0] *= self.cp.vel_scale_x
        v_cmd[1] *= self.cp.vel_scale_y
        v_cmd[2] *= self.cp.vel_scale_z

        # Apply safety limits
        v_cmd[0] = np.clip(v_cmd[0], -self.sp.max_vel_horizontal, self.sp.max_vel_horizontal)
        v_cmd[1] = np.clip(v_cmd[1], -self.sp.max_vel_horizontal, self.sp.max_vel_horizontal)
        v_cmd[2] = np.clip(v_cmd[2], -self.sp.max_vel_vertical, self.sp.max_vel_vertical)

        return {'vx': v_cmd[0], 'vy': v_cmd[1], 'vz': v_cmd[2]}

    def attitude_control(self, state: np.ndarray, yaw_d: float, yaw_dot_d: float, dt: float) -> float:
        """Attitude control (yaw only)"""
        psi = state[8]
        psi_dot = state[11]

        e_att = psi - yaw_d
        e_rate = psi_dot - yaw_dot_d
        e_att = np.arctan2(np.sin(e_att), np.cos(e_att))

        # Sliding surface
        sigma = (e_att +
                self.cp.bj * np.abs(e_att)**self.cp.alpha_j * np.sign(e_att) +
                self.cp.bj_plus1 * np.abs(e_rate)**self.cp.beta_j * np.sign(e_rate))

        # Adaptive updates
        i = 2
        if dt > 0:
            self.a_hat_att[i, 0] += self.cp.mu0j * np.abs(sigma) * np.abs(e_rate)**(self.cp.beta_j - 1) * dt
            self.a_hat_att[i, 1] += self.cp.mu1j * np.abs(sigma) * np.abs(e_att) * np.abs(e_rate)**(self.cp.beta_j - 1) * dt
            self.a_hat_att[i, 2] += self.cp.mu2j * np.abs(sigma) * np.abs(e_rate)**self.cp.beta_j * dt

        # Saturate adaptive parameters
        self.a_hat_att[i] = np.clip(self.a_hat_att[i], 0, 8.0)

        # Control law
        yaw_rate_eq = yaw_dot_d - 0.3 * e_att - 0.2 * e_rate

        K = (self.a_hat_att[i, 0] +
             self.a_hat_att[i, 1] * np.abs(e_att) +
             self.a_hat_att[i, 2] * np.abs(e_rate) +
             self.cp.hj)

        yaw_rate_sw = (-self.cp.cj * sigma - K * self._sign_smooth(sigma, self.cp.epsilon)) * 0.1

        yaw_rate = (yaw_rate_eq + yaw_rate_sw) * self.cp.yaw_rate_scale

        return np.clip(yaw_rate, -self.sp.max_yaw_rate, self.sp.max_yaw_rate)

    def compute_control(self, state: np.ndarray, desired: Dict, dt: float) -> Dict[str, float]:
        """
        Main control function

        Args:
            state: Current state [x,y,z,vx,vy,vz,φ,θ,ψ,p,q,r]
            desired: Desired trajectory state
            dt: Time step

        Returns:
            Command dict: {'vx', 'vy', 'vz', 'yaw_rate'}
        """
        vel_cmd = self.position_control(state, desired, dt)

        yaw_rate = self.attitude_control(state, desired['yaw'], desired['yaw_dot'], dt)

        cmd = {
            'vx': vel_cmd['vx'],
            'vy': vel_cmd['vy'],
            'vz': vel_cmd['vz'],
            'yaw_rate': yaw_rate
        }

        if self.alpha_smooth > 0:
            for key in cmd:
                cmd[key] = (self.alpha_smooth * self.prev_cmd[key] +
                           (1 - self.alpha_smooth) * cmd[key])

        self.prev_cmd = cmd.copy()
        self.prev_state = state.copy()
        self.prev_time = dt
        self.iteration += 1

        if self.debug and self.iteration % 50 == 0:
            pos_err = state[0:3] - desired['pos']
            print(f"[{self.iteration:4d}] e_pos=[{pos_err[0]:+.3f},{pos_err[1]:+.3f},{pos_err[2]:+.3f}] "
                  f"v_cmd=[{cmd['vx']:+.3f},{cmd['vy']:+.3f},{cmd['vz']:+.3f}]")

        return cmd

    def reset(self):
        """Reset controller state"""
        self.a_hat_pos = np.zeros((3, 3))
        self.a_hat_att = np.zeros((3, 3))
        self.prev_state = None
        self.prev_time = None
        self.prev_cmd = {'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'yaw_rate': 0.0}
        self.iteration = 0
        print("Controller reset")
