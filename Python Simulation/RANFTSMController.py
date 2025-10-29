from typing import Tuple, Dict
import numpy as np

from Params import QuadrotorParams, ControllerParams

class RANFTSMController:
    """
    Robust Adaptive Nonsingular Fast Terminal Sliding Mode Controller
    Implements equations from paper Section 3
    """
    
    def __init__(self, quad_params: QuadrotorParams, ctrl_params: ControllerParams):
        self.qp = quad_params
        self.cp = ctrl_params
        
        self.a_hat_pos = np.zeros((3, 3))  # [a0, a1, a2] for [x, y, z]
        self.a_hat_att = np.zeros((3, 3))  # [a0, a1, a2] for [phi, theta, psi]
        
        self.prev_error_pos = np.zeros(3)
        self.prev_error_vel = np.zeros(3)
        self.prev_error_att = np.zeros(3)
        self.prev_error_rate = np.zeros(3)
        self.prev_time = 0.0
    
    def _sign(self, x: float, epsilon: float = 0.1) -> float:
        """Smooth sign function using tanh to avoid chattering"""
        return np.tanh(x / epsilon)
    
    def position_controller(self, state: np.ndarray, desired: Dict, dt: float) -> Tuple[float, np.ndarray]:
        """
        Outer loop position controller
        Returns: (thrust, desired_angles)
        Implements Eqs. (26), (41)-(43), (44)-(46)
        """
        # State: [x, y, z, vx, vy, vz, phi, theta, psi, phi_dot, theta_dot, psi_dot]
        pos = state[0:3]
        vel = state[3:6]
        
        pos_d = desired['pos']
        vel_d = desired['vel']
        acc_d = desired['acc']
        
        # Position and velocity errors
        e_pos = pos - pos_d
        e_vel = vel - vel_d
        
        # Virtual control inputs V = [Vx, Vy, Vz]
        V = np.zeros(3)
        
        for i in range(3):
            # Sliding surface (Eq. 26)
            sigma = (e_pos[i] + 
                    self.cp.bn * np.abs(e_pos[i])**self.cp.alpha_n * np.sign(e_pos[i]) +
                    self.cp.bn_plus1 * np.abs(e_vel[i])**self.cp.beta_n * np.sign(e_vel[i]))
            
            # Adaptive parameter updates (Eqs. 44-46)
            self.a_hat_pos[i, 0] += self.cp.mu0n * np.abs(sigma) * np.abs(e_vel[i])**(self.cp.beta_n - 1) * dt
            self.a_hat_pos[i, 1] += self.cp.mu1n * np.abs(sigma) * np.abs(e_pos[i]) * np.abs(e_vel[i])**(self.cp.beta_n - 1) * dt
            self.a_hat_pos[i, 2] += self.cp.mu2n * np.abs(sigma) * np.abs(e_vel[i])**self.cp.beta_n * dt
            
            # Equivalent control (from Eq. 28)
            rho = -[self.qp.Kx/self.qp.m, self.qp.Ky/self.qp.m, self.qp.Kz/self.qp.m][i]
            V_eq = (acc_d[i] - rho * vel[i] - 
                   (1.0 / (self.cp.beta_n * self.cp.bn_plus1)) * 
                   np.abs(e_vel[i])**(2 - self.cp.beta_n) *
                   (1 + self.cp.alpha_n * self.cp.bn * np.abs(e_pos[i])**(self.cp.alpha_n - 1)) *
                   self._sign(e_vel[i]))
            
            # Switching control (Eq. 29)
            K = (self.a_hat_pos[i, 0] + 
                 self.a_hat_pos[i, 1] * np.abs(e_pos[i]) +
                 self.a_hat_pos[i, 2] * np.abs(e_vel[i]) +
                 self.cp.hn)
            # K = self.cp.hn
            
            V_sw = -self.cp.cn * sigma - K * self._sign(sigma)
            
            # Total control (Eq. 41-43)
            V[i] = V_eq + V_sw
        
        # Convert virtual control to thrust and desired angles (Eq. 20)

        psi_d = desired['yaw']
        
        # The denominator is (Vz + g)
        V_z_eff = V[2] + self.qp.g
        
        # Thrust magnitude (Eq. 20)
        U_T = self.qp.m * np.sqrt(V[0]**2 + V[1]**2 + V_z_eff**2)
        
        # Desired roll and pitch (Eq. 20)
        # Note: We need C_theta_d for phi_d calculation, but we don't have theta_d yet => a coupling issue 
        # The paper assumes small angles (assuming cos(theta_d) ≈ 1):
        phi_d = np.arctan2(V[0]*np.sin(psi_d) - V[1]*np.cos(psi_d), V_z_eff)
        theta_d = np.arctan2(V[0]*np.cos(psi_d) + V[1]*np.sin(psi_d), V_z_eff)
        
        # Limit angles to prevent gimbal lock and maintain small angle assumption
        # angles are limited per the paper modelling description [-pi/2, pi/2]
        phi_d = np.clip(phi_d, -np.pi/2, np.pi/2)      # ±90 degrees
        theta_d = np.clip(theta_d, -np.pi/2, np.pi/2)  # ±90 degrees
        
        desired_angles = np.array([phi_d, theta_d, psi_d])
        
        return U_T, desired_angles
    
    def attitude_controller(self, state: np.ndarray, desired_angles: np.ndarray, 
                          desired_yaw_dot: float, desired_yaw_ddot: float, dt: float) -> np.ndarray:
        """
        Inner loop attitude controller
        Returns: torques [U_phi, U_theta, U_psi]
        Implements Eqs. (56)-(62)
        """
        att = state[6:9]  # [phi, theta, psi]
        rates = state[9:12]  # [phi_dot, theta_dot, psi_dot]
        
        # Desired values (roll and pitch rates assumed zero for tracking)
        att_d = desired_angles
        rates_d = np.array([0.0, 0.0, desired_yaw_dot])
        acc_d = np.array([0.0, 0.0, desired_yaw_ddot])
        
        e_att = att - att_d
        e_rate = rates - rates_d
        
        U = np.zeros(3)
        
        omega_rotor = 0.0  # Simplified
        
        for i in range(3):
            # Sliding surface (Eq. 56)
            sigma = (e_att[i] + 
                    self.cp.bj * np.abs(e_att[i])**self.cp.alpha_j * np.sign(e_att[i]) +
                    self.cp.bj_plus1 * np.abs(e_rate[i])**self.cp.beta_j * np.sign(e_rate[i]))
            
            # Adaptive updates (Eqs. 60-62)
            self.a_hat_att[i, 0] += self.cp.mu0j * np.abs(sigma) * np.abs(e_rate[i])**(self.cp.beta_j - 1) * dt
            self.a_hat_att[i, 1] += self.cp.mu1j * np.abs(sigma) * np.abs(e_att[i]) * np.abs(e_rate[i])**(self.cp.beta_j - 1) * dt
            self.a_hat_att[i, 2] += self.cp.mu2j * np.abs(sigma) * np.abs(e_rate[i])**self.cp.beta_j * dt
            
            if i == 0:  # Roll
                J = self.qp.Jxx
                rho1 = (self.qp.Jyy - self.qp.Jzz) / J
                rho2 = -omega_rotor * self.qp.Jr / J
                rho3 = -self.qp.Kphi / J
                coupling = rho1 * rates[1] * rates[2] + rho2 * rates[1] + rho3 * rates[0]**2
            elif i == 1:  # Pitch
                J = self.qp.Jyy
                rho1 = (self.qp.Jzz - self.qp.Jxx) / J
                rho2 = omega_rotor * self.qp.Jr / J
                rho3 = -self.qp.Ktheta / J
                coupling = rho1 * rates[0] * rates[2] + rho2 * rates[0] + rho3 * rates[1]**2
            else:  # Yaw
                J = self.qp.Jzz
                rho1 = (self.qp.Jxx - self.qp.Jyy) / J
                rho2 = -self.qp.Kpsi / J
                coupling = rho1 * rates[0] * rates[1] + rho2 * rates[2]**2
            
            # Equivalent control
            U_eq = (acc_d[i] - coupling -
                   (1.0 / (self.cp.beta_j * self.cp.bj_plus1)) *
                   np.abs(e_rate[i])**(2 - self.cp.beta_j) *
                   (1 + self.cp.alpha_j * self.cp.bj * np.abs(e_att[i])**(self.cp.alpha_j - 1)) *
                   self._sign(e_rate[i]))
            
            # Switching control
            K = (self.a_hat_att[i, 0] + 
                 self.a_hat_att[i, 1] * np.abs(e_att[i]) +
                 self.a_hat_att[i, 2] * np.abs(e_rate[i]) +
                 self.cp.hj)
            # K = self.cp.hj
            U_sw = -self.cp.cj * sigma - K * self._sign(sigma)
            
            # Total control with proper scaling
            if i == 0:
                U[i] = self.qp.Jxx * (U_eq + U_sw)
            elif i == 1:
                U[i] = self.qp.Jyy * (U_eq + U_sw)
            else:
                U[i] = self.qp.Jzz * (U_eq + U_sw)
        
        return U