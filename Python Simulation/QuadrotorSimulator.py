from typing import Dict
import numpy as np
import time

from Params import QuadrotorParams
from RANFTSMController import RANFTSMController
from Generators import TrajectoryGenerator, DisturbanceGenerator

class QuadrotorSimulator:
    """Simulates quadrotor dynamics with RANFTSMC control"""
    
    def __init__(self, quad_params: QuadrotorParams, controller: RANFTSMController):
        self.qp = quad_params
        self.controller = controller
        
    def dynamics(self, state: np.ndarray, U_T: float, torques: np.ndarray, 
                disturbances: Dict) -> np.ndarray:
        """
        Quadrotor dynamics from paper Eqs. (9), (13), (17)
        State: [x, y, z, vx, vy, vz, phi, theta, psi, phi_dot, theta_dot, psi_dot]
        """
        pos = state[0:3]
        vel = state[3:6]
        att = state[6:9]
        rates = state[9:12]
        
        phi, theta, psi = att
        
        d_pos = disturbances['position']
        d_att = disturbances['attitude']
        
        # Rotation matrix (Eq. 1)
        c_phi, s_phi = np.cos(phi), np.sin(phi)
        c_theta, s_theta = np.cos(theta), np.sin(theta)
        c_psi, s_psi = np.cos(psi), np.sin(psi)
        
        # Position dynamics (Eq. 9)
        acc_x = (-(self.qp.Kx/self.qp.m) * vel[0] + 
                (1/self.qp.m) * (s_theta * c_psi * c_phi + s_psi * s_phi) * U_T + d_pos[0])
        acc_y = (-(self.qp.Ky/self.qp.m) * vel[1] + 
                (1/self.qp.m) * (s_theta * s_psi * c_phi - c_psi * s_phi) * U_T + d_pos[1])
        acc_z = (-(self.qp.Kz/self.qp.m) * vel[2] - self.qp.g + 
                (1/self.qp.m) * (c_theta * c_phi) * U_T + d_pos[2])
        
        # Attitude dynamics (Eq. 13)
        omega_rotor = 0.0  # Simplified
        
        acc_phi = ((1/self.qp.Jxx) * 
                  (rates[1] * rates[2] * (self.qp.Jyy - self.qp.Jzz) - 
                   self.qp.Jr * rates[1] * omega_rotor -
                   self.qp.Kphi * rates[0]**2 + torques[0]) + d_att[0])
        
        acc_theta = ((1/self.qp.Jyy) * 
                    (rates[0] * rates[2] * (self.qp.Jzz - self.qp.Jxx) + 
                     self.qp.Jr * rates[0] * omega_rotor -
                     self.qp.Ktheta * rates[1]**2 + torques[1]) + d_att[1])
        
        acc_psi = ((1/self.qp.Jzz) * 
                  (rates[0] * rates[1] * (self.qp.Jxx - self.qp.Jyy) -
                   self.qp.Kpsi * rates[2]**2 + torques[2]) + d_att[2])
        
        dstate = np.zeros(12)
        dstate[0:3] = vel
        dstate[3:6] = [acc_x, acc_y, acc_z]
        dstate[6:9] = rates
        dstate[9:12] = [acc_phi, acc_theta, acc_psi]
        
        return dstate
    
    def simulate(self, trajectory_type: str, disturbance_type: str,
                initial_conditions: np.ndarray = None,
                T_sim: float = None, dt: float = 0.01, verbose: bool = False):
        """Run simulation with specified trajectory and disturbances"""
        
        if T_sim is None:
            if trajectory_type == '8':
                T_sim = 2 * np.pi / 0.04
            elif trajectory_type == 'circle':
                T_sim = 40.0
            elif trajectory_type == 'square':
                T_sim = 50.0
            elif trajectory_type == 'complex':
                T_sim = 80.0
            elif trajectory_type == 'space8':
                T_sim = 120.0
            else:
                T_sim = 40.0
        
        t_vec = np.arange(0, T_sim, dt)
        N = len(t_vec)
        
        states = np.zeros((N, 12))
        states_desired = np.zeros((N, 12))
        controls = np.zeros((N, 4))  # [U_T, U_phi, U_theta, U_psi]
        
        if initial_conditions is not None:
            states[0, 0:3] = initial_conditions[0:3]  # position
            states[0, 6:9] = initial_conditions[3:6]  # attitude
            # velocities and rates start at zero
            if verbose:
                print(f"  Using custom initial conditions:")
                print(f"    Position: [{initial_conditions[0]:.2f}, {initial_conditions[1]:.2f}, {initial_conditions[2]:.2f}] m")
                print(f"    Attitude: [{initial_conditions[3]:.2f}, {initial_conditions[4]:.2f}, {initial_conditions[5]:.2f}] rad")
        else:
            if trajectory_type == '8':
                states[0] = np.array([0, 0, 0.6, 0, 0, 0, 0, 0, 0, 0, 0, 0])
            elif trajectory_type == 'circle':
                states[0] = np.array([0.5, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0])
            elif trajectory_type == 'square':
                states[0] = np.array([0.6, 0.6, 0.6, 0, 0, 0, 0, 0, 0, 0, 0, 0])
            elif trajectory_type == 'complex':
                states[0] = np.array([0.5, 0, 1, 0, 0, 0, 0, 0, 0.5, 0, 0, 0])
            elif trajectory_type == 'space8':
                states[0] = np.array([0, 0, 0.5, 0, 0, 0, 0, 0, 0.5, 0, 0, 0])
            else:
                states[0] = np.array([0, 0, 0.6, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        
        if trajectory_type == '8':
            traj_gen = TrajectoryGenerator.figure_eight
        elif trajectory_type == 'circle':
            traj_gen = TrajectoryGenerator.circular
        elif trajectory_type == 'square':
            traj_gen = TrajectoryGenerator.square
        elif trajectory_type == 'complex':
            traj_gen = TrajectoryGenerator.complex_trajectory
        elif trajectory_type == 'space8':
            traj_gen = TrajectoryGenerator.space_eight
        else:
            raise ValueError(f"Unknown trajectory type: {trajectory_type}")
        
        if disturbance_type == 'none':
            dist_gen = DisturbanceGenerator.none
        elif disturbance_type == 'constant':
            dist_gen = lambda t: DisturbanceGenerator.constant(t, "scenario2")
        elif disturbance_type == 'time_varying':
            dist_gen = lambda t: DisturbanceGenerator.time_varying(t, "scenario3")
        elif disturbance_type == 'aggressive':
            dist_gen = lambda t: DisturbanceGenerator.time_varying(t, "scenario5")
        else:
            raise ValueError(f"Unknown disturbance type: {disturbance_type}")
        
        start_time = time.time()
        
        for i in range(N-1):
            t = t_vec[i]
            
            if verbose and i % 1000 == 0:
                print(f"  Simulating: t = {t:.2f}s / {T_sim:.2f}s ({100*i/N:.1f}%)")
            
            desired = traj_gen(t)
            states_desired[i, 0:3] = desired['pos'] # pos_d from trajectory
            states_desired[i, 3:6] = desired['vel'] # vel_d from trajectory
            states_desired[i, 6] = 0.0  # phi_d from outer loop
            states_desired[i, 7] = 0.0  # theta_d from outer loop
            states_desired[i, 8] = desired['yaw'] # psi_d from trajectory
            
            disturbances = dist_gen(t)
            
            # Position controller (outer loop)
            U_T, desired_angles = self.controller.position_controller(
                states[i], desired, dt)
            
            # Update desired angles in states_desired
            states_desired[i, 6] = desired_angles[0]  # phi_d from outer loop
            states_desired[i, 7] = desired_angles[1]  # theta_d from outer loop
            
            # Attitude controller (inner loop)
            torques = self.controller.attitude_controller(
                states[i], desired_angles, desired['yaw_dot'], desired['yaw_ddot'], dt)
            
            controls[i] = [U_T, torques[0], torques[1], torques[2]]
            
            # Update dynamics (Runge-Kutta 4th order)
            k1 = self.dynamics(states[i], U_T, torques, disturbances)
            k2 = self.dynamics(states[i] + 0.5*dt*k1, U_T, torques, disturbances)
            k3 = self.dynamics(states[i] + 0.5*dt*k2, U_T, torques, disturbances)
            k4 = self.dynamics(states[i] + dt*k3, U_T, torques, disturbances)
            
            states[i+1] = states[i] + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
        
        desired = traj_gen(t_vec[-1])
        states_desired[-1, 0:3] = desired['pos']
        states_desired[-1, 3:6] = desired['vel']
        states_desired[-1, 8] = desired['yaw']
        
        sim_time = time.time() - start_time
        
        if verbose:
            print(f"\nSimulation completed in {sim_time:.2f}s")
            print(f"Real-time factor: {T_sim/sim_time:.1f}x")
        
        return t_vec, states, states_desired, controls