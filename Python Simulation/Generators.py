import numpy as np
from typing import Dict

class TrajectoryGenerator:
    """Generates reference trajectories from paper"""
    
    @staticmethod
    def figure_eight(t: float, A: float = 1.0, B: float = 1.0, omega: float = 0.04) -> Dict[str, np.ndarray]:
        """
        Generate 8-shaped trajectory
        """
        x = A * np.sin(omega * t)
        y = B * np.sin(2 * omega * t)
        z = 0.6 
        
        x_dot = A * omega * np.cos(omega * t)
        y_dot = 2 * B * omega * np.cos(2 * omega * t)
        z_dot = 0.0
        
        x_ddot = -A * omega**2 * np.sin(omega * t)
        y_ddot = -4 * B * omega**2 * np.sin(2 * omega * t)
        z_ddot = 0.0
        
        psi = 0.0
        psi_dot = 0.0
        psi_ddot = 0.0
        
        return {
            'pos': np.array([x, y, z]),
            'vel': np.array([x_dot, y_dot, z_dot]),
            'acc': np.array([x_ddot, y_ddot, z_ddot]),
            'yaw': psi,
            'yaw_dot': psi_dot,
            'yaw_ddot': psi_ddot
        }
    
    @staticmethod
    def circular(t: float, radius: float = 0.5, omega: float = np.pi / 20) -> Dict[str, np.ndarray]:
        """
        Generate circular trajectory
        From paper Eq. (69) - Simulation 2
        """
        x = radius * np.cos(omega * t)
        y = radius * np.sin(omega * t)
        z = 2.0 - radius * np.cos(omega * t)
        
        x_dot = -radius * omega * np.sin(omega * t)
        y_dot = radius * omega * np.cos(omega * t)
        z_dot = radius * omega * np.sin(omega * t)
        
        x_ddot = -radius * omega**2 * np.cos(omega * t)
        y_ddot = -radius * omega**2 * np.sin(omega * t)
        z_ddot = radius * omega**2 * np.cos(omega * t)
        
        psi = 0.0
        psi_dot = 0.0
        psi_ddot = 0.0
        
        return {
            'pos': np.array([x, y, z]),
            'vel': np.array([x_dot, y_dot, z_dot]),
            'acc': np.array([x_ddot, y_ddot, z_ddot]),
            'yaw': psi,
            'yaw_dot': psi_dot,
            'yaw_ddot': psi_ddot
        }
    
    @staticmethod
    def square(t: float) -> Dict[str, np.ndarray]:
        """
        Generate square trajectory with altitude change
        From paper Table 3 - Simulation 1
        """
        waypoints = [
            [0.6, 0.6, 0.6, 0],
            [0.3, 0.6, 0.6, 10],
            [0.3, 0.3, 0.6, 20],
            [0.6, 0.3, 0.6, 30],
            [0.6, 0.6, 0.6, 40],
            [0.6, 0.6, 0.0, 50]
        ]
        
        # Find current segment
        pos = np.array([0.6, 0.6, 0.6])
        vel = np.zeros(3)
        acc = np.zeros(3)
        
        for i in range(len(waypoints) - 1):
            t_start = waypoints[i][3]
            t_end = waypoints[i + 1][3]
            
            if t_start <= t < t_end:
                p_start = np.array(waypoints[i][:3])
                p_end = np.array(waypoints[i + 1][:3])
                
                # Linear interpolation
                tau = (t - t_start) / (t_end - t_start)
                pos = p_start + tau * (p_end - p_start)
                vel = (p_end - p_start) / (t_end - t_start)
                acc = np.zeros(3)
                break
        
        # If beyond last waypoint, hold position
        if t >= waypoints[-1][3]:
            pos = np.array(waypoints[-1][:3])
            vel = np.zeros(3)
            acc = np.zeros(3)
        
        return {
            'pos': pos,
            'vel': vel,
            'acc': acc,
            'yaw': 0.5 if t < 50 else 0.0,
            'yaw_dot': 0.0,
            'yaw_ddot': 0.0
        }
    
    @staticmethod
    def complex_trajectory(t: float) -> Dict[str, np.ndarray]:
        """
        Generate complex trajectory with multiple segments
        From paper Eqs. (74)-(75) - Simulation 3
        """
        # X trajectory (Eq. 74)
        if 0 <= t < 4*np.pi:
            x = 0.5 * np.cos(t / 2)
            x_dot = -0.25 * np.sin(t / 2)
            x_ddot = -0.125 * np.cos(t / 2)
        elif 4*np.pi <= t < 20:
            x = 0.5
            x_dot = 0.0
            x_ddot = 0.0
        elif 20 <= t < 30:
            x = 0.25*t - 4.5
            x_dot = 0.25
            x_ddot = 0.0
        else:  # t >= 30
            x = 3.0
            x_dot = 0.0
            x_ddot = 0.0
        
        # Y trajectory (Eq. 74)
        if 0 <= t < 4*np.pi:
            y = 0.5 * np.sin(t / 2)
            y_dot = 0.25 * np.cos(t / 2)
            y_ddot = -0.125 * np.sin(t / 2)
        elif 4*np.pi <= t < 20:
            y = 0.25*t - 3.14
            y_dot = 0.25
            y_ddot = 0.0
        elif 20 <= t < 30:
            y = 5 - np.pi
            y_dot = 0.0
            y_ddot = 0.0
        elif 30 <= t < 40:
            y = -0.2358*t + 8.94
            y_dot = -0.2358
            y_ddot = 0.0
        else:  # t >= 40
            y = -0.5
            y_dot = 0.0
            y_ddot = 0.0
        
        # Z trajectory (Eq. 75)
        if 0 <= t < 4*np.pi:
            z = 0.125*t + 1
            z_dot = 0.125
            z_ddot = 0.0
        elif 4*np.pi <= t < 40:
            z = 0.5*np.pi + 1
            z_dot = 0.0
            z_ddot = 0.0
        else:  # t >= 40
            z = np.exp(-0.2*t + 8.944)
            z_dot = -0.2 * np.exp(-0.2*t + 8.944)
            z_ddot = 0.04 * np.exp(-0.2*t + 8.944)
        
        return {
            'pos': np.array([x, y, z]),
            'vel': np.array([x_dot, y_dot, z_dot]),
            'acc': np.array([x_ddot, y_ddot, z_ddot]),
            'yaw': 0.0,
            'yaw_dot': 0.0,
            'yaw_ddot': 0.0
        }
    
    @staticmethod
    def space_eight(t: float) -> Dict[str, np.ndarray]:
        """
        From paper Eqs. (77)-(78) - Simulation 5
        """
        # X trajectory (Eq. 77)
        if 0 <= t < 55:
            x = 0.0
            x_dot = 0.0
            x_ddot = 0.0
        else:
            omega = np.pi / 6
            x = 0.3 * np.cos(omega * t)
            x_dot = -0.3 * omega * np.sin(omega * t)
            x_ddot = -0.3 * omega**2 * np.cos(omega * t)
        
        # Y trajectory (Eq. 77)
        if 0 <= t < 55:
            y = 0.0
            y_dot = 0.0
            y_ddot = 0.0
        else:
            omega = np.pi / 6
            y = 0.3 * np.sin(omega * t)
            y_dot = 0.3 * omega * np.cos(omega * t)
            y_ddot = -0.3 * omega**2 * np.sin(omega * t)
        
        # Z trajectory (Eq. 78)
        if 0 <= t < 42:
            z = 0.5
            z_dot = 0.0
            z_ddot = 0.0
        elif 42 <= t < 87:
            z = 0.7
            z_dot = 0.0
            z_ddot = 0.0
        else:
            z = 0.8
            z_dot = 0.0
            z_ddot = 0.0
        
        return {
            'pos': np.array([x, y, z]),
            'vel': np.array([x_dot, y_dot, z_dot]),
            'acc': np.array([x_ddot, y_ddot, z_ddot]),
            'yaw': 0.0,
            'yaw_dot': 0.0,
            'yaw_ddot': 0.0
        }


class DisturbanceGenerator:
    """Generates external disturbances from paper"""
    
    @staticmethod
    def constant(t: float, scenario: str = "scenario2") -> Dict[str, np.ndarray]:
        """
        Constant disturbances from paper Section 4.2 (Simulation 2)
        Eq. (70)-(72)
        """
        d_pos = np.zeros(3)
        d_att = np.zeros(3)
        
        if scenario == "scenario2":
            # Position disturbances
            if t >= 5.0:
                d_pos[0] = 1.0  # dX
            if t >= 15.0:
                d_pos[1] = 1.0  # dY
            if t >= 25.0:
                d_pos[2] = 1.0  # dZ
            
            # Attitude disturbances
            if t >= 10.0:
                d_att[0] = 1.0  # dPhi
            if t >= 20.0:
                d_att[1] = 1.0  # dTheta
            if t >= 30.0:
                d_att[2] = 1.0  # dPsi
        
        return {'position': d_pos, 'attitude': d_att}
    
    @staticmethod
    def time_varying(t: float, scenario: str = "scenario3") -> Dict[str, np.ndarray]:
        """
        Time-varying disturbances from paper Section 4.3 (Simulation 3)
        Eq. (73)
        """
        d_pos = np.zeros(3)
        d_att = np.zeros(3)
        
        if scenario == "scenario3":
            # Position disturbances
            if 10.0 <= t <= 30.0:
                d_pos[0] = -(0.8*np.sin(0.1013*t - 3.0403) + 
                            0.4*np.sin(0.4488*t - 13.464) +
                            0.08*np.sin(1.5708*t - 15*np.pi) +
                            0.056*np.sin(0.2856*t - 8.568))
            
            if 10.0 <= t <= 50.0:
                d_pos[1] = 0.5*np.sin(0.4*t) + 0.5*np.cos(0.7*t)
            
            d_pos[2] = 0.5*np.cos(0.7*t)
            
            # Attitude disturbances
            d_att[0] = 0.5*np.cos(0.4*t)  # dPhi
            d_att[1] = 0.5*np.sin(0.5*t)  # dTheta
            d_att[2] = 0.5*np.sin(0.7*t)  # dPsi
        
        elif scenario == "scenario5":
            # More aggressive disturbances from Simulation 5 (Eq. 76)
            if 10.0 <= t <= 30.0:
                d_pos[0] = -(0.8*np.sin(0.1013*t - 3.0403) + 
                            0.4*np.sin(0.4488*t - 13.464) +
                            0.08*np.sin(1.5708*t - 15*np.pi) +
                            0.056*np.sin(0.2856*t - 8.568))
            
            if 10.0 <= t <= 50.0:
                d_pos[1] = 0.5*np.sin(0.4*t) + 0.5*np.cos(0.7*t)
            
            d_pos[2] = 0.5*np.cos(0.7*t) + 0.7*np.sin(0.3*t)
            
            d_att[0] = 0.5*np.cos(0.4*t) + 1.0
            d_att[1] = 0.5*np.sin(0.5*t) + 1.0
            d_att[2] = 0.5*np.sin(0.7*t) + 1.0
        
        return {'position': d_pos, 'attitude': d_att}
    
    @staticmethod
    def none(t: float) -> Dict[str, np.ndarray]:
        """No disturbances"""
        return {
            'position': np.zeros(3),
            'attitude': np.zeros(3)
        }