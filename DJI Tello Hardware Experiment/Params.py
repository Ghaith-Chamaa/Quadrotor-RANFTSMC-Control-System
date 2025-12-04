"""
Controller Parameters - HYBRID VERSION
Combines best parameters from both codebases

UPDATED STRATEGY:
1. All axes use direct velocity control
2. XY-axis: Parameters from exp v2 (excellent XY tracking)
3. Z-axis: Parameters from revamped v2 (excellent Z tracking)
4. Safety limits: Relaxed to prevent false triggers
"""
from dataclasses import dataclass, field
import numpy as np


@dataclass
class QuadrotorParams:
    """Physical parameters of the DJI Ryze Tello quadrotor"""
    m: float = 0.087
    g: float = 9.81
    Jxx: float = 2.5e-4
    Jyy: float = 2.5e-4
    Jzz: float = 4.5e-4
    Jr: float = 5.0e-7
    Kx: float = 1.0e-4
    Ky: float = 1.0e-4
    Kz: float = 1.2e-4
    Kphi: float = 1.0e-5
    Ktheta: float = 1.0e-5
    Kpsi: float = 1.5e-5
    bd: float = 5.0e-4
    cd: float = 5.0e-3
    ld: float = 0.05
    max_tilt_angle: float = np.deg2rad(25)
    max_speed_horizontal: float = 8.0
    max_speed_vertical: float = 3.0

@dataclass
class ControllerParams:
    """RANFTSMC controller parameters - HYBRID VERSION"""
	
    bn_x: float = 0.08
    bn_plus1_x: float = 0.5
    cn_x: float = 1.7
    beta_n_x: float = 5/3
    alpha_n_x: float = 2.0
    hn_x: float = 1.0
    mu0n_x: float = 0.5
    mu1n_x: float = 0.001
    mu2n_x: float = 0.01

    bn_y: float = 0.08
    bn_plus1_y: float = 0.5
    cn_y: float = 1.7
    beta_n_y: float = 5/3
    alpha_n_y: float = 2.0
    hn_y: float = 1.0
    mu0n_y: float = 0.5
    mu1n_y: float = 0.001
    mu2n_y: float = 0.01

    bn_z: float = 0.065
    bn_plus1_z: float = 0.367
    cn_z: float = 0.95
    beta_n_z: float = 5/3
    alpha_n_z: float = 2.0
    hn_z: float = 0.45
    mu0n_z: float = 0.4
    mu1n_z: float = 0.0008
    mu2n_z: float = 0.008

    bj: float = 6.0
    bj_plus1: float = 0.4
    cj: float = 8.0
    beta_j: float = 1.0885
    alpha_j: float = 1.2
    hj: float = 0.5
    mu0j: float = 0.012
    mu1j: float = 5.0e-4
    mu2j: float = 0.008

    epsilon: float = 0.18

    vel_scale_x: float = 1.2
    vel_scale_y: float = 1.2
    vel_scale_z: float = 0.95
    yaw_rate_scale: float = 0.8


@dataclass
class SafetyParams:
    """
    Safety parameters - RELAXED to prevent false triggers
    User reported safety measures were falsely being activated
    """
    max_x: float = 3.0 
    max_y: float = 3.0 
    max_z: float = 2.5 
    min_z: float = 0.25
    workspace_radius: float = 3.5

    max_vel_horizontal: float = 1.5
    max_vel_vertical: float = 1.0  
    max_yaw_rate: float = 1.0      

    max_position_error: float = 1.0  
    emergency_stop_error: float = 2.5

    min_battery: float = 15.0
    
    ceiling_height: float = 2.5
    floor_height: float = 0.2
