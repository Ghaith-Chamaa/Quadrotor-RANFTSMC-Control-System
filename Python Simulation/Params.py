from dataclasses import dataclass

@dataclass
class QuadrotorParams:
    """Physical parameters of the quadrotor"""
    m: float = 0.74  # Mass (kg)
    g: float = 9.81  # Gravity (m/s^2)
    Jxx: float = 0.004  # Moment of inertia (kg.m^2)
    Jyy: float = 0.004
    Jzz: float = 0.0084
    Jr: float = 2.8385e-5  # Rotor inertia
    Kx: float = 5.567e-4  # Aerodynamic friction coefficients
    Ky: float = 5.567e-4
    Kz: float = 5.567e-4
    Kphi: float = 5.567e-4
    Ktheta: float = 5.567e-4
    Kpsi: float = 5.567e-4
    bd: float = 2.9842e-3  # Thrust coefficient
    cd: float = 3.232e-2  # Drag coefficient
    ld: float = 0.2  # Arm length (m)


@dataclass
class ControllerParams:
    """RANFTSMC controller parameters from paper Table 2"""
    # Position control parameters
    bn: float = 0.03
    bn_plus1: float = 0.8
    cn: float = 1.21
    beta_n: float = 5/3
    alpha_n: float = 2.0
    # hn: float = 0.5
    hn: float = 1.5
    mu0n: float = 0.5
    mu1n: float = 0.001
    mu2n: float = 0.01
    
    # Attitude control parameters
    bj: float = 8.7406
    bj_plus1: float = 0.4838
    cj: float = 11.7180
    beta_j: float = 1.0885
    alpha_j: float = 1.2
    # hj: float = 0.5
    hj: float = 1.5
    mu0j: float = 0.0118
    mu1j: float = 6.2942e-4
    mu2j: float = 0.01
