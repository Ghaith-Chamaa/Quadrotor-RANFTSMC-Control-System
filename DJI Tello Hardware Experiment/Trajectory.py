"""
Complex Trajectory Generator with Proper Workspace Bounding
HYBRID VERSION - From revamped v2

KEY FEATURES:
1. Workspace size parameter ensures ENTIRE trajectory fits in specified volume
2. Maintains trajectory shape proportions from paper
3. Proper scaling based on largest dimension
4. All coordinates properly bounded within workspace
5. Starts from drone's initial position
"""
import numpy as np
from typing import Dict


class ComplexTrajectory:
    """
    Complex trajectory from Labbadi & Cherkaoui (2020) - Equations 74-75

    WORKSPACE SCALING:
    - Analyzes paper trajectory bounding box
    - Scales uniformly to fit within workspace_size³
    - workspace_size=1.0 → entire trajectory fits in 1m × 1m × 1m cube
    """

    # Reference dimensions from paper trajectory (Simulation 3)
    PAPER_X_MIN = -0.5
    PAPER_X_MAX = 3.0
    PAPER_Y_MIN = -0.5
    PAPER_Y_MAX = 5.0 - np.pi
    PAPER_Z_MIN = 1.0
    PAPER_Z_MAX = 0.5 * np.pi + 1.0

    # Calculate actual ranges
    PAPER_X_RANGE = PAPER_X_MAX - PAPER_X_MIN  # 3.5m
    PAPER_Y_RANGE = PAPER_Y_MAX - PAPER_Y_MIN  # 2.358m
    PAPER_Z_RANGE = PAPER_Z_MAX - PAPER_Z_MIN  # 1.571m

    def __init__(self,
                 workspace_size: float = 1.0,
                 initial_position: np.ndarray = None,
                 time_scale: float = 1.0):
        """
        Initialize trajectory with proper workspace bounding

        Args:
            workspace_size: Size of cubic workspace in meters (0.15 to 1.0)
                          The ENTIRE trajectory will fit within this cube
            initial_position: [x, y, z] starting position (m)
            time_scale: Time scaling factor (1.0 = normal speed)
        """
        # Find the largest dimension to ensure everything fits
        max_paper_range = max(self.PAPER_X_RANGE, self.PAPER_Y_RANGE, self.PAPER_Z_RANGE)

        # Uniform scaling to fit largest dimension in workspace
        self.scale_factor = workspace_size / max_paper_range

        # Apply same scale to maintain trajectory shape
        self.scale_x = self.scale_factor
        self.scale_y = self.scale_factor
        self.scale_z = self.scale_factor

        # Calculate actual scaled dimensions
        self.actual_x_range = self.PAPER_X_RANGE * self.scale_factor
        self.actual_y_range = self.PAPER_Y_RANGE * self.scale_factor
        self.actual_z_range = self.PAPER_Z_RANGE * self.scale_factor

        self.workspace_size = workspace_size
        self.time_scale = time_scale

        # Starting position
        if initial_position is not None:
            self.offset = initial_position.copy()
        else:
            self.offset = np.array([0.0, 0.0, 0.7])

        print(f"\n{'='*60}")
        print(f"TRAJECTORY CONFIGURATION")
        print(f"{'='*60}")
        print(f"Workspace size: {workspace_size:.2f}m (cubic)")
        print(f"Uniform scale factor: {self.scale_factor:.3f}")
        print(f"Paper bounding box: {self.PAPER_X_RANGE:.2f}m × {self.PAPER_Y_RANGE:.2f}m × {self.PAPER_Z_RANGE:.2f}m")
        print(f"Scaled bounding box: {self.actual_x_range:.2f}m × {self.actual_y_range:.2f}m × {self.actual_z_range:.2f}m")
        print(f"X range: [{self.offset[0]:.2f}, {self.offset[0] + self.actual_x_range:.2f}]m")
        print(f"Y range: [{self.offset[1]:.2f}, {self.offset[1] + self.actual_y_range:.2f}]m")
        print(f"Z range: [{self.offset[2]:.2f}, {self.offset[2] + self.actual_z_range:.2f}]m")
        print(f"{'='*60}\n")

    def set_initial_position(self, position: np.ndarray):
        """Set trajectory starting position"""
        self.offset = position.copy()
        print(f"Trajectory offset updated: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
        print(f"New ranges:")
        print(f"  X: [{self.offset[0]:.2f}, {self.offset[0] + self.actual_x_range:.2f}]m")
        print(f"  Y: [{self.offset[1]:.2f}, {self.offset[1] + self.actual_y_range:.2f}]m")
        print(f"  Z: [{self.offset[2]:.2f}, {self.offset[2] + self.actual_z_range:.2f}]m")

    def get_desired_state(self, t: float) -> Dict[str, np.ndarray]:
        """
        Get desired state at time t
        Implements paper equations (74)-(75) with proper workspace bounding
        """
        # Time scaling
        t_s = t / self.time_scale

        # ==================== X TRAJECTORY (Equation 74) ====================
        if 0 <= t_s < 4*np.pi:  # Phase 1: Circular arc
            x_raw = 0.5 * np.cos(t_s / 2)
            x_dot_raw = -0.25 * np.sin(t_s / 2)
            x_ddot_raw = -0.125 * np.cos(t_s / 2)
        elif 4*np.pi <= t_s < 20:  # Phase 2: Constant
            x_raw = 0.5
            x_dot_raw = 0.0
            x_ddot_raw = 0.0
        elif 20 <= t_s < 30:  # Phase 3: Linear ramp
            x_raw = 0.25*t_s - 4.5
            x_dot_raw = 0.25
            x_ddot_raw = 0.0
        else:  # Phase 4: Constant final
            x_raw = 3.0
            x_dot_raw = 0.0
            x_ddot_raw = 0.0

        # ==================== Y TRAJECTORY (Equation 74) ====================
        if 0 <= t_s < 4*np.pi:  # Phase 1: Circular arc
            y_raw = 0.5 * np.sin(t_s / 2)
            y_dot_raw = 0.25 * np.cos(t_s / 2)
            y_ddot_raw = -0.125 * np.sin(t_s / 2)
        elif 4*np.pi <= t_s < 20:  # Phase 2: Linear ramp
            y_raw = 0.25*t_s - 3.14
            y_dot_raw = 0.25
            y_ddot_raw = 0.0
        elif 20 <= t_s < 30:  # Phase 3: Constant
            y_raw = 5 - np.pi  # ≈ 1.858
            y_dot_raw = 0.0
            y_ddot_raw = 0.0
        elif 30 <= t_s < 40:  # Phase 4: Negative ramp
            y_raw = -0.2358*t_s + 8.94
            y_dot_raw = -0.2358
            y_ddot_raw = 0.0
        else:  # Phase 5: Constant final
            y_raw = -0.5
            y_dot_raw = 0.0
            y_ddot_raw = 0.0

        # ==================== Z TRAJECTORY (Equation 75) ====================
        if 0 <= t_s < 4*np.pi:  # Phase 1: Linear ascent
            z_raw = 0.125*t_s + 1
            z_dot_raw = 0.125
            z_ddot_raw = 0.0
        elif 4*np.pi <= t_s < 40:  # Phase 2: Constant altitude
            z_raw = 0.5*np.pi + 1  # ≈ 2.571
            z_dot_raw = 0.0
            z_ddot_raw = 0.0
        else:  # Phase 3: Exponential descent
            z_raw = np.exp(-0.2*t_s + 8.944)
            z_dot_raw = -0.2 * np.exp(-0.2*t_s + 8.944)
            z_ddot_raw = 0.04 * np.exp(-0.2*t_s + 8.944)

        # SCALE AND SHIFT TO FIT WORKSPACE
        # Shift raw coordinates so minimum is at 0, then scale
        x_normalized = (x_raw - self.PAPER_X_MIN) * self.scale_x
        y_normalized = (y_raw - self.PAPER_Y_MIN) * self.scale_y
        z_normalized = (z_raw - self.PAPER_Z_MIN) * self.scale_z

        # Add offset to position in workspace
        x = self.offset[0] + x_normalized
        y = self.offset[1] + y_normalized
        z = self.offset[2] + z_normalized

        # Velocity (scaled by space and time)
        x_dot = x_dot_raw * self.scale_x / self.time_scale
        y_dot = y_dot_raw * self.scale_y / self.time_scale
        z_dot = z_dot_raw * self.scale_z / self.time_scale

        # Acceleration (scaled by space and time²)
        x_ddot = x_ddot_raw * self.scale_x / (self.time_scale**2)
        y_ddot = y_ddot_raw * self.scale_y / (self.time_scale**2)
        z_ddot = z_ddot_raw * self.scale_z / (self.time_scale**2)

        pos = np.array([x, y, z])
        vel = np.array([x_dot, y_dot, z_dot])
        acc = np.array([x_ddot, y_ddot, z_ddot])

        return {
            'pos': pos,
            'vel': vel,
            'acc': acc,
            'yaw': 0.0,
            'yaw_dot': 0.0,
            'yaw_ddot': 0.0
        }

    def get_duration(self) -> float:
        """Get total trajectory duration"""
        return 60.0 * self.time_scale

    def get_workspace_dimensions(self) -> Dict[str, float]:
        """Get actual workspace dimensions"""
        return {
            'x_range': self.actual_x_range,
            'y_range': self.actual_y_range,
            'z_range': self.actual_z_range,
            'x_min': self.offset[0],
            'x_max': self.offset[0] + self.actual_x_range,
            'y_min': self.offset[1],
            'y_max': self.offset[1] + self.actual_y_range,
            'z_min': self.offset[2],
            'z_max': self.offset[2] + self.actual_z_range,
            'workspace_size': self.workspace_size
        }


class HoveringTrajectory:
    """Simple hovering trajectory"""

    def __init__(self, hover_position: np.ndarray = None):
        if hover_position is None:
            self.hover_position = np.array([0.0, 0.0, 0.7])
        else:
            self.hover_position = hover_position.copy()

    def set_initial_position(self, position: np.ndarray):
        self.hover_position = position.copy()

    def get_desired_state(self, t: float) -> Dict[str, np.ndarray]:
        return {
            'pos': self.hover_position.copy(),
            'vel': np.zeros(3),
            'acc': np.zeros(3),
            'yaw': 0.0,
            'yaw_dot': 0.0,
            'yaw_ddot': 0.0
        }

    def get_duration(self) -> float:
        return 30.0

    def get_workspace_dimensions(self) -> Dict[str, float]:
        return {
            'x_range': 0.0,
            'y_range': 0.0,
            'z_range': 0.0,
            'workspace_size': 0.0
        }


class CircularTrajectory:
    """Circular trajectory for testing"""

    def __init__(self, radius: float = 0.3, center: np.ndarray = None, omega: float = 0.2):
        self.radius = radius
        self.omega = omega
        if center is None:
            self.center = np.array([0.0, 0.0, 0.7])
        else:
            self.center = center.copy()

    def set_initial_position(self, position: np.ndarray):
        self.center = position.copy()

    def get_desired_state(self, t: float) -> Dict[str, np.ndarray]:
        x = self.center[0] + self.radius * np.cos(self.omega * t)
        y = self.center[1] + self.radius * np.sin(self.omega * t)
        z = self.center[2]

        x_dot = -self.radius * self.omega * np.sin(self.omega * t)
        y_dot = self.radius * self.omega * np.cos(self.omega * t)
        z_dot = 0.0

        x_ddot = -self.radius * self.omega**2 * np.cos(self.omega * t)
        y_ddot = -self.radius * self.omega**2 * np.sin(self.omega * t)
        z_ddot = 0.0

        return {
            'pos': np.array([x, y, z]),
            'vel': np.array([x_dot, y_dot, z_dot]),
            'acc': np.array([x_ddot, y_ddot, z_ddot]),
            'yaw': 0.0,
            'yaw_dot': 0.0,
            'yaw_ddot': 0.0
        }

    def get_duration(self) -> float:
        return 2 * np.pi / self.omega

    def get_workspace_dimensions(self) -> Dict[str, float]:
        return {
            'x_range': 2 * self.radius,
            'y_range': 2 * self.radius,
            'z_range': 0.0,
            'workspace_size': 2 * self.radius
        }
