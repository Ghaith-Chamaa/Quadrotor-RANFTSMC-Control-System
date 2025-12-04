"""
Tello Hardware Interface - HYBRID VERSION
Based on exp v2 with safety checks removed

KEY FEATURES:
1. Minimal velocity filtering (good for XY tracking)
2. Dead reckoning with barometer for Z
3. Comprehensive debug logging
4. Safety checks REMOVED per user request
"""
import numpy as np
import time
from djitellopy import Tello
import logging
from typing import Optional, Dict


class TelloInterface:
    """Interface between controller and Tello hardware - HYBRID VERSION"""

    def __init__(self, enable_logging: bool = True, debug: bool = False):
        self.tello = Tello()
        self.connected = False
        self.in_flight = False
        self.debug = debug

        # State vector: [x, y, z, vx, vy, vz, φ, θ, ψ, p, q, r]
        self.state = np.zeros(12)

        # Position tracking
        self.position = np.array([0.0, 0.0, 0.0])
        self.takeoff_position = None

        self.prev_attitude = np.zeros(3)
        self.prev_height = 0.0
        self.last_update_time = 0.0

        self.vel_history = []
        self.vel_history_size = 1

        self.MAX_VEL_CMD = 100

        if enable_logging:
            logging.basicConfig(level=logging.DEBUG if debug else logging.INFO,
                              format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            self.logger = logging.getLogger('TelloInterface')
        else:
            self.logger = None

        self.iteration = 0

    def connect(self) -> bool:
        """Connect to Tello"""
        try:
            if self.logger:
                self.logger.info("Connecting to Tello...")

            self.tello.connect()
            time.sleep(2)

            battery = self.tello.get_battery()
            if self.logger:
                self.logger.info(f"Connected! Battery: {battery}%")

            if battery < 15:
                if self.logger:
                    self.logger.error(f"Battery too low: {battery}%")
                return False

            self.connected = True
            return True

        except Exception as e:
            if self.logger:
                self.logger.error(f"Connection failed: {e}")
            return False

    def takeoff(self) -> bool:
        """Takeoff and initialize position"""
        if not self.connected:
            return False

        try:
            if self.logger:
                self.logger.info("Taking off...")

            self.tello.takeoff()
            time.sleep(4)

            initial_height = self.tello.get_height() / 100.0

            self.position = np.array([0.0, 0.0, initial_height])
            self.takeoff_position = self.position.copy()

            self.state[0:3] = self.position
            self.state[2] = initial_height

            self.last_update_time = time.time()
            self.in_flight = True

            if self.logger:
                self.logger.info(f"Takeoff complete at height: {initial_height:.2f}m")
            return True

        except Exception as e:
            if self.logger:
                self.logger.error(f"Takeoff failed: {e}")
            return False

    def land(self) -> bool:
        """Land the drone"""
        try:
            if self.logger:
                self.logger.info("Landing...")

            self.tello.land()
            time.sleep(3)
            self.in_flight = False

            self.position = np.array([0.0, 0.0, 0.0])

            if self.logger:
                self.logger.info("Landed successfully")
            return True

        except Exception as e:
            if self.logger:
                self.logger.error(f"Landing failed: {e}")
            return False

    def emergency_stop(self):
        """Emergency stop"""
        if self.logger:
            self.logger.error("EMERGENCY STOP!")
        try:
            self.tello.emergency()
        except:
            pass
        self.in_flight = False

    def _get_tello_measurements(self) -> Optional[Dict]:
        """Get all measurements from Tello sensors"""
        try:
            measurements = {
                'height': self.tello.get_height() / 100.0,
                'roll': np.deg2rad(self.tello.get_roll()),
                'pitch': np.deg2rad(self.tello.get_pitch()),
                'yaw': np.deg2rad(self.tello.get_yaw()),
                'speed_x': self.tello.get_speed_x() / 100.0,  # cm/s to m/s
                'speed_y': self.tello.get_speed_y() / 100.0,
                'speed_z': self.tello.get_speed_z() / 100.0,
                'battery': self.tello.get_battery(),
                'flight_time': self.tello.get_flight_time()
            }
            return measurements
        except Exception as e:
            if self.logger:
                self.logger.warning(f"Measurement error: {e}")
            return None

    def get_state(self) -> np.ndarray:
        """
        Get current state estimate

        Returns:
            State vector [x, y, z, vx, vy, vz, φ, θ, ψ, p, q, r]
        """
        current_time = time.time()
        dt = current_time - self.last_update_time

        meas = self._get_tello_measurements()
        if meas is None:
            return self.state.copy()

        height = meas['height']
        roll = meas['roll']
        pitch = meas['pitch']
        yaw = meas['yaw']
        vx_body = meas['speed_x']
        vy_body = meas['speed_y']
        vz_body = meas['speed_z']

        # Transform velocities from body to world frame
        c_yaw = np.cos(yaw)
        s_yaw = np.sin(yaw)

        vx_world = vx_body * c_yaw - vy_body * s_yaw
        vy_world = vx_body * s_yaw + vy_body * c_yaw
        vz_world = vz_body

        current_vel = np.array([vx_world, vy_world, vz_world])
        self.vel_history.append(current_vel)
        if len(self.vel_history) > self.vel_history_size:
            self.vel_history.pop(0)
        vel_filtered = np.mean(self.vel_history, axis=0) if self.vel_history else current_vel

        # Update position using dead reckoning
        if 0 < dt < 1.0:
            self.position[0] += vel_filtered[0] * dt
            self.position[1] += vel_filtered[1] * dt
            self.position[2] = height  # Use barometer for altitude

        # Estimate angular rates
        if dt > 0 and dt < 1.0:
            p_est = (roll - self.prev_attitude[0]) / dt
            q_est = (pitch - self.prev_attitude[1]) / dt

            yaw_diff = yaw - self.prev_attitude[2]
            yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
            r_est = yaw_diff / dt
        else:
            p_est, q_est, r_est = 0.0, 0.0, 0.0

        # Construct state vector
        self.state[0:3] = self.position
        self.state[3:6] = vel_filtered
        self.state[6:9] = [roll, pitch, yaw]
        self.state[9:12] = [p_est, q_est, r_est]

        if self.debug and self.iteration % 20 == 0:
            if self.logger:
                self.logger.debug(
                    f"[{self.iteration:4d}] Tello raw: vx={vx_body:.4f}, vy={vy_body:.4f}, vz={vz_body:.4f} | "
                    f"World: vx={vx_world:.4f}, vy={vy_world:.4f} | "
                    f"Pos: [{self.position[0]:.3f}, {self.position[1]:.3f}, {self.position[2]:.3f}]"
                )

        # Update history
        self.prev_attitude = np.array([roll, pitch, yaw])
        self.prev_height = height
        self.last_update_time = current_time
        self.iteration += 1

        return self.state.copy()

    def send_velocity_command(self, vx: float, vy: float, vz: float, yaw_rate: float):
        """
        Send velocity command to Tello

        Args:
            vx, vy, vz: Velocities in WORLD frame (m/s)
            yaw_rate: Yaw rate (rad/s)
        """
        if not self.in_flight:
            return

        # Transform to body frame
        yaw = self.state[8]
        c_yaw = np.cos(yaw)
        s_yaw = np.sin(yaw)

        vx_body = vx * c_yaw + vy * s_yaw
        vy_body = -vx * s_yaw + vy * c_yaw
        vz_body = vz

        # Convert to Tello units (cm/s)
        vx_cmd = int(np.clip(vx_body * 100, -self.MAX_VEL_CMD, self.MAX_VEL_CMD))
        vy_cmd = int(np.clip(vy_body * 100, -self.MAX_VEL_CMD, self.MAX_VEL_CMD))
        vz_cmd = int(np.clip(vz_body * 100, -self.MAX_VEL_CMD, self.MAX_VEL_CMD))
        yaw_rate_cmd = int(np.clip(np.rad2deg(yaw_rate), -100, 100))

        # Debug output
        if self.debug and self.iteration % 20 == 0:
            if self.logger:
                self.logger.debug(
                    f"CMD: World=[{vx:.3f},{vy:.3f},{vz:.3f}] → "
                    f"Body=[{vx_body:.3f},{vy_body:.3f},{vz_body:.3f}] → "
                    f"RC=[{vx_cmd},{vy_cmd},{vz_cmd}] cm/s"
                )

        try:
            self.tello.send_rc_control(vy_cmd, vx_cmd, vz_cmd, yaw_rate_cmd)
        except Exception as e:
            if self.logger:
                self.logger.error(f"Command send failed: {e}")

    def get_position(self) -> np.ndarray:
        """Get current position"""
        return self.state[0:3].copy()

    def get_attitude(self) -> np.ndarray:
        """Get current attitude"""
        return self.state[6:9].copy()

    def get_battery(self) -> int:
        """Get battery percentage"""
        try:
            return self.tello.get_battery()
        except:
            return -1

    def disconnect(self):
        """Disconnect from Tello"""
        if self.in_flight:
            self.land()

        try:
            try:
                self.tello.disable_mission_pads()
            except:
                pass

            self.tello.end()
            if self.logger:
                self.logger.info("Disconnected from Tello")
        except:
            pass

        self.connected = False
