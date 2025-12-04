#!/usr/bin/env python3
"""
Main Flight Control Script - AXIS-SPECIFIC VERSION
Flexible control with independent parameter tuning per axis

FEATURES:
- Trajectory: Proper workspace bounding
- All axes: Axis-specific RANFTSMC control (separate params for X, Y, Z)
- Safety: Relaxed limits to prevent false triggers
"""
import numpy as np
import time
import argparse
from Params import QuadrotorParams, ControllerParams, SafetyParams
from Controller import RANFTSMController
from TelloInterface import TelloInterface
from Trajectory import ComplexTrajectory, HoveringTrajectory, CircularTrajectory
from PerformanceMetrics import PerformanceMetrics


def main():
    parser = argparse.ArgumentParser(description='Tello RANFTSMC Flight Controller - AXIS-SPECIFIC')
    parser.add_argument('--trajectory', type=str, default='complex',
                       choices=['complex', 'hover', 'circle'],
                       help='Trajectory type')
    parser.add_argument('--workspace-size', type=float, default=0.5,
                       help='Workspace size in meters (0.15-1.0) - entire trajectory fits in this cube')
    parser.add_argument('--duration', type=float, default=None,
                       help='Flight duration (seconds)')
    parser.add_argument('--debug', action='store_true',
                       help='Enable debug output')
    parser.add_argument('--save-data', action='store_true', default=True,
                       help='Save flight data')

    args = parser.parse_args()

    # Validate workspace size
    if not (0.15 <= args.workspace_size <= 1.0):
        print("ERROR: workspace-size must be between 0.15 and 1.0")
        return

    qp = QuadrotorParams()
    cp = ControllerParams()
    sp = SafetyParams()

    controller = RANFTSMController(qp, cp, sp, debug=args.debug)

    tello = TelloInterface(enable_logging=True, debug=args.debug)

    print("\n" + "="*70)
    print("TELLO RANFTSMC CONTROLLER - AXIS-SPECIFIC VERSION")
    print("="*70)

    if not tello.connect():
        print("Failed to connect to Tello")
        return

    if not tello.takeoff():
        print("Takeoff failed")
        tello.disconnect()
        return

    print("Stabilizing for 3 seconds...")
    time.sleep(3)

    actual_position = tello.get_position()
    print(f"Actual position after takeoff: {actual_position}")

    if args.trajectory == 'complex':
        trajectory = ComplexTrajectory(
            workspace_size=args.workspace_size,
            initial_position=actual_position,
            time_scale=1.0
        )
    elif args.trajectory == 'hover':
        trajectory = HoveringTrajectory(hover_position=actual_position)
        print("\nHovering Trajectory:")
    else:
        trajectory = CircularTrajectory(
            radius=0.3 * args.workspace_size,
            center=actual_position,
            omega=0.3
        )
        print(f"\nCircular Trajectory (radius={0.3*args.workspace_size:.2f}m):")

    trajectory.set_initial_position(actual_position)

    # Get workspace info
    if hasattr(trajectory, 'get_workspace_dimensions'):
        workspace = trajectory.get_workspace_dimensions()
        print("\nWorkspace dimensions:")
        for key, value in workspace.items():
            if isinstance(value, float):
                print(f"  {key}: {value:.3f}m")

    if args.duration is None:
        duration = trajectory.get_duration()
    else:
        duration = args.duration

    print(f"\nFlight duration: {duration:.1f}s")
    print(f"Control rate: 20 Hz")
    print(f"X velocity scaling: {cp.vel_scale_x}x")
    print(f"Y velocity scaling: {cp.vel_scale_y}x")
    print(f"Z velocity scaling: {cp.vel_scale_z}x")
    print("\nStarting trajectory tracking...")
    print("="*70 + "\n")

    time_data = []
    states_data = []
    desired_states_data = []
    commands_data = []
    errors_data = []

    # Control loop
    dt = 0.0125 # 0.016 # 0.025 # 0.05
    t = 0.0
    iteration = 0

    start_time = time.time()

    try:
        while t < duration:
            loop_start = time.time()

            state = tello.get_state()

            desired = trajectory.get_desired_state(t)

            cmd = controller.compute_control(state, desired, dt)

            tello.send_velocity_command(
                cmd['vx'],
                cmd['vy'],
                cmd['vz'],
                cmd['yaw_rate']
            )

            if args.save_data:
                time_data.append(t)
                states_data.append(state.copy())
                desired_states_data.append(np.array([
                    desired['pos'][0], desired['pos'][1], desired['pos'][2],
                    desired['vel'][0], desired['vel'][1], desired['vel'][2],
                    0.0, 0.0, desired['yaw'],
                    0.0, 0.0, desired['yaw_dot']
                ]))
                commands_data.append(np.array([
                    cmd['vx'], cmd['vy'], cmd['vz'], cmd['yaw_rate']
                ]))

                error = state[0:3] - desired['pos']
                errors_data.append(error)

            if iteration % 20 == 0:
                error = state[0:3] - desired['pos']
                error_norm = np.linalg.norm(error)
                battery = tello.get_battery()
                print(f"t={t:5.1f}s | e_total={error_norm*100:5.1f}cm | "
                      f"e_x={error[0]*100:+5.1f}cm | e_y={error[1]*100:+5.1f}cm | "
                      f"e_z={error[2]*100:+5.1f}cm | bat={battery}%")

            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

            t += dt
            iteration += 1

    except KeyboardInterrupt:
        print("\n\nFlight interrupted by user")

    except Exception as e:
        print(f"\n\nError during flight: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("\n" + "="*70)
        print("Landing...")
        tello.land()
        tello.disconnect()

        if args.save_data and len(time_data) > 0:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"flight_data_hybrid_{timestamp}.npz"

            np.savez(
                filename,
                time=np.array(time_data),
                states=np.array(states_data),
                states_desired=np.array(desired_states_data),
                commands=np.array(commands_data),
                errors=np.array(errors_data),
                controller_type='RANFTSMC_HYBRID',
                workspace_size=args.workspace_size
            )

            print(f"\nFlight data saved: {filename}")

            print("\nCalculating performance metrics...")
            metrics = PerformanceMetrics.calculate_metrics(
                np.array(time_data),
                np.array(states_data),
                np.array(desired_states_data),
                verbose=True
            )

        print("="*70)
        print("\nAXIS-SPECIFIC VERSION SUMMARY:")
        print("  All axes (XYZ): Axis-specific RANFTSMC control")
        print("  Separate parameters for X, Y, Z axes")
        print("  Independent tuning for optimal performance per axis")
        print("="*70)


if __name__ == "__main__":
    main()
