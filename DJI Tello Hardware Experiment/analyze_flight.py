#!/usr/bin/env python3
"""
Flight Data Analysis Tool
Analyze and visualize saved Tello flight data
"""
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from PerformanceMetrics import PerformanceMetrics

def load_flight_data(filename):
    """Load flight data from npz file"""
    try:
        data = np.load(filename, allow_pickle=True)
        return {
            'time': data['time'],
            'states': data['states'],
            'states_desired': data['states_desired'],
            'commands': data['commands'],
            'controller_type': str(data['controller_type']) if 'controller_type' in data else 'unknown'
        }
    except Exception as e:
        print(f"Error loading file: {e}")
        return None

def plot_comprehensive_analysis(data):
    """Create comprehensive analysis plots"""
    t = data['time']
    states = data['states']
    states_desired = data['states_desired']
    commands = data['commands']
    
    fig = plt.figure(figsize=(18, 12))
    gs = GridSpec(4, 4, figure=fig, hspace=0.3, wspace=0.3)
    
    ax_3d = fig.add_subplot(gs[0:2, 0:2], projection='3d')
    ax_3d.plot(states_desired[:, 0], states_desired[:, 1], states_desired[:, 2],
              'b--', linewidth=2, label='Desired', alpha=0.7)
    ax_3d.plot(states[:, 0], states[:, 1], states[:, 2],
              'r-', linewidth=1.5, label='Actual', alpha=0.9)
    ax_3d.scatter(states[0, 0], states[0, 1], states[0, 2], 
                 c='g', s=100, marker='o', label='Start')
    ax_3d.scatter(states[-1, 0], states[-1, 1], states[-1, 2],
                 c='m', s=100, marker='s', label='End')
    ax_3d.set_xlabel('X (m)')
    ax_3d.set_ylabel('Y (m)')
    ax_3d.set_zlabel('Z (m)')
    ax_3d.set_title('3D Trajectory', fontweight='bold')
    ax_3d.legend()
    ax_3d.grid(True, alpha=0.3)
    
    ax_xy = fig.add_subplot(gs[0, 2])
    ax_xy.plot(states_desired[:, 0], states_desired[:, 1], 'b--', linewidth=2, alpha=0.7)
    ax_xy.plot(states[:, 0], states[:, 1], 'r-', linewidth=1.5, alpha=0.9)
    ax_xy.scatter(states[0, 0], states[0, 1], c='g', s=50, marker='o')
    ax_xy.scatter(states[-1, 0], states[-1, 1], c='m', s=50, marker='s')
    ax_xy.set_xlabel('X (m)')
    ax_xy.set_ylabel('Y (m)')
    ax_xy.set_title('Top View (X-Y)')
    ax_xy.grid(True, alpha=0.3)
    ax_xy.axis('equal')
    
    ax_xz = fig.add_subplot(gs[1, 2])
    ax_xz.plot(states_desired[:, 0], states_desired[:, 2], 'b--', linewidth=2, alpha=0.7)
    ax_xz.plot(states[:, 0], states[:, 2], 'r-', linewidth=1.5, alpha=0.9)
    ax_xz.set_xlabel('X (m)')
    ax_xz.set_ylabel('Z (m)')
    ax_xz.set_title('Side View (X-Z)')
    ax_xz.grid(True, alpha=0.3)
    
    ax_yz = fig.add_subplot(gs[0, 3])
    ax_yz.plot(states_desired[:, 1], states_desired[:, 2], 'b--', linewidth=2, alpha=0.7)
    ax_yz.plot(states[:, 1], states[:, 2], 'r-', linewidth=1.5, alpha=0.9)
    ax_yz.set_xlabel('Y (m)')
    ax_yz.set_ylabel('Z (m)')
    ax_yz.set_title('Side View (Y-Z)')
    ax_yz.grid(True, alpha=0.3)
    
    ax_err = fig.add_subplot(gs[1, 3])
    errors = states[:, 0:3] - states_desired[:, 0:3]
    error_norm = np.linalg.norm(errors, axis=1)
    ax_err.plot(t, error_norm, 'r-', linewidth=1.5)
    ax_err.axhline(y=0.1, color='orange', linestyle='--', linewidth=0.8, alpha=0.5, label='0.1m')
    ax_err.set_xlabel('Time (s)')
    ax_err.set_ylabel('Position Error (m)')
    ax_err.set_title('Tracking Error Magnitude')
    ax_err.legend()
    ax_err.grid(True, alpha=0.3)
    
    for i, (label, ax_pos) in enumerate(zip(['X', 'Y', 'Z'], 
                                            [fig.add_subplot(gs[2, j]) for j in range(3)])):
        ax_pos.plot(t, states_desired[:, i], 'b--', linewidth=2, label='Desired', alpha=0.7)
        ax_pos.plot(t, states[:, i], 'r-', linewidth=1.5, label='Actual', alpha=0.9)
        ax_pos.fill_between(t, states_desired[:, i], states[:, i], alpha=0.2, color='red')
        ax_pos.set_xlabel('Time (s)')
        ax_pos.set_ylabel(f'{label} (m)')
        ax_pos.set_title(f'Position {label}')
        ax_pos.legend(fontsize=8)
        ax_pos.grid(True, alpha=0.3)
    
    ax_err_comp = fig.add_subplot(gs[2, 3])
    ax_err_comp.plot(t, errors[:, 0], 'r-', label='e_x', alpha=0.7, linewidth=1.5)
    ax_err_comp.plot(t, errors[:, 1], 'g-', label='e_y', alpha=0.7, linewidth=1.5)
    ax_err_comp.plot(t, errors[:, 2], 'b-', label='e_z', alpha=0.7, linewidth=1.5)
    ax_err_comp.axhline(y=0, color='k', linestyle='--', linewidth=0.8)
    ax_err_comp.set_xlabel('Time (s)')
    ax_err_comp.set_ylabel('Error (m)')
    ax_err_comp.set_title('Position Error Components')
    ax_err_comp.legend()
    ax_err_comp.grid(True, alpha=0.3)
    
    ax_vcmd = fig.add_subplot(gs[3, 0])
    ax_vcmd.plot(t, commands[:, 0], 'r-', label='v_x', alpha=0.7)
    ax_vcmd.plot(t, commands[:, 1], 'g-', label='v_y', alpha=0.7)
    ax_vcmd.plot(t, commands[:, 2], 'b-', label='v_z', alpha=0.7)
    ax_vcmd.set_xlabel('Time (s)')
    ax_vcmd.set_ylabel('Velocity (m/s)')
    ax_vcmd.set_title('Velocity Commands')
    ax_vcmd.legend()
    ax_vcmd.grid(True, alpha=0.3)
    
    ax_vmag = fig.add_subplot(gs[3, 1])
    v_cmd_horz = np.sqrt(commands[:, 0]**2 + commands[:, 1]**2)
    v_cmd_vert = np.abs(commands[:, 2])
    ax_vmag.plot(t, v_cmd_horz, 'purple', label='Horizontal', linewidth=1.5)
    ax_vmag.plot(t, v_cmd_vert, 'orange', label='Vertical', linewidth=1.5)
    ax_vmag.axhline(y=0.8, color='purple', linestyle='--', linewidth=0.8, alpha=0.5)
    ax_vmag.axhline(y=0.5, color='orange', linestyle='--', linewidth=0.8, alpha=0.5)
    ax_vmag.set_xlabel('Time (s)')
    ax_vmag.set_ylabel('Speed (m/s)')
    ax_vmag.set_title('Velocity Magnitude')
    ax_vmag.legend()
    ax_vmag.grid(True, alpha=0.3)
    
    ax_yaw = fig.add_subplot(gs[3, 2])
    ax_yaw.plot(t, np.rad2deg(commands[:, 3]), 'b-', linewidth=1.5)
    ax_yaw.set_xlabel('Time (s)')
    ax_yaw.set_ylabel('Yaw Rate (°/s)')
    ax_yaw.set_title('Yaw Rate Command')
    ax_yaw.grid(True, alpha=0.3)
    
    ax_effort = fig.add_subplot(gs[3, 3])
    control_effort = np.sum(np.abs(commands[:, 0:3]), axis=1)
    ax_effort.plot(t, control_effort, 'purple', linewidth=1.5)
    ax_effort.fill_between(t, 0, control_effort, alpha=0.3, color='purple')
    ax_effort.set_xlabel('Time (s)')
    ax_effort.set_ylabel('Total Control Effort')
    ax_effort.set_title('Control Activity')
    ax_effort.grid(True, alpha=0.3)
    
    controller_type = data.get('controller_type', 'unknown')
    plt.suptitle(f'Tello Flight Analysis - {controller_type.upper()} Controller', 
                fontsize=16, fontweight='bold')
    
    return fig

def print_statistics(data):
    """Print detailed statistics"""
    t = data['time']
    states = data['states']
    states_desired = data['states_desired']
    commands = data['commands']
    
    print("\n" + "="*70)
    print("FLIGHT STATISTICS")
    print("="*70)
    
    print(f"\nFlight Duration: {t[-1]:.2f} seconds")
    print(f"Data Points: {len(t)}")
    print(f"Average Sample Rate: {len(t)/t[-1]:.1f} Hz")
    
    metrics = PerformanceMetrics.calculate_metrics(t, states, states_desired, verbose=False)
    
    print("\nPerformance Metrics:")
    print(f"  ISE (Integral Square Error):")
    print(f"    X: {metrics['ISE']['x']:.6f}")
    print(f"    Y: {metrics['ISE']['y']:.6f}")
    print(f"    Z: {metrics['ISE']['z']:.6f}")
    
    print(f"\n  RMSE (Root Mean Square Error):")
    print(f"    X: {metrics['RMSE']['x']:.6f} m")
    print(f"    Y: {metrics['RMSE']['y']:.6f} m")
    print(f"    Z: {metrics['RMSE']['z']:.6f} m")
    
    print(f"\n  Maximum Error:")
    print(f"    X: {metrics['max_error']['x']:.6f} m")
    print(f"    Y: {metrics['max_error']['y']:.6f} m")
    print(f"    Z: {metrics['max_error']['z']:.6f} m")
    
    print("\nPosition Range:")
    print(f"  X: [{states[:, 0].min():.3f}, {states[:, 0].max():.3f}] m")
    print(f"  Y: [{states[:, 1].min():.3f}, {states[:, 1].max():.3f}] m")
    print(f"  Z: [{states[:, 2].min():.3f}, {states[:, 2].max():.3f}] m")
    
    v_horz = np.sqrt(commands[:, 0]**2 + commands[:, 1]**2)
    v_vert = np.abs(commands[:, 2])
    
    print("\nVelocity Commands:")
    print(f"  Horizontal - Max: {v_horz.max():.3f} m/s, Avg: {v_horz.mean():.3f} m/s")
    print(f"  Vertical   - Max: {v_vert.max():.3f} m/s, Avg: {v_vert.mean():.3f} m/s")
    print(f"  Yaw Rate   - Max: {np.rad2deg(np.abs(commands[:, 3])).max():.1f}°/s")
    
    distances = np.diff(states[:, 0:3], axis=0)
    total_distance = np.sum(np.linalg.norm(distances, axis=1))
    print(f"\nTotal Distance Traveled: {total_distance:.3f} m")
    
    print("\nSafety Checks:")
    max_altitude = states[:, 2].max()
    min_altitude = states[:, 2].min()
    max_horiz_dist = np.max(np.sqrt(states[:, 0]**2 + states[:, 1]**2))
    
    print(f"  Max altitude: {max_altitude:.3f} m (limit: 2.5 m)")
    print(f"  Min altitude: {min_altitude:.3f} m (limit: 0.3 m)")
    print(f"  Max horizontal distance: {max_horiz_dist:.3f} m (limit: 3.0 m)")
    
    warnings = []
    if max_altitude > 2.5:
        warnings.append("⚠️  Exceeded maximum altitude limit")
    if min_altitude < 0.3:
        warnings.append("⚠️  Went below minimum altitude limit")
    if max_horiz_dist > 3.0:
        warnings.append("⚠️  Exceeded maximum horizontal distance")
    if v_horz.max() > 0.9:
        warnings.append("⚠️  Exceeded recommended horizontal velocity")
    if v_vert.max() > 0.6:
        warnings.append("⚠️  Exceeded recommended vertical velocity")
    
    if warnings:
        print("\nWarnings:")
        for w in warnings:
            print(f"  {w}")
    else:
        print("\n✓ All safety limits respected")
    
    print("="*70 + "\n")

def main():
    parser = argparse.ArgumentParser(description='Analyze Tello flight data')
    parser.add_argument('filename', type=str, help='Flight data file (.npz)')
    parser.add_argument('--save', type=str, default=None, 
                       help='Save plot to file')
    parser.add_argument('--no-plot', action='store_true',
                       help='Only show statistics, no plots')
    
    args = parser.parse_args()
    
    print(f"Loading flight data from: {args.filename}")
    data = load_flight_data(args.filename)
    
    if data is None:
        print("Failed to load flight data")
        return
    
    print(f"✓ Loaded {len(data['time'])} data points")
    
    print_statistics(data)
    
    if not args.no_plot:
        print("Generating plots...")
        fig = plot_comprehensive_analysis(data)
        
        if args.save:
            print(f"Saving plot to: {args.save}")
            fig.savefig(args.save, dpi=300, bbox_inches='tight')
            print("✓ Plot saved")
        else:
            print("Displaying plots...")
            plt.show()

if __name__ == "__main__":
    main()
