import numpy as np
from matplotlib.gridspec import GridSpec
import matplotlib.pyplot as plt
from typing import Dict

class Plotter:
    """Generate plots similar to paper figures"""
    
    @staticmethod
    def plot_results(t: np.ndarray, states: np.ndarray, states_desired: np.ndarray,
                    controls: np.ndarray, trajectory_type: str, disturbance_type: str,
                    metrics: Dict, scenario_label: str = None):
        """Create comprehensive visualization"""
        
        fig = plt.figure(figsize=(16, 12))
        gs = GridSpec(4, 3, figure=fig, hspace=0.3, wspace=0.3)
        
        ax_3d = fig.add_subplot(gs[0:2, 0:2], projection='3d')
        ax_3d.plot(states_desired[:, 0], states_desired[:, 1], states_desired[:, 2],
                  'b--', linewidth=2, label='Desired', alpha=0.7)
        ax_3d.plot(states[:, 0], states[:, 1], states[:, 2],
                  'r-', linewidth=1.5, label='Actual (RANFTSMC)', alpha=0.9)
        ax_3d.scatter(states[0, 0], states[0, 1], states[0, 2], 
                     c='g', s=100, marker='o', label='Start', zorder=5)
        ax_3d.scatter(states[-1, 0], states[-1, 1], states[-1, 2],
                     c='m', s=100, marker='s', label='End', zorder=5)
        ax_3d.set_xlabel('X (m)', fontsize=10)
        ax_3d.set_ylabel('Y (m)', fontsize=10)
        ax_3d.set_zlabel('Z (m)', fontsize=10)
        
        title_str = f'3D Trajectory Tracking - {trajectory_type.upper()} Shape'
        if scenario_label:
            title_str = f'{scenario_label}: ' + title_str
        title_str += f'\nDisturbance: {disturbance_type}'
        ax_3d.set_title(title_str, fontsize=12, fontweight='bold')
        
        ax_3d.legend(loc='upper right', fontsize=8)
        ax_3d.grid(True, alpha=0.3)
        
        axes_pos = [fig.add_subplot(gs[0, 2]), fig.add_subplot(gs[1, 2]), fig.add_subplot(gs[2, 0])]
        labels = ['X (m)', 'Y (m)', 'Z (m)']
        
        for i, (ax, label) in enumerate(zip(axes_pos, labels)):
            ax.plot(t, states_desired[:, i], 'b--', linewidth=2, label='Desired', alpha=0.7)
            ax.plot(t, states[:, i], 'r-', linewidth=1.5, label='Actual', alpha=0.9)
            ax.set_xlabel('Time (s)', fontsize=9)
            ax.set_ylabel(label, fontsize=9)
            ax.set_title(f'Position {label[0]}', fontsize=10)
            ax.legend(fontsize=7, loc='best')
            ax.grid(True, alpha=0.3)
        
        axes_att = [fig.add_subplot(gs[2, 1]), fig.add_subplot(gs[2, 2]), fig.add_subplot(gs[3, 0])]
        labels_att = ['φ (rad)', 'θ (rad)', 'ψ (rad)']
        
        for i, (ax, label) in enumerate(zip(axes_att, labels_att)):
            ax.plot(t, states_desired[:, 6+i], 'b--', linewidth=2, label='Desired', alpha=0.7)
            ax.plot(t, states[:, 6+i], 'r-', linewidth=1.5, label='Actual', alpha=0.9)
            ax.set_xlabel('Time (s)', fontsize=9)
            ax.set_ylabel(label, fontsize=9)
            ax.set_title(f'Attitude {label.split()[0]}', fontsize=10)
            ax.legend(fontsize=7, loc='best')
            ax.grid(True, alpha=0.3)
        
        # Control inputs
        ax_thrust = fig.add_subplot(gs[3, 1])
        ax_thrust.plot(t[:-1], controls[:-1, 0], 'b-', linewidth=1.5)
        ax_thrust.set_xlabel('Time (s)', fontsize=9)
        ax_thrust.set_ylabel('Thrust (N)', fontsize=9)
        ax_thrust.set_title('Control Input: U_T', fontsize=10)
        ax_thrust.grid(True, alpha=0.3)
        
        ax_torques = fig.add_subplot(gs[3, 2])
        ax_torques.plot(t[:-1], controls[:-1, 1], 'r-', linewidth=1, label='U_φ', alpha=0.7)
        ax_torques.plot(t[:-1], controls[:-1, 2], 'g-', linewidth=1, label='U_θ', alpha=0.7)
        ax_torques.plot(t[:-1], controls[:-1, 3], 'b-', linewidth=1, label='U_ψ', alpha=0.7)
        ax_torques.set_xlabel('Time (s)', fontsize=9)
        ax_torques.set_ylabel('Torques (N·m)', fontsize=9)
        ax_torques.set_title('Control Inputs: Torques', fontsize=10)
        ax_torques.legend(fontsize=7, loc='best')
        ax_torques.grid(True, alpha=0.3)
        
        metrics_text = f"Performance Metrics:\n"
        metrics_text += f"ISE: x={metrics['ISE']['x']:.4f}, y={metrics['ISE']['y']:.4f}, z={metrics['ISE']['z']:.4e}\n"
        metrics_text += f"RMSE: x={metrics['RMSE']['x']:.4f}m, y={metrics['RMSE']['y']:.4f}m, z={metrics['RMSE']['z']:.4e}m"
        
        fig.text(0.02, 0.02, metrics_text, fontsize=8, family='monospace',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        main_title = 'RANFTSMC Quadrotor Trajectory Tracking'
        if scenario_label:
            main_title = f'{scenario_label}: ' + main_title
        main_title += '\n(Labbadi & Cherkaoui, ISA Trans. 2020)'
        plt.suptitle(main_title, fontsize=14, fontweight='bold', y=0.995)
        
        return fig
    
    @staticmethod
    def plot_errors(t: np.ndarray, states: np.ndarray, states_desired: np.ndarray):
        """Plot tracking errors (similar to paper Figures 6, 7)"""
        
        fig, axes = plt.subplots(2, 3, figsize=(15, 8))
        fig.suptitle('Tracking Errors', fontsize=14, fontweight='bold')
        
        e_pos = states[:, 0:3] - states_desired[:, 0:3]
        labels_pos = ['e_x (m)', 'e_y (m)', 'e_z (m)']
        
        for i in range(3):
            axes[0, i].plot(t, e_pos[:, i], 'r-', linewidth=1.5)
            axes[0, i].axhline(y=0, color='k', linestyle='--', linewidth=0.8, alpha=0.5)
            axes[0, i].set_xlabel('Time (s)', fontsize=10)
            axes[0, i].set_ylabel(labels_pos[i], fontsize=10)
            axes[0, i].set_title(f'Position Error: {labels_pos[i].split()[0]}', fontsize=11)
            axes[0, i].grid(True, alpha=0.3)
        
        e_att = states[:, 6:9] - states_desired[:, 6:9]
        labels_att = ['e_φ (rad)', 'e_θ (rad)', 'e_ψ (rad)']
        
        for i in range(3):
            axes[1, i].plot(t, e_att[:, i], 'b-', linewidth=1.5)
            axes[1, i].axhline(y=0, color='k', linestyle='--', linewidth=0.8, alpha=0.5)
            axes[1, i].set_xlabel('Time (s)', fontsize=10)
            axes[1, i].set_ylabel(labels_att[i], fontsize=10)
            axes[1, i].set_title(f'Attitude Error: {labels_att[i].split()[0]}', fontsize=11)
            axes[1, i].grid(True, alpha=0.3)
        
        plt.tight_layout()
        return fig

    @staticmethod
    def plot_comprehensive_analysis(t: np.ndarray, states: np.ndarray, states_desired: np.ndarray,
                                    controls: np.ndarray, sliding_surfaces: np.ndarray,
                                    trajectory_type: str, disturbance_type: str, scenario_label: str = None):
        """
        Create comprehensive analysis plots similar to hardware.png
        Includes: 3D trajectory, position tracking, velocity commands, sliding surfaces, and errors
        Multi-axis variables are overlayed on the same plot for better comparison
        """
        fig = plt.figure(figsize=(20, 14))
        gs = GridSpec(4, 4, figure=fig, hspace=0.35, wspace=0.35)

        # Row 0: 3D Trajectory (large) + X Position + Y Position
        ax_3d = fig.add_subplot(gs[0:2, 0:2], projection='3d')
        ax_3d.plot(states_desired[:, 0], states_desired[:, 1], states_desired[:, 2],
                  'b--', linewidth=2, label='Desired', alpha=0.7)
        ax_3d.plot(states[:, 0], states[:, 1], states[:, 2],
                  'r-', linewidth=1.5, label='Actual', alpha=0.9)
        ax_3d.scatter(states[0, 0], states[0, 1], states[0, 2],
                     c='g', s=100, marker='o', label='Start', zorder=5)
        ax_3d.scatter(states[-1, 0], states[-1, 1], states[-1, 2],
                     c='m', s=100, marker='s', label='End', zorder=5)
        ax_3d.set_xlabel('X (m)', fontsize=9)
        ax_3d.set_ylabel('Y (m)', fontsize=9)
        ax_3d.set_zlabel('Z (m)', fontsize=9)
        ax_3d.set_title('3D Trajectory', fontsize=10, fontweight='bold')
        ax_3d.legend(fontsize=7)
        ax_3d.grid(True, alpha=0.3)

        # X Position
        ax_x = fig.add_subplot(gs[0, 2])
        ax_x.plot(t, states_desired[:, 0], 'b--', linewidth=1.5, label='Desired', alpha=0.7)
        ax_x.plot(t, states[:, 0], 'r-', linewidth=1.2, label='Actual', alpha=0.9)
        ax_x.fill_between(t, states_desired[:, 0], states[:, 0], alpha=0.2, color='red')
        ax_x.set_xlabel('Time (s)', fontsize=8)
        ax_x.set_ylabel('X (m)', fontsize=8)
        ax_x.set_title('X Position', fontsize=9, fontweight='bold')
        ax_x.legend(fontsize=6)
        ax_x.grid(True, alpha=0.3)

        # Y Position
        ax_y = fig.add_subplot(gs[0, 3])
        ax_y.plot(t, states_desired[:, 1], 'b--', linewidth=1.5, label='Desired', alpha=0.7)
        ax_y.plot(t, states[:, 1], 'r-', linewidth=1.2, label='Actual', alpha=0.9)
        ax_y.fill_between(t, states_desired[:, 1], states[:, 1], alpha=0.2, color='red')
        ax_y.set_xlabel('Time (s)', fontsize=8)
        ax_y.set_ylabel('Y (m)', fontsize=8)
        ax_y.set_title('Y Position', fontsize=9, fontweight='bold')
        ax_y.legend(fontsize=6)
        ax_y.grid(True, alpha=0.3)

        # Row 1: Z Position + Attitude (Roll, Pitch, Yaw overlayed) + Control Commands
        ax_z = fig.add_subplot(gs[1, 2])
        ax_z.plot(t, states_desired[:, 2], 'b--', linewidth=1.5, label='Desired', alpha=0.7)
        ax_z.plot(t, states[:, 2], 'r-', linewidth=1.2, label='Actual', alpha=0.9)
        ax_z.fill_between(t, states_desired[:, 2], states[:, 2], alpha=0.2, color='red')
        ax_z.set_xlabel('Time (s)', fontsize=8)
        ax_z.set_ylabel('Z (m)', fontsize=8)
        ax_z.set_title('Z Position', fontsize=9, fontweight='bold')
        ax_z.legend(fontsize=6)
        ax_z.grid(True, alpha=0.3)

        # Control Commands (Thrust and Torques)
        ax_controls = fig.add_subplot(gs[1, 3])
        # Note: controls array has shape (N, 4) = [U_T, U_phi, U_theta, U_psi]
        # Normalize thrust for better visualization alongside torques
        thrust_normalized = (controls[:, 0] - controls[:, 0].mean()) / (controls[:, 0].std() + 1e-10)
        ax_controls.plot(t, thrust_normalized, 'k-', linewidth=1.0, alpha=0.7, label='U_T (norm)')
        ax_controls.plot(t, controls[:, 1], 'r-', linewidth=1.0, alpha=0.9, label='U_φ')
        ax_controls.plot(t, controls[:, 2], 'g-', linewidth=1.0, alpha=0.9, label='U_θ')
        ax_controls.plot(t, controls[:, 3], 'b-', linewidth=1.0, alpha=0.9, label='U_ψ')
        ax_controls.set_xlabel('Time (s)', fontsize=8)
        ax_controls.set_ylabel('Control', fontsize=8)
        ax_controls.set_title('Control Commands', fontsize=9, fontweight='bold')
        ax_controls.legend(fontsize=6, ncol=2, loc='best')
        ax_controls.grid(True, alpha=0.3)

        # Row 2: Position Sliding Surfaces (all overlayed) - spans full width
        ax_sigma_pos = fig.add_subplot(gs[2, 0:4])
        ax_sigma_pos.plot(t, sliding_surfaces[:, 0], 'r-', linewidth=1.2, alpha=0.9, label='σ_x')
        ax_sigma_pos.plot(t, sliding_surfaces[:, 1], 'g-', linewidth=1.2, alpha=0.9, label='σ_y')
        ax_sigma_pos.plot(t, sliding_surfaces[:, 2], 'b-', linewidth=1.2, alpha=0.9, label='σ_z')
        ax_sigma_pos.axhline(y=0, color='k', linestyle='--', linewidth=0.8, alpha=0.5)
        ax_sigma_pos.set_xlabel('Time (s)', fontsize=8)
        ax_sigma_pos.set_ylabel('σ', fontsize=8)
        ax_sigma_pos.set_title('Position Sliding Surfaces (X, Y, Z)', fontsize=9, fontweight='bold')
        ax_sigma_pos.legend(fontsize=6, ncol=3, loc='best')
        ax_sigma_pos.grid(True, alpha=0.3)

        # Row 3: Position Errors (all overlayed) - spans full width
        errors = states[:, 0:3] - states_desired[:, 0:3]
        ax_err_pos = fig.add_subplot(gs[3, 0:4])
        ax_err_pos.plot(t, errors[:, 0], 'r-', linewidth=1.2, alpha=0.9, label='e_x')
        ax_err_pos.plot(t, errors[:, 1], 'g-', linewidth=1.2, alpha=0.9, label='e_y')
        ax_err_pos.plot(t, errors[:, 2], 'b-', linewidth=1.2, alpha=0.9, label='e_z')
        ax_err_pos.axhline(y=0, color='k', linestyle='--', linewidth=0.8, alpha=0.5)
        ax_err_pos.set_xlabel('Time (s)', fontsize=8)
        ax_err_pos.set_ylabel('Error (m)', fontsize=8)
        ax_err_pos.set_title('Position Errors (X, Y, Z)', fontsize=9, fontweight='bold')
        ax_err_pos.legend(fontsize=6, ncol=3, loc='best')
        ax_err_pos.grid(True, alpha=0.3)

        # Title
        main_title = f'RANFTSMC - {trajectory_type.upper()} Controller'
        if scenario_label:
            main_title = f'{scenario_label}: ' + main_title
        plt.suptitle(main_title, fontsize=14, fontweight='bold', y=0.995)

        return fig