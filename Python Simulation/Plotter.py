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