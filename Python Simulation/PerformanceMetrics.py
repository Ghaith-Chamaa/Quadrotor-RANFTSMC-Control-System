import numpy as np
from typing import Dict

class PerformanceMetrics:
    """Calculate performance metrics from paper Table 4, 5"""
    
    @staticmethod
    def calculate_metrics(t: np.ndarray, states: np.ndarray, 
                         states_desired: np.ndarray, verbose: bool = False) -> Dict:
        """Calculate ISE, RMSE, IAE, and average error"""
        
        # Position errors
        e_pos = states[:, 0:3] - states_desired[:, 0:3]
        
        # Attitude errors
        e_att = states[:, 6:9] - states_desired[:, 6:9]
        
        # ISE: Integral Square Error (Eq. 79)
        dt = t[1] - t[0]
        ISE = {
            'x': np.sum(e_pos[:, 0]**2) * dt,
            'y': np.sum(e_pos[:, 1]**2) * dt,
            'z': np.sum(e_pos[:, 2]**2) * dt,
            'phi': np.sum(e_att[:, 0]**2) * dt,
            'theta': np.sum(e_att[:, 1]**2) * dt,
            'psi': np.sum(e_att[:, 2]**2) * dt,
        }
        
        # RMSE: Root Mean Square Error
        N = len(t)
        RMSE = {
            'x': np.sqrt(np.mean(e_pos[:, 0]**2)),
            'y': np.sqrt(np.mean(e_pos[:, 1]**2)),
            'z': np.sqrt(np.mean(e_pos[:, 2]**2)),
            'phi': np.sqrt(np.mean(e_att[:, 0]**2)),
            'theta': np.sqrt(np.mean(e_att[:, 1]**2)),
            'psi': np.sqrt(np.mean(e_att[:, 2]**2)),
        }
        
        # IAE: Integral Absolute Error
        IAE = {
            'x': np.sum(np.abs(e_pos[:, 0])) * dt,
            'y': np.sum(np.abs(e_pos[:, 1])) * dt,
            'z': np.sum(np.abs(e_pos[:, 2])) * dt,
            'phi': np.sum(np.abs(e_att[:, 0])) * dt,
            'theta': np.sum(np.abs(e_att[:, 1])) * dt,
            'psi': np.sum(np.abs(e_att[:, 2])) * dt,
        }
        
        # Average Error
        avg_error = {
            'x': np.mean(np.abs(e_pos[:, 0])),
            'y': np.mean(np.abs(e_pos[:, 1])),
            'z': np.mean(np.abs(e_pos[:, 2])),
            'phi': np.mean(np.abs(e_att[:, 0])),
            'theta': np.mean(np.abs(e_att[:, 1])),
            'psi': np.mean(np.abs(e_att[:, 2])),
        }
        
        # Max Error
        max_error = {
            'x': np.max(np.abs(e_pos[:, 0])),
            'y': np.max(np.abs(e_pos[:, 1])),
            'z': np.max(np.abs(e_pos[:, 2])),
            'phi': np.max(np.abs(e_att[:, 0])),
            'theta': np.max(np.abs(e_att[:, 1])),
            'psi': np.max(np.abs(e_att[:, 2])),
        }
        
        if verbose:
            print("\n" + "="*60)
            print("PERFORMANCE METRICS (ISA Transactions Format)")
            print("="*60)
            print("\nIntegral Square Error (ISE) - Table 4:")
            print(f"  Position:")
            print(f"    x:     {ISE['x']:.4f}")
            print(f"    y:     {ISE['y']:.4f}")
            print(f"    z:     {ISE['z']:.4e}")
            print(f"  Attitude:")
            print(f"    φ:     {ISE['phi']:.4f}")
            print(f"    θ:     {ISE['theta']:.4f}")
            print(f"    ψ:     {ISE['psi']:.4f}")
            
            print("\nRoot Mean Square Error (RMSE):")
            print(f"  Position:")
            print(f"    x:     {RMSE['x']:.4f} m")
            print(f"    y:     {RMSE['y']:.4f} m")
            print(f"    z:     {RMSE['z']:.4e} m")
            print(f"  Attitude:")
            print(f"    φ:     {RMSE['phi']:.4f} rad ({np.rad2deg(RMSE['phi']):.2f}°)")
            print(f"    θ:     {RMSE['theta']:.4f} rad ({np.rad2deg(RMSE['theta']):.2f}°)")
            print(f"    ψ:     {RMSE['psi']:.4f} rad ({np.rad2deg(RMSE['psi']):.2f}°)")
            
            print("\nIntegral Absolute Error (IAE):")
            print(f"  Position:")
            print(f"    x:     {IAE['x']:.4f}")
            print(f"    y:     {IAE['y']:.4f}")
            print(f"    z:     {IAE['z']:.4f}")
            print(f"  Attitude:")
            print(f"    φ:     {IAE['phi']:.4f}")
            print(f"    θ:     {IAE['theta']:.4f}")
            print(f"    ψ:     {IAE['psi']:.4f}")
            
            print("\nAverage Absolute Error:")
            print(f"  Position:")
            print(f"    x:     {avg_error['x']:.4f} m")
            print(f"    y:     {avg_error['y']:.4f} m")
            print(f"    z:     {avg_error['z']:.4e} m")
            print(f"  Attitude:")
            print(f"    φ:     {avg_error['phi']:.4f} rad")
            print(f"    θ:     {avg_error['theta']:.4f} rad")
            print(f"    ψ:     {avg_error['psi']:.4f} rad")
            
            print("\nMaximum Absolute Error:")
            print(f"  Position:")
            print(f"    x:     {max_error['x']:.4f} m")
            print(f"    y:     {max_error['y']:.4f} m")
            print(f"    z:     {max_error['z']:.4f} m")
            print(f"  Attitude:")
            print(f"    φ:     {max_error['phi']:.4f} rad")
            print(f"    θ:     {max_error['theta']:.4f} rad")
            print(f"    ψ:     {max_error['psi']:.4f} rad")
            print("="*60 + "\n")
        
        return {
            'ISE': ISE,
            'RMSE': RMSE,
            'IAE': IAE,
            'avg_error': avg_error,
            'max_error': max_error
        }