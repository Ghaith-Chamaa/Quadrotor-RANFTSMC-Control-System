#!/usr/bin/env python3
"""
Robust Adaptive Nonsingular Fast Terminal Sliding Mode Control (RANFTSMC)
for Quadrotor UAV Trajectory Tracking

Based on: Labbadi & Cherkaoui (2020) - ISA Transactions
"""
import argparse
import matplotlib.pyplot as plt
import numpy as np

from Params import QuadrotorParams, ControllerParams
from RANFTSMController import RANFTSMController
from PerformanceMetrics import PerformanceMetrics
from QuadrotorSimulator import QuadrotorSimulator
from Plotter import Plotter

def parse_initial_conditions(init_str):
    """Parse initial condition string into array"""
    if init_str is None:
        return None
    try:
        vals = [float(x.strip()) for x in init_str.split(',')]
        if len(vals) != 6:
            raise ValueError("Initial conditions must have 6 values: [x, y, z, phi, theta, psi]")
        return np.array(vals)
    except Exception as e:
        raise ValueError(f"Error parsing initial conditions: {e}")

def main():
    """Main simulation function"""
    
    parser = argparse.ArgumentParser(
        description='RANFTSMC Quadrotor Trajectory Tracking Simulation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Simulation 1: Figure-8 trajectory with no disturbances (Paper Sec 4.1)
  python main.py --scenario sim1
  
  # Simulation 2: Circular trajectory with constant disturbances (Paper Sec 4.2)
  python main.py --scenario sim2
  
  # Simulation 3: Complex trajectory with time-varying disturbances (Paper Sec 4.3)
  python main.py --scenario sim3
  
  # Simulation 5: Figure-8 with aggressive disturbances and uncertainties (Paper Sec 4.5)
  python main.py --scenario sim5
  
  # Custom configuration
  python main.py --trajectory 8 --disturbance none --initial "0,0,0.5,0,0,0" --time 50
  
  # Custom with square trajectory (Paper Sec 4.1)
  python main.py --trajectory square --disturbance none --initial "0,0,0,0,0,0"
            """)
    
    parser.add_argument('--scenario', type=str, default=None,
                       choices=['sim1', 'sim2', 'sim3', 'sim5'],
                       help='Run specific paper simulation scenario (overrides other trajectory/disturbance options)')
    
    parser.add_argument('--trajectory', type=str, default='8',
                       choices=['8', 'circle', 'square', 'complex', 'space8'],
                       help='Trajectory shape: 8 (figure-eight), circle, square (Sim1), complex (Sim3), space8 (Sim5)')
    
    parser.add_argument('--disturbance', type=str, default='none',
                       choices=['none', 'constant', 'time_varying', 'aggressive'],
                       help='Disturbance type: none, constant (Sim2), time_varying (Sim3), aggressive (Sim5)')
    
    parser.add_argument('--initial', type=str, default=None,
                       help='Initial conditions as comma-separated: "x,y,z,phi,theta,psi" (e.g., "0,0,0.5,0,0,0")')
    
    parser.add_argument('--time', type=float, default=None,
                       help='Simulation time in seconds (default: auto-calculated per trajectory)')
    
    parser.add_argument('--dt', type=float, default=0.01,
                       help='Time step in seconds (default: 0.01)')
    
    parser.add_argument('--verbose', action='store_true',
                       help='Print detailed performance metrics and progress')
    
    parser.add_argument('--save', type=str, default=None,
                       help='Save plots to file (e.g., results.png)')
    
    args = parser.parse_args()
    
    # Handle paper scenarios
    if args.scenario:
        if args.scenario == 'sim1':
            args.trajectory = 'square'
            args.disturbance = 'none'
            args.initial = "0,0,0,0,0,0"  # Paper Sec 4.1: null initial conditions
            args.time = 50.0
        elif args.scenario == 'sim2':
            args.trajectory = 'circle'
            args.disturbance = 'constant'
            args.initial = "0,0,0.5,0,0,0"  # Paper Sec 4.2
            args.time = 40.0
        elif args.scenario == 'sim3':
            args.trajectory = 'complex'
            args.disturbance = 'time_varying'
            args.initial = "0.5,0.5,0.5,0,0,0.5"  # Paper Sec 4.3
            args.time = 80.0
        elif args.scenario == 'sim5':
            args.trajectory = 'space8'
            args.disturbance = 'aggressive'
            args.initial = "0.5,0.5,0.5,0,0,0.5"  # Paper Sec 4.5
            args.time = 120.0
    
    # Parse initial conditions
    initial_conditions = parse_initial_conditions(args.initial)
    
    print("\n" + "="*70)
    print("RANFTSMC QUADROTOR TRAJECTORY TRACKING SIMULATION")
    print("Based on: Labbadi & Cherkaoui (2020) - ISA Transactions")
    print("="*70)
    print(f"\nConfiguration:")
    if args.scenario:
        print(f"  Scenario:      {args.scenario.upper()} (Paper Section 4.{args.scenario[-1]})")
    print(f"  Trajectory:    {args.trajectory.upper()}")
    print(f"  Disturbance:   {args.disturbance}")
    if initial_conditions is not None:
        print(f"  Initial pos:   [{initial_conditions[0]:.2f}, {initial_conditions[1]:.2f}, {initial_conditions[2]:.2f}] m")
        print(f"  Initial att:   [{initial_conditions[3]:.2f}, {initial_conditions[4]:.2f}, {initial_conditions[5]:.2f}] rad")
    print(f"  Sim Time:      {args.time if args.time else 'auto'}s")
    print(f"  Time Step:     {args.dt}s")
    print(f"  Verbose:       {args.verbose}")
    print("="*70 + "\n")
    
    quad_params = QuadrotorParams()
    ctrl_params = ControllerParams()
    controller = RANFTSMController(quad_params, ctrl_params)
    simulator = QuadrotorSimulator(quad_params, controller)
    
    print("Starting simulation...")
    t, states, states_desired, controls = simulator.simulate(
        trajectory_type=args.trajectory,
        disturbance_type=args.disturbance,
        initial_conditions=initial_conditions,
        T_sim=args.time,
        dt=args.dt,
        verbose=args.verbose
    )
    
    print("\nCalculating performance metrics...")
    metrics = PerformanceMetrics.calculate_metrics(
        t, states, states_desired, verbose=args.verbose)
    
    print("\nGenerating plots...")
    scenario_label = f"Simulation {args.scenario[-1]}" if args.scenario else None
    fig_main = Plotter.plot_results(
        t, states, states_desired, controls, 
        args.trajectory, args.disturbance, metrics, scenario_label)
    
    fig_errors = Plotter.plot_errors(t, states, states_desired)
    
    if args.save:
        print(f"\nSaving plots to {args.save}...")
        fig_main.savefig(args.save, dpi=300, bbox_inches='tight')
        error_filename = args.save.replace('.png', '_errors.png')
        fig_errors.savefig(error_filename, dpi=300, bbox_inches='tight')
        print(f"Saved: {args.save}, {error_filename}")
    else:
        print("\nDisplaying plots... (close windows to exit)")
        plt.show()
    
    print("\n" + "="*70)
    print("SIMULATION COMPLETE")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()