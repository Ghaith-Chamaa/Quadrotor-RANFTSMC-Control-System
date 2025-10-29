# Quadrotor RANFTSMC Control System

**Robust Adaptive Nonsingular Fast Terminal Sliding Mode Control for Quadrotor UAV Trajectory Tracking**

A comprehensive implementation of the RANFTSMC algorithm from Labbadi & Cherkaoui (ISA Transactions, 2020) for robust quadrotor control under uncertainties and external disturbances. This repository contains Python simulations, ROS2 integration, and hardware experimental validation.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Repository Structure](#repository-structure)
- [Installation](#installation)
- [Usage](#usage)
  - [Python Simulation](#python-simulation)
  - [ROS2 Integration](#ros2-integration)
- [ROS2 Gazebo Results](#ros2-gazebo-results)
- [Hardware Flight Tests](#hardware-flight-tests)
- [Citation](#citation)
- [License](#license)

---

### Overview

This project implements a **Robust Adaptive Nonsingular Fast Terminal Sliding Mode Controller (RANFTSMC)** for quadrotor UAVs, addressing key challenges in autonomous flight:

- **Robustness** against external disturbances (wind gusts, aerodynamic effects)
- **Adaptation** to parameter uncertainties and unknown dynamics
- **Chattering elimination** through smooth approximation
- **Real-time implementation** on embedded hardware

The implementation progresses through three stages:
1. **Simulation**: Python-based validation with comprehensive scenario testing
2. **ROS2 Integration**: Gazebo simulation with communication protocols
3. **Hardware Testing**: Real quadrotor flights with onboard computation

---

### Features

### Simulation Framework
- **Multiple Trajectory Types**: Figure-8, circular, square, complex multi-segment, space figure-8
- **Disturbance Models**: Constant, time-varying, aggressive (matching paper scenarios)
- **Performance Metrics**: ISE, RMSE, IAE, maximum error tracking
- **Comprehensive Visualization**: 3D trajectories, state tracking, error plots, control inputs
- **Adaptive Control**: Real-time parameter estimation with gain saturation
- **CLI Interface**: Easy scenario selection and parameter tuning

---

### Repository Structure

```
quadrotor-ranftsmc-control/
│
├── python_simulation/          # Pure Python implementation
│   ├── main.py                 # Main simulation entry point
│   ├── RANFTSMController.py    # RANFTSMC controller implementation
│   ├── QuadrotorSimulator.py   # Quadrotor dynamics and simulation
│   ├── Generators.py           # Trajectory and disturbance generators
│   ├── Params.py               # System and controller parameters
│   ├── PerformanceMetrics.py   # Metric calculation (ISE, RMSE, etc.)
│   └── Plotter.py              # Visualization utilities
│
├── ros2_workspace/             # ROS2 packages 
│   └── TODO
│
├── hardware/                   # Hardware implementation 
│   └── TODO
│
├── LICENSE                     # MIT License
└── README.md                   # This file
```

---

### Installation

### Prerequisites

- **Python 3.8+** (for simulation)
- **ROS2 Humble** (for integration and hardware)

### Python Simulation Setup

```bash
# Clone the repository
git clone https://github.com/yourusername/quadrotor-ranftsmc-control.git
cd quadrotor-ranftsmc-control

# Install Python dependencies
pip install numpy matplotlib
```

### ROS2 Workspace Setup (TODO)

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Build workspace
cd ros2_workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

---

### 🎮 Usage

### Python Simulation

#### Quick Start - Paper Scenarios

Run the predefined scenarios from the paper:

```bash
# Simulation 1: Square trajectory, no disturbances
python python_simulation/main.py --scenario sim1 --verbose

# Simulation 2: Circular trajectory, constant disturbances  
python python_simulation/main.py --scenario sim2 --verbose

# Simulation 3: Complex trajectory, time-varying disturbances
python python_simulation/main.py --scenario sim3 --verbose --save results/sim3.png

# Simulation 5: Space figure-8, aggressive disturbances
python python_simulation/main.py --scenario sim5 --verbose
```

#### Custom Configuration

```bash
# Custom trajectory with specific initial conditions
python python_simulation/main.py \
    --trajectory circle \
    --disturbance time_varying \
    --initial "0.5,0,1.0,0,0,0.2" \
    --time 60 \
    --dt 0.01 \
    --verbose \
    --save my_experiment.png
```

#### Available Options

| Argument | Choices | Description |
|----------|---------|-------------|
| `--scenario` | sim1, sim2, sim3, sim5 | Paper simulation scenarios |
| `--trajectory` | 8, circle, square, complex, space8 | Trajectory shape |
| `--disturbance` | none, constant, time_varying, aggressive | Disturbance type |
| `--initial` | "x,y,z,φ,θ,ψ" | Initial conditions (comma-separated) |
| `--time` | float | Simulation duration (seconds) |
| `--dt` | float | Time step for RK approximiation (default: 0.01s) |
| `--verbose` | flag | Detailed output and metrics |
| `--save` | filename | Save plots to file |

### ROS2 Integration

```bash

```
---

### ROS2 Gazebo Results

---

### Hardware Flight Tests

---

### Citation

If you use this code in your research, please cite the original authors:

**Original Paper**:
```bibtex
@article{labbadi2020robust,
  title={Robust adaptive nonsingular fast terminal sliding-mode tracking control for an uncertain quadrotor UAV subjected to disturbances},
  author={Labbadi, Moussa and Cherkaoui, Mohamed},
  journal={ISA transactions},
  volume={99},
  pages={290--304},
  year={2020},
  publisher={Elsevier}
}
```

---

### License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

---