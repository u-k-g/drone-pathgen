# GCOPTER Python Wrapper

A Python interface for advanced drone trajectory planning that generates smooth, collision-free flight paths in complex 3D environments.

<img width="1165" alt="image" src="https://github.com/user-attachments/assets/20b018d9-9d51-4967-9a82-de7ea06e38cb" />

## Features

- **Smart Path Planning**: Automatically generates optimal trajectories that avoid obstacles while considering real drone physics and dynamics
- **3D Environment Support**: Works with any 3D map format including voxel grids, point clouds, and obstacle lists
- **Physics-Aware**: Incorporates drone mass, drag, thrust limits, and other physical constraints for realistic flight paths
- **Smooth Trajectories**: Produces continuous, jerk-limited paths that result in stable, efficient flight
- **Safe Flight Corridors**: Creates protective zones around planned paths to ensure collision avoidance with safety margins
- **Fast Optimization**: High-performance trajectory generation suitable for real-time applications
- **Python Native**: Seamless integration with NumPy, SciPy, and other Python scientific libraries
- **Not ROS Dependent**: Doesn't require ROS

## Quick Start

### Building the Project

```fish
# Quick build and test (recommended)
./scripts/build.sh --test

# Build, test, and visualize in one command
./scripts/build.sh --test --visualize

# Or manually step by step
mkdir -p build && cd build
cmake .. && make
./test > ../output/enhanced_output.txt
cd .. && uv run python visualization/visualize_open3d.py
```

## Usage

The library provides a simple workflow for drone path planning:

1. **Configure Environment**: Load your 3D map with obstacles and barriers
2. **Set Waypoints**: Define where your drone needs to start and end
3. **Generate Path**: Let the optimizer create a smooth, safe trajectory
4. **Execute Flight**: Use the generated waypoints and timing for drone control

Perfect for researchers, hobbyists, and professionals who need reliable drone path planning without dealing with complex robotics frameworks.

## Applications

- **Autonomous Delivery**: Plan efficient routes for package delivery drones in urban environments
- **Search and Rescue**: Navigate drones through debris and obstacles during emergency operations  
- **Inspection Tasks**: Generate precise flight paths for infrastructure inspection and monitoring
- **Research Projects**: Rapidly prototype and test new drone navigation algorithms
- **Simulation**: Create realistic flight paths for drone simulators and training systems
- **Competition Flying**: Plan optimal racing lines and stunt sequences for drone competitions
- **Mapping and Surveying**: Generate systematic flight patterns for aerial photography and LiDAR scanning
- **Indoor Navigation**: Navigate drones through buildings, warehouses, and confined spaces

