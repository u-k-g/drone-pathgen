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

## Python API Usage

After building the project, you can use the Python wrapper to plan drone trajectories:

```python
import sys
import numpy as np
sys.path.append('build')  # add build directory to path
import gcopter_cpp

# create api instance
api = gcopter_cpp.GCopterAPI()

# 1. configure 3d map with obstacles
map_size = np.array([20, 20, 10], dtype=np.int32)  # 20x20x10 voxel grid
origin = np.array([-5.0, -5.0, 0.0])  # map origin in world coords
voxel_scale = 0.5  # 0.5m per voxel

# define obstacle positions
obstacles = [
    np.array([0.0, 0.0, 1.0]),   # center obstacle
    np.array([-1.0, -1.0, 1.0]), # side obstacle
    np.array([1.0, 1.0, 1.0]),   # side obstacle
]

api.configure_map(map_size, origin, voxel_scale, obstacles, dilation_radius=2)

# 2. set start and goal positions
start_pos = np.array([-3.0, -3.0, 1.0])
goal_pos = np.array([3.0, 3.0, 1.0])
api.set_endpoints(start_pos, goal_pos)

# 3. set up optimization parameters
planning_timeout = 5.0
time_weight = 50.0
segment_length = 2.0
smoothing_epsilon = 1e-3
integral_resolution = 8

# vehicle constraints: [v_max, omega_max, theta_max, thrust_min, thrust_max]
magnitude_bounds = np.array([5.0, 10.0, np.pi/3, 5.0, 15.0])

# penalty weights: [pos, vel, omega, theta, thrust]
penalty_weights = np.array([1.0, 1.0, 1.0, 1.0, 1.0])

# physical params: [mass, grav, horiz_drag, vert_drag, parasitic_drag, speed_smooth]
physical_params = np.array([1.0, 9.81, 0.0, 0.0, 0.0, 0.01])

# 4. run trajectory optimization
success = api.run_inference(
    planning_timeout, time_weight, segment_length,
    smoothing_epsilon, integral_resolution,
    magnitude_bounds, penalty_weights, physical_params
)

if success:
    # get trajectory statistics
    stats = gcopter_cpp.TrajectoryStatistics()
    api.get_statistics(stats)
    print(f"trajectory duration: {stats.total_duration:.2f}s")
    print(f"max velocity: {stats.max_velocity:.2f} m/s")
    
    # get drone state at specific time
    state = gcopter_cpp.DroneState()
    api.get_state_at_time(stats.total_duration * 0.5, state)  # halfway point
    print(f"position at t={stats.total_duration*0.5:.2f}s: {state.position}")
    
    # get visualization data for plotting
    result = api.get_visualization_data(show_initial_route=True)
    success, trajectory_points, voxel_data, voxel_size, start, goal, initial_route = result
    print(f"trajectory has {len(trajectory_points)} points")
else:
    print("trajectory optimization failed!")
```

### complete example

see `src/test_wrapper.py` for a complete working example with 3d visualization using open3d.

### key concepts

- **voxel map**: 3d grid representing obstacles (0=free, 1=occupied, 2=safety dilation)
- **safe flight corridor**: convex polytopes around the planned path ensuring collision avoidance
- **trajectory optimization**: smooth polynomial trajectory considering drone physics and constraints

## Applications

- **Autonomous Delivery**: Plan efficient routes for package delivery drones in urban environments
- **Search and Rescue**: Navigate drones through debris and obstacles during emergency operations  
- **Inspection Tasks**: Generate precise flight paths for infrastructure inspection and monitoring
- **Research Projects**: Rapidly prototype and test new drone navigation algorithms
- **Simulation**: Create realistic flight paths for drone simulators and training systems
- **Competition Flying**: Plan optimal racing lines and stunt sequences for drone competitions
- **Mapping and Surveying**: Generate systematic flight patterns for aerial photography and LiDAR scanning
- **Indoor Navigation**: Navigate drones through buildings, warehouses, and confined spaces

