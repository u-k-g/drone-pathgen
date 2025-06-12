# GCOPTER Python Wrapper

A Python interface for the GCOPTER library, enabling advanced drone trajectory planning that generates smooth, collision-free flight paths in complex 3D environments.

<img width="459" alt="image" src="https://github.com/user-attachments/assets/aec72152-8c2c-4bcf-8da1-df6a68c17cd9" />

## Features

- **Smart Path Planning**: Automatically generates optimal trajectories that avoid obstacles while considering real drone physics and dynamics.
- **3D Environment Support**: Works with any 3D map format that can be represented as a voxel grid, including point clouds and obstacle lists.
- **Physics-Aware**: Incorporates drone mass, drag, thrust limits, and other physical constraints for realistic flight paths.
- **Smooth Trajectories**: Produces continuous, jerk-limited paths that result in stable, efficient flight.
- **Safe Flight Corridors (SFC)**: Creates protective convex polytope zones around planned paths to ensure collision avoidance with safety margins.
- **Fast Optimization**: High-performance trajectory generation suitable for real-time applications.
- **Python-Native Integration**: Seamlessly works with NumPy and other Python scientific libraries.
- **Standalone**: Does not require ROS (Robot Operating System) or other heavy frameworks.

## Quick Start

### Prerequisites
- A C++ compiler (e.g., GCC, Clang)
- CMake (version 3.10 or newer)
- Python (version 3.8 or newer)
- `uv` (for Python environment management)

### Building the Project

The project includes a simple build script to handle compilation and setup.

```fish
# quick build and run the example (recommended)
./scripts/build.sh

# to run with 3d visualization (requires open3d)
# pip install open3d
./scripts/build.sh --visualize
```

For manual compilation, follow these steps:
```fish
# create a build directory
mkdir -p build && cd build

# configure and compile
cmake ..
make

# run the example from the root directory
cd ..
uv run python src/example_usage.py
```

## Python API Usage

After building the project, you can use the Python wrapper to plan drone trajectories as shown below.

```python
import sys
import numpy as np

# add the build directory to the python path to find our module
sys.path.append('build')  
import gcopter_cpp

# create an api instance
api = gcopter_cpp.GCopterAPI()

# 1. configure a 3d map with obstacles
map_size = np.array([20, 20, 10], dtype=np.int32)  # 20x20x10 voxel grid
origin = np.array([-5.0, -5.0, 0.0])              # map origin in world coordinates
voxel_scale = 0.5                                 # 0.5 meters per voxel

# define obstacle positions in world coordinates
obstacles = [
    np.array([0.0, 0.0, 1.0]),   # an obstacle in the middle
    np.array([-1.0, -1.0, 1.0]), # a side obstacle
    np.array([1.0, 1.0, 1.0]),   # another side obstacle
]

# this creates the internal voxel map and inflates obstacles for safety
api.configure_map(map_size, origin, voxel_scale, obstacles, dilation_radius=2)

# 2. set the start and goal positions for the trajectory
start_pos = np.array([-3.0, -3.0, 1.0])
goal_pos = np.array([3.0, 3.0, 1.0])
api.set_endpoints(start_pos, goal_pos) # optional: add start/goal velocities as 3rd/4th args

# 3. set up optimization parameters
planning_timeout = 5.0      # max time for initial path finding
time_weight = 50.0          # penalty on total flight time (higher = faster)
segment_length = 2.0        # length of each corridor segment
smoothing_epsilon = 1e-3    # epsilon for corridor smoothing
integral_resolution = 8     # number of points per segment for penalty checks

# vehicle constraints: [v_max, omega_max, theta_max, thrust_min, thrust_max]
magnitude_bounds = np.array([5.0, 10.0, np.pi/3, 5.0, 15.0])

# penalty weights for optimizer: [pos, vel, omega, theta, thrust]
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
    print("✅ trajectory optimization successful!")
    
    # get trajectory statistics
    stats = gcopter_cpp.TrajectoryStatistics()
    api.get_statistics(stats)
    print(f"  trajectory duration: {stats.total_duration:.2f}s")
    print(f"  max velocity: {stats.max_velocity:.2f} m/s")
    
    # get drone state (pos, vel, acc) at a specific time
    state = gcopter_cpp.DroneState()
    api.get_state_at_time(stats.total_duration * 0.5, state)  # halfway point
    print(f"  position at t={stats.total_duration*0.5:.2f}s: {state.position}")
    
    # get visualization data for plotting
    result = api.get_visualization_data(show_initial_route=True)
    # unpack tuple: success, trajectory_points, voxel_data, voxel_size, start, goal, initial_route
    print(f"  trajectory has {len(result[1])} points")
else:
    print("❌ trajectory optimization failed!")
```

### Complete Example

See `src/example_usage.py` for a complete working example with 3D visualization using Open3D.

### Key Concepts

- **Voxel Map**: A 3D grid representing the environment. Each cell (voxel) can be free, occupied by an obstacle, or part of a safety margin.
- **Safe Flight Corridor (SFC)**: A series of connected, overlapping convex polyhedra that create a tunnel-like safe zone for the drone to fly through.
- **Trajectory Optimization**: The process of finding the smoothest possible flight path (a polynomial trajectory) that stays within the SFC and respects the drone's physical limits.

## Applications

- **Autonomous Delivery**: Plan efficient routes for package delivery drones in urban environments.
- **Search and Rescue**: Navigate drones through debris and obstacles during emergency operations.
- **Inspection Tasks**: Generate precise flight paths for infrastructure inspection and monitoring.
- **Research & Prototyping**: Rapidly test new drone navigation and control algorithms.
- **Simulation**: Create realistic flight paths for drone simulators and training systems.
- **Competition Flying**: Plan optimal racing lines and stunt sequences for drone competitions.
- **Mapping and Surveying**: Generate systematic flight patterns for aerial photography and LiDAR scanning.
- **Indoor Navigation**: Navigate drones through buildings, warehouses, and other confined spaces.

