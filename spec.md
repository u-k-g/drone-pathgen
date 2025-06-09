# GCOPTER Python API Wrapper - Project Specification

## Project Overview
This project creates a Python-accessible wrapper around GCOPTER, a state-of-the-art trajectory optimization library for multicopters developed by ZJU-FAST-Lab. GCOPTER provides smooth, physics-aware trajectory generation that considers vehicle dynamics, obstacle avoidance, and control constraints.
**Repository**: [ZJU-FAST-Lab/GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER)
## GCOPTER.

https://github.com/ZJU-FAST-Lab/GCOPTER

GCOPTER is a trajectory optimizer built on MINCO (Minimum Control Output) that:
- Takes 3D maps and start/end points as input
- Generates smooth trajectories that avoid obstacles
- Embeds control constraints directly in the path planning
- Considers multicopter dynamics including nonlinear drag effects
- Produces trajectories with embedded control inputs for smooth flight

## Project Goals
The absolute top priority is to:

- create a cpp api to do some basic calls, (eg. configure map, set endpoints, run inference, visualize results)

- wrap it in a pybind11 library so that we can call it in python
### Main Objectives (explained in more depth)

1. **Create a C++ API** with some basic functions. some examples are:
	1. `configureMap()` - Load and process 3D environment data
	2. `setEndpoints()` - Define start and goal positions
	3. `runInference()` - Execute trajectory optimization
	4. `visualizeResults()` - Return trajectory data for visualization
	
2. **Python Integration** via pybind11:
	1. Wrap the C++ API for Python access
	2. Enable seamless data exchange between Python and C++
	3. Support numpy arrays for trajectory data

### Core Capabilities
- **Input**: 3D maps (voxel grids, point clouds, or obstacle lists) + start/end points
- **Processing**: Safe Flight Corridor (SFC) generation + trajectory optimization
- **Output**: Smooth, dynamically-feasible trajectories with timing
### Processing Pipeline
1. **Environment Representation** - Build 3D occupancy grid from obstacle data
2. **Initial Path Planning** - Use OMPL planner to find collision-free waypoints  
3. **Safe-Flight-Corridor Generation** - Create convex polytopes around path segments
4. **Polynomial Trajectory Initialization** - Fit piecewise polynomials through corridors using MINCO
5. **Nonlinear Optimization** - Optimize trajectory considering dynamics, collisions, and vehicle limits
6. **Flatness Mapping** - Convert trajectory to thrust, attitude, and body rates for control
7. **Trajectory Sampling** - Sample position, velocity, acceleration at any time point
8. **Root Finding** - Solve polynomials for precise geometric computations
9. **Linear Programming** - Handle polytope geometry operations  
10. **Convex Hull** - Convert half-space representations to vertex form

**Basic Usage Flow:**
1. Create VoxelMap from point cloud or obstacle data
2. Generate initial geometric path using OMPL
3. Build safe flight corridors around the path
4. Initialize polynomial trajectory through corridors
5. Run optimization to refine trajectory
6. Extract final trajectory for drone control

## Input/Output Specifications

### Input Formats

- **Map Data**:
	- Voxel grids (3D binary occupancy)
	- Point clouds (obstacle points)
	- JSON format with obstacles list
- **Endpoints**: 3D coordinates (Eigen::Vector3d)

### Output Data
- **Trajectory Points**: Time-stamped 3D positions
- **Velocity Profile**: 3D velocity vectors over time
- **Timing**: Total trajectory duration
- **Optional**: Acceleration, jerk, control inputs (thrust, attitude)

## Notes
- This is a **wrapper project** - we use GCOPTER's existing algorithms
- The underlying GCOPTER algorithms handle the complex trajectory optimization
- The goal is to make the gcopter library easy to use from Python
- This project does not use ROS 