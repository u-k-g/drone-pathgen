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
1. **Map Configuration**:
	1. Parse input map data (JSON, binary, or point cloud)
	2. Create `VoxelMap` representation
	3. Store obstacle information

2. **Path Planning** (when endpoints are set):
	1. Use OMPL via `sfc_gen::planPath()` for initial geometric path
	2. Generate Safe Flight Corridor using `sfc_gen::convexCover()`
	3. Create sequence of convex polytopes along path

3. **Trajectory Optimization** (in `runInference()`):
	1. Setup `GCOPTER_PolytopeSFC` with:
		1. Initial/terminal PVA (Position, Velocity, Acceleration)
		2. Safe corridors (polytopes)
		3. Vehicle parameters (mass, drag coefficients, thrust limits)
		4. Optimization weights
	2. Run L-BFGS optimization to generate smooth trajectory
	3. Output `Trajectory<5>` object

4. **Data Extraction**:
	1. Sample trajectory at regular intervals
	2. Extract positions, velocities, accelerations
	3. Optionally convert to control inputs using `FlatnessMap`
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