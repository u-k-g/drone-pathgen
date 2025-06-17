# API Reference

## Core Classes

### GCopterAPI

The main interface for drone trajectory planning and optimization.

```python
import gcopter_cpp as gc
api = gc.GCopterAPI()
```

#### Methods

##### `configure_map(map_size, origin, voxel_scale, obstacle_points, dilation_radius=1)`

Configures the 3D voxel map environment with obstacles.

**Parameters:**
- `map_size` (np.array): 3D array [width, height, depth] in voxels
- `origin` (np.array): World coordinates of map origin [x, y, z] in meters
- `voxel_scale` (float): Size of each voxel in meters
- `obstacle_points` (list): List of 3D points marking obstacle locations
- `dilation_radius` (int, optional): Safety margin expansion radius. Default: 1

**Example:**
```python
map_size = np.array([30, 30, 15], dtype=np.int32)
origin = np.array([-7.5, -7.5, 0.0])
obstacles = [np.array([0.0, 0.0, 2.0]), np.array([2.0, 2.0, 1.5])]
api.configure_map(map_size, origin, 0.5, obstacles, dilation_radius=2)
```

##### `set_endpoints(start_pos, goal_pos, start_vel=None, goal_vel=None)`

Sets trajectory start and goal positions with optional velocity constraints.

**Parameters:**
- `start_pos` (np.array): Starting position [[x], [y], [z]] in meters
- `goal_pos` (np.array): Goal position [[x], [y], [z]] in meters  
- `start_vel` (np.array, optional): Initial velocity. Default: zero
- `goal_vel` (np.array, optional): Final velocity. Default: zero

**Example:**
```python
start = np.array([[-5.0], [-5.0], [1.5]])
goal = np.array([[5.0], [5.0], [5.0]])
api.set_endpoints(start, goal)
```

##### `run_inference(planning_timeout, time_weight, segment_length, smoothing_epsilon, integral_resolution, magnitude_bounds, penalty_weights, physical_params)`

Executes trajectory planning and optimization.

**Parameters:**
- `planning_timeout` (float): Maximum planning time in seconds
- `time_weight` (float): Objective function time penalty weight
- `segment_length` (float): Target length for trajectory segments
- `smoothing_epsilon` (float): Smoothness tolerance (1e-3 recommended)
- `integral_resolution` (int): Integration resolution for optimization
- `magnitude_bounds` (np.array): [vel_max, acc_max, omega_max, tilt_max, thrust_max]
- `penalty_weights` (np.array): Penalty weights for constraint violations
- `physical_params` (np.array): [mass, gravity, drag_coeffs...] 

**Returns:**
- `bool`: True if optimization succeeded

**Example:**
```python
success = api.run_inference(
    planning_timeout=5.0,
    time_weight=50.0,
    segment_length=2.0,
    smoothing_epsilon=1e-3,
    integral_resolution=8,
    magnitude_bounds=np.array([5., 10., np.pi/3, 5., 15.]),
    penalty_weights=np.array([1, 1, 1, 1, 1]),
    physical_params=np.array([1., 9.81, 0., 0., 0., 0.01])
)
```

##### `get_state_at_time(time, state)`

Retrieves drone kinematic state at a specific time.

**Parameters:**
- `time` (float): Query time in seconds from trajectory start
- `state` (DroneState): Output state object to populate

**Example:**
```python
state = gc.DroneState()
api.get_state_at_time(2.5, state)
print(f"Position: {state.position}")
print(f"Velocity: {state.velocity}")
```

##### `get_control_inputs(time, inputs, yaw=0.0, yaw_rate=0.0)`

Computes control commands (thrust/attitude) for a given time.

**Parameters:**
- `time` (float): Query time in seconds
- `inputs` (ControlInputs): Output control object to populate
- `yaw` (float, optional): Desired yaw angle. Default: 0.0
- `yaw_rate` (float, optional): Desired yaw rate. Default: 0.0

##### `get_statistics(stats)`

Retrieves trajectory performance metrics.

**Parameters:**
- `stats` (TrajectoryStatistics): Output statistics object to populate

**Example:**
```python
stats = gc.TrajectoryStatistics()
api.get_statistics(stats)
print(f"Duration: {stats.total_duration:.2f}s")
print(f"Max velocity: {stats.max_velocity:.2f} m/s")
```

##### `get_visualization_data(show_initial_route=False)`

Extracts data for 3D visualization.

**Parameters:**
- `show_initial_route` (bool, optional): Include initial path. Default: False

**Returns:**
- `tuple`: (success, trajectory_points, voxel_data, voxel_size, start_pos, goal_pos[, initial_route])

##### `print_voxel_map()`

Prints a 2D console visualization of the voxel map.

## Data Structures

### DroneState

Complete kinematic state of the drone at a specific time.

**Attributes:**
- `time` (float): Time from trajectory start [s]
- `position` (np.array): 3D position vector [m]
- `velocity` (np.array): 3D velocity vector [m/s]
- `acceleration` (np.array): 3D acceleration vector [m/s²]
- `jerk` (np.array): 3D jerk vector [m/s³]

### ControlInputs

Control commands for drone actuation.

**Attributes:**
- `thrust` (float): Thrust command [N]
- `quaternion` (np.array): Attitude quaternion [w, x, y, z]
- `angular_velocity` (np.array): Body angular velocity [rad/s]
- `yaw_angle` (float): Yaw angle [rad]
- `yaw_rate` (float): Yaw rate [rad/s]

### TrajectoryStatistics

Performance metrics for computed trajectory.

**Attributes:**
- `total_duration` (float): Total trajectory time [s]
- `num_pieces` (int): Number of polynomial pieces
- `optimization_cost` (float): Final optimization cost
- `max_velocity` (float): Maximum velocity magnitude [m/s]
- `max_acceleration` (float): Maximum acceleration magnitude [m/s²]
- `start_pos` (np.array): Start position [m]
- `goal_pos` (np.array): Goal position [m]

## Parameter Guidelines

### Magnitude Bounds
Recommended values for typical multicopter:
```python
magnitude_bounds = np.array([
    5.0,      # max velocity [m/s]
    10.0,     # max acceleration [m/s²] 
    np.pi/3,  # max angular velocity [rad/s]
    5.0,      # max tilt angle [rad]
    15.0      # max thrust [N]
])
```

### Physical Parameters
Standard quadcopter parameters:
```python
physical_params = np.array([
    1.0,    # mass [kg]
    9.81,   # gravity [m/s²]
    0.0,    # drag coefficient x
    0.0,    # drag coefficient y  
    0.0,    # drag coefficient z
    0.01    # moment of inertia
])
```

### Tuning Tips

- **Higher `time_weight`**: Faster trajectories, less smooth
- **Lower `smoothing_epsilon`**: Smoother paths, slower optimization
- **Higher `segment_length`**: Fewer pieces, less flexible
- **Higher `dilation_radius`**: Larger safety margins around obstacles
