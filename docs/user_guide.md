# User Guide

## Getting Started

`drone-pathgen` is a Python wrapper around the GCOPTER trajectory optimizer, designed for planning smooth, collision-free drone paths in 3D environments.

### Basic Workflow

Every trajectory planning task follows these core steps:

1. **Create API instance** - Initialize the planning interface
2. **Configure environment** - Set up 3D voxel map with obstacles  
3. **Define endpoints** - Specify start and goal positions
4. **Run optimization** - Execute trajectory planning
5. **Query results** - Extract trajectory data and statistics

## Quick Start Example

```python
import numpy as np
import gcopter_cpp as gc

# 1. Create API instance
api = gc.GCopterAPI()

# 2. Configure 3D environment
map_size = np.array([20, 20, 10], dtype=np.int32)  # 20x20x10 voxel grid
origin = np.array([-5.0, -5.0, 0.0])               # world origin
voxel_scale = 0.5                                   # 0.5m per voxel

obstacles = [
    np.array([0.0, 0.0, 2.0]),    # central pillar
    np.array([2.0, -2.0, 1.5]),  # side obstacle
]

api.configure_map(map_size, origin, voxel_scale, obstacles, dilation_radius=1)

# 3. Set start and goal positions  
start_pos = np.array([[-4.0], [-4.0], [1.0]])
goal_pos = np.array([[4.0], [4.0], [3.0]])
api.set_endpoints(start_pos, goal_pos)

# 4. Run trajectory optimization
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

# 5. Query results
if success:
    stats = gc.TrajectoryStatistics()
    api.get_statistics(stats)
    print(f"✅ Success! Duration: {stats.total_duration:.2f}s")
    
    # Sample trajectory at midpoint
    state = gc.DroneState()
    api.get_state_at_time(stats.total_duration * 0.5, state)
    print(f"Midpoint position: {state.position}")
else:
    print("❌ Planning failed")
```

## Environment Setup

### Voxel Maps

The library uses 3D voxel grids to represent the environment:

```python
# Define map dimensions
map_size = np.array([width, height, depth], dtype=np.int32)  # in voxels
origin = np.array([x_min, y_min, z_min])                     # world coordinates  
voxel_scale = 0.5  # meters per voxel

# Each voxel represents a 0.5m³ cube in this example
```

### Adding Obstacles

Obstacles are specified as 3D points in world coordinates:

```python
obstacles = [
    np.array([x1, y1, z1]),  # obstacle center 1
    np.array([x2, y2, z2]),  # obstacle center 2  
    # ... more obstacles
]

# Safety margins are automatically added based on dilation_radius
api.configure_map(map_size, origin, voxel_scale, obstacles, dilation_radius=2)
```

**Dilation Radius**: Expands obstacles by N voxels to create safety margins. Higher values = safer but more constrained paths.

### Complex Environments

For detailed environments, create obstacle patterns:

```python
# Create a corridor with pillars
obstacles = []

# Add pillars along corridor
for i in range(5):
    x = i * 3.0 - 6.0  # space pillars 3m apart
    obstacles.extend([
        np.array([x, -1.5, 2.0]),  # left pillar
        np.array([x, 1.5, 2.0]),   # right pillar  
        np.array([x, -1.5, 3.0]),  # pillar top
        np.array([x, 1.5, 3.0]),
    ])

# Add ceiling obstacles
for x in np.linspace(-8, 8, 10):
    for y in np.linspace(-3, 3, 5):
        obstacles.append(np.array([x, y, 6.0]))
```

## Trajectory Planning

### Parameter Tuning

The `run_inference()` parameters control optimization behavior:

**Planning Parameters:**
- `planning_timeout`: Maximum solve time (increase for complex environments)
- `time_weight`: Balance between speed and smoothness (higher = faster paths)
- `segment_length`: Trajectory piece length (shorter = more flexible)

**Quality Parameters:**  
- `smoothing_epsilon`: Smoothness tolerance (1e-3 to 1e-6 typical)
- `integral_resolution`: Integration accuracy (8-16 recommended)

**Physical Constraints:**
```python
magnitude_bounds = np.array([
    max_velocity,      # [m/s] 
    max_acceleration,  # [m/s²]
    max_angular_vel,   # [rad/s]
    max_tilt_angle,    # [rad]
    max_thrust         # [N]
])
```

### Common Parameter Sets

**Fast Planning** (for real-time applications):
```python
success = api.run_inference(
    planning_timeout=2.0,      # quick solve
    time_weight=100.0,         # prioritize speed
    segment_length=3.0,        # fewer pieces
    smoothing_epsilon=1e-2,    # less smoothness
    integral_resolution=6,     # faster integration
    # ... other params
)
```

**High Quality** (for film/precision work):
```python
success = api.run_inference(
    planning_timeout=30.0,     # allow longer solve
    time_weight=10.0,          # prioritize smoothness  
    segment_length=1.0,        # more pieces
    smoothing_epsilon=1e-5,    # very smooth
    integral_resolution=12,    # high accuracy
    # ... other params
)
```

## Working with Results

### Trajectory Sampling

Sample the trajectory at any time point:

```python
if success:
    stats = gc.TrajectoryStatistics()
    api.get_statistics(stats)
    
    # Sample at regular intervals
    n_samples = 100
    times = np.linspace(0, stats.total_duration, n_samples)
    
    positions = []
    velocities = []
    
    state = gc.DroneState()
    for t in times:
        api.get_state_at_time(t, state)
        positions.append(state.position.copy())
        velocities.append(state.velocity.copy())
    
    positions = np.array(positions)  # shape: (n_samples, 3)
    velocities = np.array(velocities)
```

### Control Commands

Get thrust and attitude commands for flight controllers:

```python
controls = gc.ControlInputs()
for t in times:
    api.get_control_inputs(t, controls, yaw=0.0, yaw_rate=0.0)
    
    # Send to flight controller
    thrust = controls.thrust
    quaternion = controls.quaternion  # [w, x, y, z]
    angular_velocity = controls.angular_velocity
```

### Performance Analysis

```python
stats = gc.TrajectoryStatistics()
api.get_statistics(stats)

print(f"Total time: {stats.total_duration:.2f}s")
print(f"Max velocity: {stats.max_velocity:.2f} m/s") 
print(f"Max acceleration: {stats.max_acceleration:.2f} m/s²")
print(f"Trajectory pieces: {stats.num_pieces}")
print(f"Optimization cost: {stats.optimization_cost:.4f}")
```

## Visualization

For 3D visualization with Open3D:

```python
# Install with visualization support
# pip install "git+https://github.com/u-k-g/drone-pathgen.git[viz]"

import open3d as o3d

# Get visualization data
result = api.get_visualization_data(show_initial_route=True)
success, trajectory_points, voxel_data, voxel_size, start, goal, initial_route = result

if success:
    # Create trajectory line
    trajectory_line = o3d.geometry.LineSet()
    trajectory_line.points = o3d.utility.Vector3dVector(trajectory_points)
    lines = [[i, i + 1] for i in range(len(trajectory_points) - 1)]
    trajectory_line.lines = o3d.utility.Vector2iVector(lines)
    trajectory_line.paint_uniform_color([0.2, 0.5, 0.9])  # blue
    
    # Add start/goal markers
    start_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.3)
    start_sphere.translate(start)
    start_sphere.paint_uniform_color([0.2, 0.8, 0.2])  # green
    
    goal_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.3)  
    goal_sphere.translate(goal)
    goal_sphere.paint_uniform_color([0.8, 0.2, 0.8])  # purple
    
    # Launch viewer
    o3d.visualization.draw_geometries([trajectory_line, start_sphere, goal_sphere])
```

See `examples/visualization.py` for complete visualization code including obstacles and safety margins.

## Troubleshooting

### Common Issues

**Planning Fails:**
- Check that start/goal positions are not inside obstacles
- Increase `planning_timeout` for complex environments  
- Reduce `dilation_radius` if path is over-constrained
- Verify `magnitude_bounds` are reasonable for your drone

**Jerky Trajectories:**
- Decrease `smoothing_epsilon` (e.g., 1e-4 → 1e-5)
- Reduce `time_weight` to prioritize smoothness
- Decrease `segment_length` for more flexible paths

**Slow Performance:**
- Reduce map resolution (increase `voxel_scale`)
- Decrease `integral_resolution` (e.g., 12 → 8)
- Increase `segment_length` for fewer trajectory pieces
- Set stricter `planning_timeout`

### Debug Tools

Print voxel map to console:
```python
api.print_voxel_map()  # Shows 2D projection
```

Validate environment setup:
```python
# Check if start/goal are valid
result = api.get_visualization_data()
success, _, voxel_data, _, start, goal, _ = result

if success:
    print(f"Start: {start}, Goal: {goal}")
    print(f"Map size: {np.array(voxel_data).shape}")
```

## Next

1. Read the [API Reference](api_reference.md) for detailed documentation

2. Check out [examples/](../examples/) for working code  

