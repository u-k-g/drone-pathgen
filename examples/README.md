# Examples

This directory contains practical examples demonstrating how to use the `drone-pathgen` library for various trajectory planning scenarios.

## Available Examples

### 1. Basic Path Generation (`basic_pathgen.py`)

**Purpose**: Demonstrates the core trajectory planning workflow from start to finish.

**What it shows:**
- Setting up a 3D voxel environment with obstacles
- Configuring start and goal positions
- Running trajectory optimization
- Querying results and statistics

**Key concepts:**
- Map configuration with obstacle dilation
- Parameter tuning for optimization
- Trajectory statistics analysis
- State sampling at specific times

**Run it:**
```bash
python examples/basic_pathgen.py
```

**Expected output:**
```
🚁 basic path generation
📍 configuring map with obstacles...
🎯 planning from [-5. -5.  1.5] to [5. 5. 5.]
⚡ running trajectory optimization...
✅ trajectory optimization successful!
   duration: 3.45s
   max velocity: 4.23 m/s
   segments: 7
   position at t=1.73s: [0.15, 0.28, 3.12]
   velocity at t=1.73s: [2.87, 2.91, 1.45]
```

### 2. 3D Visualization (`visualization.py`)

**Purpose**: Shows how to create interactive 3D visualizations of planned trajectories.

**What it shows:**
- Reusing trajectory planning from `basic_pathgen.py`
- Extracting visualization data from the API
- Building 3D meshes for obstacles and safety margins
- Creating trajectory lines and waypoint markers
- Launching Open3D viewer

**Key concepts:**
- Visualization data extraction
- Open3D geometry creation
- Voxel mesh building for obstacles
- Color coding for different elements

**Prerequisites:**
```bash
# Install with visualization support
uv add "git+https://github.com/u-k-g/drone-pathgen.git[viz]"
# or
pip install "git+https://github.com/u-k-g/drone-pathgen.git[viz]"
```

**Run it:**
```bash
python examples/visualization.py
```

**Expected output:**
```
🚁 trajectory visualization example
🚁 basic path generation
📍 configuring map with obstacles...
🎯 planning from [-5. -5.  1.5] to [5. 5. 5.]
⚡ running trajectory optimization...
✅ trajectory optimization successful!
✅ trajectory generated successfully!
🎨 building 3d visualization...
🚀 launching open3d viewer...
   🟢 green sphere = start
   🟣 purple sphere = goal
   🔴 red cubes = obstacles
   🟡 yellow cubes = safety margins
   🔵 blue line = optimized trajectory
   ⚪ gray line = initial route
```

**Visualization controls:**
- **Mouse drag**: Rotate view
- **Mouse wheel**: Zoom in/out
- **Shift + mouse drag**: Pan view
- **R**: Reset view
- **ESC**: Close window

## Running the Examples

### Prerequisites

1. **Install drone-pathgen:**
   ```bash
   # Basic installation
   pip install git+https://github.com/u-k-g/drone-pathgen.git
   
   # With visualization support (for visualization.py)
   pip install "git+https://github.com/u-k-g/drone-pathgen.git[viz]"
   ```

2. **System dependencies** (see [installation guide](../docs/installation.md)):
   ```bash
   # macOS
   brew install ompl eigen boost
   
   # Ubuntu/Debian
   sudo apt-get install libompl-dev libeigen3-dev libboost-all-dev
   ```

### Quick Test

Run the basic example to verify your installation:

```bash
cd examples/
python basic_pathgen.py
```

If successful, you should see trajectory planning output with statistics.

## Understanding the Examples

### Environment Setup

Both examples use the same environment setup pattern:

```python
# Define 3D voxel grid
map_size = np.array([30, 30, 15], dtype=np.int32)  # width × height × depth in voxels
origin = np.array([-7.5, -7.5, 0.0])               # world origin coordinates
voxel_scale = 0.5                                   # meters per voxel

# Define obstacles as 3D points
obstacles = [
    np.array([0.0, 0.0, 2.0]),    # central pillar
    np.array([-2.0, -2.0, 1.5]),  # corner obstacles
    # ... more obstacles
]

# Configure map with safety dilation
api.configure_map(map_size, origin, voxel_scale, obstacles, dilation_radius=2)
```

### Parameter Explanation

The optimization parameters in the examples are chosen for demonstration:

```python
success = api.run_inference(
    planning_timeout=5.0,         # Max solve time [seconds]
    time_weight=50.0,             # Speed vs smoothness balance
    segment_length=2.0,           # Trajectory piece length [meters]
    smoothing_epsilon=1e-3,       # Smoothness tolerance
    integral_resolution=8,        # Integration accuracy
    magnitude_bounds=np.array([   # Physical limits:
        5.,      # max velocity [m/s]
        10.,     # max acceleration [m/s²]
        np.pi/3, # max angular velocity [rad/s]
        5.,      # max tilt angle [rad]
        15.      # max thrust [N]
    ]),
    penalty_weights=np.array([1, 1, 1, 1, 1]),  # Constraint penalties
    physical_params=np.array([    # Drone parameters:
        1.,      # mass [kg]
        9.81,    # gravity [m/s²]
        0., 0., 0.,  # drag coefficients
        0.01     # moment of inertia
    ])
)
```

## Extending the Examples

### Modify Environment

Try different obstacle configurations:

```python
# Create a corridor
obstacles = []
for i in range(5):
    x = i * 2.0 - 4.0
    obstacles.extend([
        np.array([x, -2.0, 2.0]),  # left wall
        np.array([x, 2.0, 2.0]),   # right wall
    ])

# Create random obstacles
import random
obstacles = []
for _ in range(10):
    x = random.uniform(-6, 6)
    y = random.uniform(-6, 6)
    z = random.uniform(1, 4)
    obstacles.append(np.array([x, y, z]))
```

### Adjust Planning Parameters

Experiment with different optimization settings:

```python
# Fast planning (less smooth)
success = api.run_inference(
    planning_timeout=2.0,
    time_weight=100.0,      # Prioritize speed
    segment_length=3.0,     # Fewer pieces
    smoothing_epsilon=1e-2, # Less smoothness
    # ... other params
)

# High-quality planning (very smooth)
success = api.run_inference(
    planning_timeout=15.0,
    time_weight=10.0,       # Prioritize smoothness
    segment_length=1.0,     # More pieces
    smoothing_epsilon=1e-5, # Very smooth
    # ... other params
)
```

### Add Trajectory Analysis

```python
if success:
    # Sample trajectory densely
    stats = gc.TrajectoryStatistics()
    api.get_statistics(stats)
    
    times = np.linspace(0, stats.total_duration, 100)
    positions = []
    velocities = []
    
    state = gc.DroneState()
    for t in times:
        api.get_state_at_time(t, state)
        positions.append(state.position.copy())
        velocities.append(state.velocity.copy())
    
    positions = np.array(positions)
    velocities = np.array(velocities)
    
    # Analysis
    speeds = np.linalg.norm(velocities, axis=1)
    print(f"Average speed: {np.mean(speeds):.2f} m/s")
    print(f"Path length: {np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1)):.2f} m")
```

## Next Steps

After running these examples:

1. **Read the [User Guide](../docs/user_guide.md)** for detailed concepts
2. **Check the [API Reference](../docs/api_reference.md)** for all available methods
3. **Browse [docs/examples.md](../docs/examples.md)** for more advanced scenarios
4. **Experiment** with your own environments and parameters

## Troubleshooting

### Import Errors

**Problem**: `ImportError: No module named 'gcopter_cpp'`

**Solution**: Install the library:
```bash
pip install git+https://github.com/u-k-g/drone-pathgen.git
```

### Visualization Issues

**Problem**: `ImportError: No module named 'open3d'`

**Solution**: Install with visualization support:
```bash
pip install "git+https://github.com/u-k-g/drone-pathgen.git[viz]"
```

### Planning Failures

**Problem**: `❌ trajectory optimization failed!`

**Solutions**:
- Increase `planning_timeout` (e.g., 5.0 → 10.0)
- Reduce `dilation_radius` (e.g., 2 → 1)
- Check that start/goal positions are not inside obstacles
- Relax `magnitude_bounds` constraints

### System Dependencies

**Problem**: Library fails to load or compile

**Solution**: Install system dependencies:
```bash
# macOS
brew install ompl eigen boost

# Ubuntu/Debian
sudo apt-get install libompl-dev libeigen3-dev libboost-all-dev
```

See the [Installation Guide](../docs/installation.md) for detailed troubleshooting.
