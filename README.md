# GCOPTER python wrapper

<details>
<summary><strong>overview</strong></summary>

`drone-pathgen` is a lightweight python wrapper around **gcopter** – a state-of-the-art trajectory optimizer for multicopters. it lets you generate smooth, collision-free drone paths directly from python while keeping the heavy number-crunching in c++.

</details>

<details>
<summary><strong>features</strong></summary>

- smart path planning that avoids obstacles and respects real drone physics
- voxel-map support for arbitrary 3d environments
- smooth, jerk-limited trajectories within safe-flight-corridors (sfc)
- built-in optimization for time, energy and feasibility
- numpy-friendly api – results come back as plain ndarrays
- optional open3d visualization extras

</details>

<details>
<summary><strong>installation</strong></summary>

prerequisites – make sure **ompl**, **eigen3** and **boost** are available on your system.

```bash
# macos (homebrew)
brew install ompl eigen boost

# ubuntu / debian
sudo apt-get update
sudo apt-get install libompl-dev libeigen3-dev libboost-all-dev
```

install the library (core only):

```bash
uv add git+https://github.com/u-k-g/drone-pathgen.git
```

install with visualization support (adds open3d):

```bash
uv add "git+https://github.com/u-k-g/drone-pathgen.git[viz]"
```

supported python versions:

- core functionality: 3.8 → 3.13
- with visualization: 3.8 → 3.12 (open3d limitation)

</details>

<details>
<summary><strong>quick test</strong></summary>

```python
import gcopter_cpp
api = gcopter_cpp.GCopterAPI()
print("✅ gcopter wrapper loaded!")
```

</details>

<details>
<summary><strong>python api usage</strong></summary>

```python
import numpy as np
import gcopter_cpp as gc

api = gc.GCopterAPI()

# 1. build a voxel map
map_size     = np.array([20, 20, 10], dtype=np.int32)
origin       = np.array([-5.0, -5.0, 0.0])
voxel_scale  = 0.5  # metres per voxel
obstacles    = [np.array([0.0, 0.0, 1.0]), np.array([-1., -1., 1.]), np.array([1., 1., 1.])]
api.configure_map(map_size, origin, voxel_scale, obstacles, dilation_radius=2)

# 2. define endpoints
api.set_endpoints(start=np.array([-3., -3., 1.]), goal=np.array([3., 3., 1.]))

# 3. optimisation parameters
success = api.run_inference(
    planning_timeout   = 5.0,
    time_weight        = 50.0,
    segment_length     = 2.0,
    smoothing_epsilon  = 1e-3,
    integral_resolution= 8,
    magnitude_bounds   = np.array([5., 10., np.pi/3, 5., 15.]),
    penalty_weights    = np.array([1, 1, 1, 1, 1]),
    physical_params    = np.array([1., 9.81, 0., 0., 0., 0.01])
)

if success:
    stats = gc.TrajectoryStatistics()
    api.get_statistics(stats)
    print(f"trajectory duration: {stats.total_duration:.2f}s")
```

</details>

<details>
<summary><strong>key concepts</strong></summary>

- **voxel map** – 3d occupancy grid representing obstacles
- **safe-flight-corridor (sfc)** – chain of convex polytopes that guarantee clearance
- **trajectory optimisation** – polynomial path refined to satisfy dynamics & safety

</details>

<details>
<summary><strong>applications</strong></summary>

- autonomous delivery
- search & rescue navigation
- infrastructure inspection
- simulation & research prototypes
- indoor warehouse flight planning

</details>

<details>
<summary><strong>license</strong></summary>

released under the mit license – see `license` file for details.

</details>

