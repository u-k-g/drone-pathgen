# drone-pathgen

<details open>
<summary><strong>overview</strong></summary>

`drone-pathgen` is a lightweight python wrapper around **gcopter** – a trajectory optimizer for multicopters. it lets you generate smooth, collision-free drone paths directly from python swiftly and painlessly.

<details>
<summary><strong>applications</strong></summary>

- autonomous delivery
- search & rescue navigation
- infrastructure inspection
- simulation & research prototypes
- indoor warehouse flight planning

</details>
<details open>
<summary><strong>features</strong></summary>

- smart path planning that avoids obstacles and respects real drone physics
- voxel-map support for arbitrary 3d environments
- smooth, jerk-limited trajectories within safe-flight-corridors (sfc)
- built-in optimization for time, energy and feasibility
- numpy-friendly api – results come back as plain ndarrays
- optional open3d visualization extras [see screenshot below]

</details>
<img width="400" alt="image" src="https://github.com/user-attachments/assets/d2c40804-ac8f-440d-a3db-ab63a1d49f26" />
</details>




<details open>
<summary><strong>installation</strong></summary>

prerequisites – make sure **ompl**, **eigen3** and **boost** are available on your system.

```fish
# macos [homebrew]
brew install ompl eigen boost

# ubuntu / debian
sudo apt-get update
sudo apt-get install libompl-dev libeigen3-dev libboost-all-dev
```

install the library:
<details>
<summary><strong>core only</strong></summary>

```fish
uv add git+https://github.com/u-k-g/drone-pathgen.git
# or with pip 
pip install git+https://github.com/u-k-g/drone-pathgen.git
```

</details>

<details>
<summary><strong>with visualization support [adds open3d]</strong></summary>

```fish
uv add "git+https://github.com/u-k-g/drone-pathgen.git[viz]"
# or with pip
pip install "git+https://github.com/u-k-g/drone-pathgen.git[viz]"
```

</details>

supported python versions [3.8 → 3.12]

</details>

<details open>
<summary><strong>quick start</strong></summary>

```python
import numpy as np
import gcopter_cpp as gc

# create api and configure environment
api = gc.GCopterAPI()
api.configure_map(
    map_size=np.array([20, 20, 10], dtype=np.int32),
    origin=np.array([-5.0, -5.0, 0.0]),
    voxel_scale=0.5,
    obstacle_points=[np.array([0.0, 0.0, 2.0])],
    dilation_radius=2
)

# set trajectory endpoints
start = np.array([[-4.0], [-4.0], [1.0]])
goal = np.array([[4.0], [4.0], [3.0]])
api.set_endpoints(start, goal)

# plan trajectory
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

# get results
if success:
    stats = gc.TrajectoryStatistics()
    api.get_statistics(stats)
    print(f"✅ planned {stats.total_duration:.2f}s trajectory")
```

</details>

<details open>
<summary><strong>documentation</strong></summary>

📚 **comprehensive documentation available in [`docs/`](docs/)**

- **[installation guide](docs/installation.md)** - system setup and troubleshooting
- **[user guide](docs/user_guide.md)** - getting started and core concepts
- **[api reference](docs/api_reference.md)** - complete method documentation
- **[examples](docs/examples.md)** - 9 practical code examples and use cases

📁 **working examples in [`examples/`](examples/)**

- [`examples/basic_pathgen.py`](examples/basic_pathgen.py) - core trajectory planning workflow
- [`examples/visualization.py`](examples/visualization.py) - 3d visualization with open3d

</details>



<details>
<summary><strong>license</strong></summary>

released under the mit license – see `license` file for details.

</details>

