# GCOPTER python wrapper

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
- optional open3d visualization extras [see screenshot in overview]

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
<summary><strong>usage</strong></summary>

see the examples folder for complete working demos:

- [`examples/basic_pathgen.py`](examples/basic_pathgen.py) - core trajectory planning workflow
- [`examples/visualization.py`](examples/visualization.py) - 3d visualization with open3d

basic workflow:
```python
import gcopter_cpp as gc
api = gc.GCopterAPI()
api.configure_map(map_size, origin, voxel_scale, obstacles, dilation_radius)
api.set_endpoints(start_pos, goal_pos)
success = api.run_inference(...)
```

</details>



<details>
<summary><strong>license</strong></summary>

released under the mit license – see `license` file for details.

</details>

