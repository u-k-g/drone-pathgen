# drone-pathgen directory structure

```fish
❯ eza --tree
CMakeLists.txt
examples
├── basic_pathgen.py
└── visualization.py
include
├── api
│   └── gcopter_api.hpp
└── gcopter
    ├── firi.hpp
    ├── flatness.hpp
    ├── gcopter.hpp
    ├── geo_utils.hpp
    ├── lbfgs.hpp
    ├── minco.hpp
    ├── quickhull.hpp
    ├── root_finder.hpp
    ├── sdlp.hpp
    ├── sfc_gen.hpp
    ├── trajectory.hpp
    ├── voxel_dilater.hpp
    └── voxel_map.hpp
LICENSE
MANIFEST.in
pyproject.toml
README.md
setup.py
src
├── drone_pathgen.cpp
└── pybindings.cpp
```
