# GCOPTER Python API Wrapper

The GCOPTER Python API Wrapper provides a seamless interface to the GCOPTER C++ trajectory optimization library for multicopters. It enables developers and researchers to integrate high-performance, physics-aware trajectory planning into Python applications without ROS dependencies.

## Key Capabilities

- Configure three-dimensional occupancy grids and populate them with obstacles
- Define start and goal states (position, velocity, acceleration) for the multicopter
- Generate smooth, dynamically feasible trajectories that respect vehicle dynamics and avoid collisions
- Produce safe flight corridors encapsulating the free space around planned paths
- Access detailed trajectory data, including position waypoints, timing allocations, and control commands
- Leverage NumPy-compatible data structures for seamless integration with Python scientific workflows

## Core Features

- High-performance C++ backend utilizing GCOPTER's advanced geometric optimization and MINCO trajectory representation
- Pybind11-based bindings exposing a clean, intuitive Python API
- Standalone library with no external runtime dependencies beyond Eigen and OMPL
- Native support for modern C++ standards and Python 3 environments

## Repository Layout

- `include/` — Header-only GCOPTER components and utilities
- `src/` — C++ API implementation and Python binding code
- `test/` — Minimal example and environment validation harness
- `pyproject.toml` — Python project configuration managed by uv
- `CMakeLists.txt` — Build configuration for compiling the C++ library and bindings

## License

This project is released under the MIT License, in accordance with the original GCOPTER library license.
