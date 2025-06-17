# Installation Guide

## Prerequisites

Before installing `drone-pathgen`, ensure you have the required system dependencies:

### macOS (Homebrew)

```bash
brew install ompl eigen boost
```

### Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install libompl-dev libeigen3-dev libboost-all-dev
```

### System Requirements

- **Python**: 3.8 - 3.12
- **OS**: Linux, macOS  
- **Compiler**: C++17 compatible (GCC 7+, Clang 8+)

## Installation Options

### Option 1: Core Library Only

For basic trajectory planning functionality:

```bash
# Using uv (recommended)
uv add git+https://github.com/u-k-g/drone-pathgen.git

# Using pip
pip install git+https://github.com/u-k-g/drone-pathgen.git
```

### Option 2: With Visualization Support

For 3D visualization capabilities (adds Open3D dependency):

```bash
# Using uv
uv add "git+https://github.com/u-k-g/drone-pathgen.git[viz]"

# Using pip  
pip install "git+https://github.com/u-k-g/drone-pathgen.git[viz]"
```

## Verification

Test your installation:

```python
import gcopter_cpp as gc
import numpy as np

# Create API instance
api = gc.GCopterAPI()
print("✅ drone-pathgen installed successfully!")

# Test basic functionality
map_size = np.array([10, 10, 5], dtype=np.int32)
origin = np.array([0.0, 0.0, 0.0])
obstacles = [np.array([2.0, 2.0, 1.0])]

api.configure_map(map_size, origin, 0.5, obstacles)
print("✅ Library functions working correctly!")
```

## Virtual Environment Setup

### Using uv

```bash
# Create project with uv
uv init my-drone-project
cd my-drone-project

# Add drone-pathgen dependency
uv add "git+https://github.com/u-k-g/drone-pathgen.git[viz]"

# Run your code
uv run python main.py
```

## Next

After successful installation:

1. Read the [User Guide](user_guide.md) for basic usage

2. Review the [API Reference](api_reference.md) for detailed documentation
3. Check out [examples/](../examples/) for working code  
