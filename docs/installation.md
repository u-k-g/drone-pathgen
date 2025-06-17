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

### Option 3: Development Installation

For contributing or development:

```bash
# Clone repository
git clone https://github.com/u-k-g/drone-pathgen.git
cd drone-pathgen

# Install in development mode
pip install -e ".[dev,viz]"
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

## Common Installation Issues

### Missing System Dependencies

**Error:** `ImportError: ... libompl.so not found`

**Solution:** Install OMPL system package:
```bash
# Ubuntu/Debian
sudo apt-get install libompl-dev

# macOS  
brew install ompl
```

### Compiler Issues

**Error:** `error: Microsoft Visual C++ 14.0 is required`

**Solution:** On Windows, install Visual Studio Build Tools or use WSL2 with Ubuntu.

### Python Version Issues

**Error:** `This package requires Python >=3.8,<3.13`

**Solution:** Check your Python version:
```bash
python --version
```

Use a supported Python version or create a virtual environment:
```bash
# Using conda
conda create -n drone-env python=3.11
conda activate drone-env

# Using venv
python3.11 -m venv drone-env
source drone-env/bin/activate  # Linux/macOS
# drone-env\Scripts\activate  # Windows
```

### Memory Issues During Build

**Error:** `c++: internal compiler error: Killed`

**Solution:** Increase available memory or add swap space:
```bash
# Add temporary swap (Linux)
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

## Docker Installation

For containerized environments:

```dockerfile
FROM python:3.11-slim

# Install system dependencies
RUN apt-get update && apt-get install -y \
    libompl-dev \
    libeigen3-dev \
    libboost-all-dev \
    build-essential \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install drone-pathgen
RUN pip install "git+https://github.com/u-k-g/drone-pathgen.git[viz]"

# Test installation
RUN python -c "import gcopter_cpp as gc; print('✅ Installation successful')"
```

Build and run:
```bash
docker build -t drone-pathgen .
docker run -it drone-pathgen python
```

## Virtual Environment Setup

### Using uv (Recommended)

```bash
# Create project with uv
uv init my-drone-project
cd my-drone-project

# Add drone-pathgen dependency
uv add "git+https://github.com/u-k-g/drone-pathgen.git[viz]"

# Run your code
uv run python main.py
```

### Using conda

```bash
# Create environment
conda create -n drone-env python=3.11 numpy matplotlib
conda activate drone-env

# Install system dependencies (if using conda-forge)
conda install -c conda-forge eigen boost-cpp

# Install drone-pathgen
pip install "git+https://github.com/u-k-g/drone-pathgen.git[viz]"
```

### Using virtualenv

```bash
# Create virtual environment
python -m venv drone-env
source drone-env/bin/activate  # Linux/macOS
# drone-env\Scripts\activate   # Windows

# Upgrade pip
pip install --upgrade pip

# Install drone-pathgen
pip install "git+https://github.com/u-k-g/drone-pathgen.git[viz]"
```

## IDE Setup

### VS Code

Add to your `.vscode/settings.json`:

```json
{
    "python.defaultInterpreterPath": "./drone-env/bin/python",
    "python.linting.enabled": true,
    "python.linting.pylintEnabled": true
}
```

### PyCharm

1. File → Settings → Project → Python Interpreter
2. Click gear icon → Add
3. Select your virtual environment interpreter
4. Apply settings

## Performance Optimization

For better performance, consider:

### OpenMP Support

Enable parallel processing:
```bash
# Linux
export OMP_NUM_THREADS=4

# Add to ~/.bashrc for persistence
echo "export OMP_NUM_THREADS=4" >> ~/.bashrc
```

### NumPy BLAS

Ensure NumPy uses optimized BLAS:
```python
import numpy as np
np.show_config()  # Check BLAS configuration
```

Install optimized NumPy:
```bash
# Intel MKL (if available)
pip install mkl numpy

# Or OpenBLAS
pip install openblas numpy
```

## Troubleshooting Build Issues

### Enable Verbose Output

```bash
pip install -v "git+https://github.com/u-k-g/drone-pathgen.git"
```

### Clean Installation

```bash
# Remove cached builds
pip cache purge

# Force reinstall
pip install --force-reinstall --no-cache-dir "git+https://github.com/u-k-g/drone-pathgen.git"
```

### Check System Libraries

```bash
# Linux: Check library availability
ldconfig -p | grep -E "(ompl|eigen|boost)"

# macOS: Check Homebrew installations  
brew list | grep -E "(ompl|eigen|boost)"
```

## Next Steps

After successful installation:

1. Read the [User Guide](user_guide.md) for basic usage
2. Check out [examples/](../examples/) for working code  
3. Review the [API Reference](api_reference.md) for detailed documentation
4. Run `examples/basic_pathgen.py` to test functionality

## Getting Help

If you encounter issues:

1. Check this troubleshooting section
2. Search [GitHub Issues](https://github.com/u-k-g/drone-pathgen/issues)
3. Create a new issue with:
   - Your OS and Python version
   - Complete error message
   - Installation commands used
   - Output of `pip list` or `uv tree`
