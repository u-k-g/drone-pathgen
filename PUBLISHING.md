# Publishing to PyPI

This document explains how to publish the drone-pathgen package to PyPI.

## Prerequisites

1. Install publishing tools:
   ```bash
   pip install twine
   ```

2. Create accounts on:
   - [PyPI](https://pypi.org/) (production)
   - [TestPyPI](https://test.pypi.org/) (testing)

3. Generate API tokens for both accounts (recommended over username/password)

## Publishing Process

### Step 1: Update Version

Update the version in `pyproject.toml`:
```toml
[project]
name = "drone-pathgen"
version = "0.1.1"  # Increment this
```

### Step 2: Build Distribution

Clean previous builds and create new ones:
```bash
# Clean previous builds
rm -rf dist/ build/ src/*.egg-info/

# Build source distribution and wheel
uv build

# Verify the build contents
ls -la dist/
```

### Step 3: Test on TestPyPI First

Upload to TestPyPI to verify everything works:
```bash
# Upload to TestPyPI
twine upload --repository testpypi dist/*

# Test installation from TestPyPI
pip install --index-url https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple/ drone-pathgen

# Run the test script
python test_installation.py
```

### Step 4: Publish to PyPI

If TestPyPI works correctly, publish to the main PyPI:
```bash
# Upload to PyPI
twine upload dist/*

# Test installation from PyPI
pip install drone-pathgen

# Verify it works
python test_installation.py
```

## Automation with GitHub Actions

For automated publishing, create `.github/workflows/publish.yml`:

```yaml
name: Publish to PyPI

on:
  release:
    types: [published]

jobs:
  build-and-publish:
    runs-on: macos-latest  # Use macOS for the native build
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Set up Python
      uses: actions/setup-python@v4
      with:
        python-version: '3.8'
    
    - name: Install system dependencies
      run: |
        brew install ompl eigen boost
    
    - name: Install build dependencies
      run: |
        pip install build twine
    
    - name: Build package
      run: |
        python -m build
    
    - name: Publish to PyPI
      env:
        TWINE_USERNAME: __token__
        TWINE_PASSWORD: ${{ secrets.PYPI_API_TOKEN }}
      run: |
        twine upload dist/*
```

## Checklist Before Publishing

- [x] ✅ Package builds successfully (`uv build`)
- [x] ✅ Wheel installs and imports correctly
- [x] ✅ Test script passes (`python test_installation.py`)
- [ ] 📝 Update CHANGELOG.md with new features/fixes
- [ ] 📝 Version number incremented in pyproject.toml
- [ ] 🧪 Tested on TestPyPI
- [ ] 📚 Documentation is up to date
- [ ] 🏷️ Git tag created for the release

## Version Strategy

Follow semantic versioning (SemVer):
- **Major** (1.0.0): Breaking changes
- **Minor** (0.1.0): New features, backward compatible
- **Patch** (0.1.1): Bug fixes, backward compatible

## Troubleshooting

### Build Failures
- Ensure all system dependencies are installed
- Check that the build environment matches the target system
- Verify all source files are included in MANIFEST.in

### Upload Failures
- Check API token permissions
- Ensure version number hasn't been used before
- Verify package name is available (check on PyPI)

### Installation Failures
- Test on multiple Python versions
- Verify dependency specifications in pyproject.toml
- Check that all required shared libraries are available 