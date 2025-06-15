#!/usr/bin/env python3

import os
import glob
from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

def find_ompl_include():
    """find the OMPL include directory automatically."""
    # common locations to check
    search_paths = [
        "/opt/homebrew/Cellar/ompl/*/include/ompl-*",
        "/usr/local/include",
        "/usr/include",
    ]
    
    for pattern in search_paths:
        matches = glob.glob(pattern)
        if matches:
            return matches[0]  # return the first match
    
    # fallback - assume it's in a standard location
    return "/opt/homebrew/include"

def find_lib_paths():
    """find library paths for linking."""
    paths = ["/opt/homebrew/lib"]
    if os.path.exists("/usr/local/lib"):
        paths.append("/usr/local/lib")
    return paths

# find dynamic paths
ompl_include = find_ompl_include()
lib_paths = find_lib_paths()

# the main extension module
ext_modules = [
    Pybind11Extension(
        "gcopter_cpp",
        [
            "src/pybindings.cpp",
            "src/drone_pathgen.cpp",
        ],
        # include directories
        include_dirs=[
            "include",
            "/opt/homebrew/include/eigen3",
            "/opt/homebrew/include",
            ompl_include,
        ],
        # library directories  
        library_dirs=lib_paths,
        # libraries to link
        libraries=[
            "ompl",
            "boost_filesystem",
            "boost_serialization", 
            "boost_system",
        ],
        # language standard
        language='c++',
        cxx_std=17,
    ),
]

setup(
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.8",
) 