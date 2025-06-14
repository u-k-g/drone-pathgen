#!/usr/bin/env python3
"""
Simple test script to verify drone-pathgen installation.
Run this after installing the package to ensure everything works correctly.
"""

import numpy as np

def test_installation():
    """Test basic functionality of the drone-pathgen package."""
    print("🧪 Testing drone-pathgen installation...")
    
    try:
        # test import
        import gcopter_cpp
        print("✅ Successfully imported gcopter_cpp module")
        
        # test api creation
        api = gcopter_cpp.GCopterAPI()
        print("✅ Successfully created GCopterAPI instance")
        
        # test data structures
        state = gcopter_cpp.DroneState()
        inputs = gcopter_cpp.ControlInputs()
        stats = gcopter_cpp.TrajectoryStatistics()
        print("✅ Successfully created data structures")
        
        # test basic map configuration
        map_size = np.array([10, 10, 5], dtype=np.int32)
        origin = np.array([0.0, 0.0, 0.0])
        voxel_scale = 1.0
        obstacles = [np.array([2.0, 2.0, 1.0])]
        
        api.configure_map(map_size, origin, voxel_scale, obstacles, 1)
        print("✅ Successfully configured map")
        
        # test endpoint setting
        start_pos = np.array([1.0, 1.0, 1.0])
        goal_pos = np.array([8.0, 8.0, 1.0])
        api.set_endpoints(start_pos, goal_pos)
        print("✅ Successfully set endpoints")
        
        print("\n🎉 All tests passed! Your installation is working correctly.")
        print("📖 See the documentation for complete usage examples.")
        return True
        
    except ImportError as e:
        print(f"❌ Import failed: {e}")
        print("💡 Make sure you have installed the package: pip install drone-pathgen")
        return False
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        print("💡 There might be missing system dependencies (OMPL, Eigen, Boost)")
        return False

if __name__ == "__main__":
    success = test_installation()
    exit(0 if success else 1) 