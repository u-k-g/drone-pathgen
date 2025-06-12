#!/usr/bin/env python3
"""
Simple test script to verify the GCOPTER Python API works.
"""
import sys
import numpy as np

# add the build directory to path to import our module
sys.path.append('build')
import gcopter_cpp

def test_basic_api():
    """Test basic API functionality"""
    print("🔧 Creating GCopterAPI instance...")
    api = gcopter_cpp.GCopterAPI()
    
    # create a simple 3d map with a few obstacles
    map_size = np.array([20, 20, 10], dtype=np.int32)  # 20x20x10 voxel grid
    origin = np.array([-5.0, -5.0, 0.0])  # origin at (-5,-5,0)
    voxel_scale = 0.5  # 0.5m per voxel
    # map goes from (-5,-5,0) to (5,5,5) in world coordinates
    
    # add some obstacles within the map bounds
    obstacles = [
        np.array([0.0, 0.0, 1.0]),   # center obstacle
        np.array([1.0, 1.0, 1.0]),   # obstacle to avoid
        np.array([-1.0, -1.0, 1.0]), # another obstacle
    ]
    
    print("🗺️  Configuring map...")
    api.configure_map(map_size, origin, voxel_scale, obstacles, 1)
    
    # set start and goal positions (within map bounds)
    start_pos = np.array([-3.0, -3.0, 1.0])  # within bounds
    goal_pos = np.array([3.0, 3.0, 1.0])     # within bounds
    
    print("🎯 Setting endpoints...")
    api.set_endpoints(start_pos, goal_pos)
    
    # set up optimization parameters
    planning_timeout = 5.0  # 5 seconds for path planning
    time_weight = 50.0      # weight on total flight time
    segment_length = 2.0    # corridor segment length
    smoothing_epsilon = 1e-3
    integral_resolution = 8
    
    # magnitude bounds: [v_max, omega_max, theta_max, thrust_min, thrust_max]
    magnitude_bounds = np.array([5.0, 10.0, np.pi/3, 5.0, 15.0])
    
    # penalty weights: [pos, vel, omega, theta, thrust]
    penalty_weights = np.array([1.0, 1.0, 1.0, 1.0, 1.0])
    
    # physical params: [mass, grav, horiz_drag, vert_drag, parasitic_drag, speed_smooth]
    physical_params = np.array([1.0, 9.81, 0.0, 0.0, 0.0, 0.01])
    
    print("🚁 Running trajectory optimization...")
    success = api.run_inference(
        planning_timeout, time_weight, segment_length,
        smoothing_epsilon, integral_resolution,
        magnitude_bounds, penalty_weights, physical_params
    )
    
    if success:
        print("✅ Trajectory optimization successful!")
        
        # test getting trajectory statistics
        stats = gcopter_cpp.TrajectoryStatistics()
        if api.get_statistics(stats):
            print(f"📊 Trajectory Statistics:")
            print(f"   Total Duration: {stats.total_duration:.2f}s")
            print(f"   Number of Pieces: {stats.num_pieces}")
            print(f"   Optimization Cost: {stats.optimization_cost:.4f}")
            print(f"   Max Velocity: {stats.max_velocity:.2f} m/s")
            print(f"   Max Acceleration: {stats.max_acceleration:.2f} m/s²")
        
        # test getting state at a specific time
        state = gcopter_cpp.DroneState()
        test_time = stats.total_duration * 0.5  # halfway through trajectory
        if api.get_state_at_time(test_time, state):
            print(f"🎯 State at t={test_time:.2f}s:")
            print(f"   Position: [{state.position[0]:.2f}, {state.position[1]:.2f}, {state.position[2]:.2f}]")
            print(f"   Velocity: [{state.velocity[0]:.2f}, {state.velocity[1]:.2f}, {state.velocity[2]:.2f}]")
        
        # test getting visualization data
        trajectory_points = []
        voxel_data = []
        voxel_size = 0.0
        start_vis = np.zeros(3)
        goal_vis = np.zeros(3)
        
        if api.get_visualization_data(trajectory_points, voxel_data, voxel_size, start_vis, goal_vis):
            print(f"📈 Visualization data extracted:")
            print(f"   Trajectory points: {len(trajectory_points)}")
            print(f"   Voxel size: {voxel_size}")
            if len(voxel_data) > 0 and len(voxel_data[0]) > 0:
                print(f"   Voxel grid shape: {len(voxel_data)}x{len(voxel_data[0])}x{len(voxel_data[0][0])}")
            else:
                print("   Voxel data: empty")
        
        return True
    else:
        print("❌ Trajectory optimization failed!")
        return False

if __name__ == "__main__":
    print("🧪 Testing GCOPTER Python API...")
    if test_basic_api():
        print("🎉 All tests passed!")
    else:
        print("💥 Tests failed!")
        sys.exit(1) 