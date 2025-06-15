#!/usr/bin/env python3
"""
basic path generation example

demonstrates core trajectory planning workflow:
1. build voxel map with obstacles
2. set start/goal endpoints  
3. run trajectory optimization
4. query results
"""

import numpy as np
import gcopter_cpp as gc

def main():
    print("🚁 basic path generation example")
    
    # create api instance
    api = gc.GCopterAPI()

    # 1. build a voxel map
    map_size     = np.array([20, 20, 10], dtype=np.int32)
    origin       = np.array([-5.0, -5.0, 0.0])
    voxel_scale  = 0.5  # metres per voxel
    obstacles    = [
        np.array([0.0, 0.0, 1.0]), 
        np.array([-1., -1., 1.]), 
        np.array([1., 1., 1.])
    ]
    
    print("📍 configuring map with obstacles...")
    api.configure_map(map_size, origin, voxel_scale, obstacles, dilation_radius=2)

    # 2. define endpoints
    start_pos = np.array([[-5.0], [-5.0], [1.5]])
    goal_pos  = np.array([[5.0], [5.0], [3.0]])
    api.set_endpoints(start_pos, goal_pos)
    print(f"🎯 planning from {start_pos} to {goal_pos}")

    # 3. optimization parameters
    print("⚡ running trajectory optimization...")
    success = api.run_inference(
        planning_timeout   = 5.0,
        time_weight        = 50.0,
        segment_length     = 2.0,
        smoothing_epsilon  = 1e-3,
        integral_resolution= 8,
        magnitude_bounds   = np.array([5., 10., np.pi/3, 5., 15.]),
        penalty_weights    = np.array([1, 1, 1, 1, 1]),
        physical_params    = np.array([1., 9.81, 0., 0., 0., 0.01])
    )

    # 4. get results
    if success:
        print("✅ trajectory optimization successful!")
        
        # get trajectory statistics
        stats = gc.TrajectoryStatistics()
        api.get_statistics(stats)
        print(f"   duration: {stats.total_duration:.2f}s")
        print(f"   max velocity: {stats.max_velocity:.2f} m/s")
        print(f"   segments: {stats.num_pieces}")
        
        # sample trajectory at specific time
        state = gc.DroneState()
        mid_time = stats.total_duration * 0.5
        api.get_state_at_time(mid_time, state)
        pos = state.position
        vel = state.velocity
        print(f"   position at t={mid_time:.2f}s: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
        print(f"   velocity at t={mid_time:.2f}s: [{vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f}]")
        
    else:
        print("❌ trajectory optimization failed!")

if __name__ == "__main__":
    main()