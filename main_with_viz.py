#!/usr/bin/env python3
"""
simple test of the drone-pathgen package with visualization
"""

import numpy as np

def main():
    print("🚁 testing drone-pathgen package...")
    
    # test import
    try:
        import gcopter_cpp
        print("✅ successfully imported gcopter_cpp")
    except ImportError as e:
        print(f"❌ failed to import: {e}")
        return
    
    # create api instance
    api = gcopter_cpp.GCopterAPI()
    print("✅ created api instance")
    
    # setup a simple test environment
    print("\n🗺️  setting up test environment...")
    
    # create a 15x15x8 voxel map (bigger for better visualization)
    map_size = np.array([15, 15, 8], dtype=np.int32)
    origin = np.array([-3.75, -3.75, 0.0])  # center the map
    voxel_scale = 0.5  # 0.5m voxels
    
    # add multiple obstacles to make it interesting
    obstacles = [
        # central tower
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, 0.0, 2.0]),
        np.array([0.0, 0.0, 3.0]),
        
        # side obstacles
        np.array([-1.0, 1.0, 1.5]),
        np.array([1.0, -1.0, 1.5]),
        np.array([0.5, 1.5, 2.0]),
        np.array([-1.5, -0.5, 2.0]),
        
        # create a wall to go around
        np.array([0.0, -2.0, 1.0]),
        np.array([0.5, -2.0, 1.0]),
        np.array([-0.5, -2.0, 1.0]),
    ]
    
    api.configure_map(map_size, origin, voxel_scale, obstacles, dilation_radius=1)
    print("✅ configured map with obstacles")
    
    # set start and goal (should navigate around obstacles)
    start_pos = np.array([-2.5, -2.5, 2.0])
    goal_pos = np.array([2.5, 2.5, 2.0])
    
    api.set_endpoints(start_pos, goal_pos)
    print("✅ set start and goal positions")
    
    # run trajectory optimization
    print("\n🔄 running trajectory optimization...")
    
    # optimization parameters
    planning_timeout = 15.0  # more time for complex environment
    time_weight = 20.0
    segment_length = 1.5
    smoothing_epsilon = 1e-3
    integral_resolution = 8
    
    # bounds: [v_max, omega_max, theta_max, thrust_min, thrust_max]
    magnitude_bounds = np.array([4.0, 8.0, np.pi/3, 6.0, 14.0])
    
    # penalty weights: [pos, vel, omega, theta, thrust]
    penalty_weights = np.array([1.0, 1.0, 1.0, 1.0, 1.0])
    
    # physical params: [mass, gravity, horiz_drag, vert_drag, parasitic_drag, speed_smooth]
    physical_params = np.array([1.0, 9.81, 0.0, 0.0, 0.0, 0.01])
    
    success = api.run_inference(
        planning_timeout, time_weight, segment_length, smoothing_epsilon,
        integral_resolution, magnitude_bounds, penalty_weights, physical_params
    )
    
    if not success:
        print("❌ trajectory optimization failed")
        return
    
    print("✅ trajectory optimization successful!")
    
    # get and display results
    print("\n📊 trajectory results:")
    stats = gcopter_cpp.TrajectoryStatistics()
    if api.get_statistics(stats):
        print(f"   duration: {stats.total_duration:.2f}s")
        print(f"   segments: {stats.num_pieces}")
        print(f"   max velocity: {stats.max_velocity:.2f} m/s")
        print(f"   optimization cost: {stats.optimization_cost:.4f}")
        print(f"   start: [{stats.start_pos[0]:.2f}, {stats.start_pos[1]:.2f}, {stats.start_pos[2]:.2f}]")
        print(f"   goal:  [{stats.goal_pos[0]:.2f}, {stats.goal_pos[1]:.2f}, {stats.goal_pos[2]:.2f}]")
    
    # test getting state at different times
    print("\n🎯 sampling trajectory states:")
    state = gcopter_cpp.DroneState()
    
    for t_ratio in [0.0, 0.25, 0.5, 0.75, 1.0]:  # more samples
        time = stats.total_duration * t_ratio
        if api.get_state_at_time(time, state):
            pos = state.position
            vel = state.velocity
            acc = state.acceleration
            print(f"   t={time:.2f}s: pos=[{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}], "
                  f"vel=[{vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f}], "
                  f"acc=[{acc[0]:.2f}, {acc[1]:.2f}, {acc[2]:.2f}]")
    
    # test control inputs
    print("\n🎮 testing control inputs:")
    inputs = gcopter_cpp.ControlInputs()
    mid_time = stats.total_duration * 0.5
    if api.get_control_inputs(mid_time, inputs):
        print(f"   at t={mid_time:.2f}s:")
        print(f"   thrust: {inputs.thrust:.2f} N")
        print(f"   quaternion: [{inputs.quaternion[0]:.3f}, {inputs.quaternion[1]:.3f}, "
              f"{inputs.quaternion[2]:.3f}, {inputs.quaternion[3]:.3f}]")
        print(f"   angular_vel: [{inputs.angular_velocity[0]:.3f}, {inputs.angular_velocity[1]:.3f}, "
              f"{inputs.angular_velocity[2]:.3f}] rad/s")
    
    # test visualization
    print("\n🎨 testing visualization...")
    test_visualization(api)
    
    print("\n🎉 all tests passed! your installation is working correctly.")

def test_visualization(api):
    """test the visualization functionality"""
    
    # test basic visualization data extraction
    try:
        result = api.get_visualization_data(show_initial_route=True)
        success, trajectory_points, voxel_data, voxel_size, start_pos, goal_pos, initial_route = result
        
        if success:
            print(f"✅ extracted visualization data:")
            print(f"   trajectory points: {len(trajectory_points)}")
            print(f"   voxel size: {voxel_size:.2f}m")
            print(f"   voxel grid: {len(voxel_data)}x{len(voxel_data[0])}x{len(voxel_data[0][0])}")
            print(f"   initial route points: {len(initial_route)}")
            
            # count obstacles in voxel data
            occupied_count = 0
            dilated_count = 0
            for layer in voxel_data:
                for row in layer:
                    for val in row:
                        if val == 1:
                            occupied_count += 1
                        elif val == 2:
                            dilated_count += 1
            
            print(f"   occupied voxels: {occupied_count}")
            print(f"   dilated voxels: {dilated_count}")
        else:
            print("❌ failed to extract visualization data")
            return
            
    except Exception as e:
        print(f"❌ visualization data extraction failed: {e}")
        return
    
    # test 3d visualization with open3d
    try:
        import open3d as o3d
        print("✅ open3d available - launching 3d visualization...")
        
        # create a simple 3d visualization
        visualize_3d(trajectory_points, voxel_data, voxel_size, start_pos, goal_pos, 
                    initial_route, api.map_.getOrigin() if hasattr(api, 'map_') else [-3.75, -3.75, 0.0])
        
    except ImportError:
        print("⚠️  open3d not available - skipping 3d visualization")
        print("   install with: uv add \"git+https://github.com/u-k-g/drone-pathgen.git[viz]\"")
    except Exception as e:
        print(f"❌ 3d visualization failed: {e}")

def visualize_3d(trajectory_points, voxel_data, voxel_size, start_pos, goal_pos, initial_route, origin):
    """create 3d visualization using open3d"""
    import open3d as o3d
    
    geometries = []
    origin = np.array(origin)
    
    # helper function to create voxel meshes
    def build_voxel_mesh(indices, color):
        if not indices:
            return None
        
        cube = o3d.geometry.TriangleMesh.create_box(
            width=voxel_size, height=voxel_size, depth=voxel_size
        )
        cube.compute_vertex_normals()
        cube.paint_uniform_color(color)
        
        mesh = o3d.geometry.TriangleMesh()
        for x, y, z in indices:
            cube_copy = cube.translate(
                origin + np.array([x, y, z]) * voxel_size, relative=False
            )
            mesh += cube_copy
        return mesh
    
    # collect voxel indices
    occupied_indices, dilated_indices = [], []
    for z, layer in enumerate(voxel_data):
        for y, row in enumerate(layer):
            for x, val in enumerate(row):
                if val == 1:  # occupied
                    occupied_indices.append((x, y, z))
                elif val == 2:  # dilated (safety margin)
                    dilated_indices.append((x, y, z))
    
    # create obstacle meshes
    if occupied_indices:
        occupied_mesh = build_voxel_mesh(occupied_indices, [0.8, 0.1, 0.1])  # red obstacles
        geometries.append(occupied_mesh)
    
    if dilated_indices:
        dilated_mesh = build_voxel_mesh(dilated_indices, [0.9, 0.7, 0.1])  # yellow safety margin
        geometries.append(dilated_mesh)
    
    # create trajectory line (optimized)
    if len(trajectory_points) >= 2:
        traj_line = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(np.array(trajectory_points)),
            lines=o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(trajectory_points) - 1)])
        )
        traj_line.paint_uniform_color([0.1, 0.1, 0.9])  # blue trajectory
        geometries.append(traj_line)
    
    # create initial route line (unoptimized)
    if len(initial_route) >= 2:
        initial_line = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(np.array(initial_route)),
            lines=o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(initial_route) - 1)])
        )
        initial_line.paint_uniform_color([0.1, 0.7, 0.1])  # green initial route
        geometries.append(initial_line)
    
    # create start/goal spheres
    start_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=voxel_size * 0.8)
    start_sphere.translate(start_pos).paint_uniform_color([0.1, 0.9, 0.1])  # bright green start
    
    goal_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=voxel_size * 0.8)  
    goal_sphere.translate(goal_pos).paint_uniform_color([0.9, 0.1, 0.1])  # bright red goal
    
    geometries.extend([start_sphere, goal_sphere])
    
    # create coordinate frame at origin
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=origin)
    geometries.append(coord_frame)
    
    print("🎨 launching open3d visualizer...")
    print("   legend:")
    print("   🔴 red spheres/voxels: obstacles and goal")
    print("   🟡 yellow voxels: safety margin")
    print("   🔵 blue line: optimized trajectory")
    print("   🟢 green line: initial path, green sphere: start")
    print("   📐 colored axes: coordinate frame")
    
    # launch visualizer
    o3d.visualization.draw_geometries(
        geometries,
        window_name="drone trajectory visualization",
        width=1200,
        height=800
    )

if __name__ == "__main__":
    main() 