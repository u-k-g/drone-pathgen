#!/usr/bin/env python3
"""
Simple test script to verify the GCOPTER Python API works.
"""
import sys
import numpy as np

# add the build directory to path to import our module
sys.path.append('build')
import gcopter_cpp

def example_usage():
    """Example demonstrating how to use the gcopter_cpp Python wrapper.

    Steps shown:
    1. Configure a voxel map with obstacles
    2. Set start and goal endpoints
    3. Run trajectory optimization
    4. Query statistics and state samples
    5. Visualize map + trajectory in Open3D (if installed)
    """
    print("🔧 Creating GCopterAPI instance...")
    api = gcopter_cpp.GCopterAPI()
    
    # create a challenging test with obstacles directly in the path
    map_size = np.array([20, 20, 10], dtype=np.int32)  # 20x20x10 voxel grid
    origin = np.array([-5.0, -5.0, 0.0])  # origin at (-5,-5,0)
    voxel_scale = 0.5  # 0.5m per voxel
    # map goes from (-5,-5,0) to (5,5,5) in world coordinates
    
    # add obstacles directly blocking the straight path from (-3,-3,1) to (3,3,1)
    obstacles = [
        np.array([0.0, 0.0, 1.0]),   # center obstacle - blocks direct path
        np.array([-1.0, -1.0, 1.0]), # diagonal obstacle
        np.array([1.0, 1.0, 1.0]),   # diagonal obstacle
    ]
    
    print("🗺️  Configuring map...")
    api.configure_map(map_size, origin, voxel_scale, obstacles, 2)  # NO dilation
    
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
    penalty_weights = np.array([1.0, 1.0, 1.0, 1.0, 1.0])  # back to normal values
    
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
        result = api.get_visualization_data(show_initial_route=True)
        
        if len(result) == 7:  # with initial route
            success, trajectory_points, voxel_data, voxel_size, start_pos, goal_pos, initial_route = result
            has_initial_route = True
        else:  # without initial route  
            success, trajectory_points, voxel_data, voxel_size, start_pos, goal_pos = result
            has_initial_route = False
            initial_route = []
        
        if success:
            print(f"📈 Visualization data extracted:")
            print(f"   Trajectory points: {len(trajectory_points)}")
            if has_initial_route:
                print(f"   Initial route points: {len(initial_route)}")
            print(f"   Voxel size: {voxel_size}")
            if voxel_data and voxel_data[0]:
                print(f"   Voxel grid shape: {len(voxel_data)}x{len(voxel_data[0])}x{len(voxel_data[0][0])}")
            else:
                print("   Voxel data: empty")
            
            # ---- Open3D visualization ----
            try:
                import open3d as o3d
            except ImportError:
                print("⚠️  Open3D not installed; skipping visualization.")
                return

            # Helper: build a triangle mesh for a set of voxel indices
            def build_voxel_mesh(indices, voxel_size, origin, color):
                """Return a single TriangleMesh that is the union of unit cubes
                translated to each index given.
                """
                cube_template = o3d.geometry.TriangleMesh.create_box(
                    width=voxel_size,
                    height=voxel_size,
                    depth=voxel_size,
                )
                cube_template.compute_vertex_normals()
                cube_template.paint_uniform_color(color)

                out_mesh = o3d.geometry.TriangleMesh()
                for (x, y, z) in indices:
                    cube = cube_template.translate(
                        origin + np.array([x, y, z], dtype=float) * voxel_size,
                        relative=False,
                    )
                    out_mesh += cube
                return out_mesh

            # Collect voxel indices for occupied (1) and dilated (2)
            occ_indices = []
            dil_indices = []
            for z in range(len(voxel_data)):
                for y in range(len(voxel_data[z])):
                    for x in range(len(voxel_data[z][y])):
                        val = voxel_data[z][y][x]
                        if val == 1:
                            occ_indices.append((x, y, z))
                        elif val == 2:
                            dil_indices.append((x, y, z))

            # Build meshes (semi-transparent via material later)
            occ_mesh = build_voxel_mesh(occ_indices, voxel_size, origin, [1, 0, 0])
            dil_mesh = build_voxel_mesh(dil_indices, voxel_size, origin, [1, 1, 0])

            # Create list of geometries to display
            geometries = [occ_mesh, dil_mesh]

            # Optimized trajectory LineSet (blue)
            pts = np.array(trajectory_points)
            if pts.shape[0] >= 2:
                lines = [[i, i + 1] for i in range(pts.shape[0] - 1)]
                line_set = o3d.geometry.LineSet(
                    points=o3d.utility.Vector3dVector(pts),
                    lines=o3d.utility.Vector2iVector(lines),
                )
                line_set.paint_uniform_color([0, 0, 1])  # blue optimized path
                geometries.append(line_set)

            # Initial route LineSet (green) if available
            if has_initial_route and len(initial_route) >= 2:
                initial_pts = np.array(initial_route)
                initial_lines = [[i, i + 1] for i in range(initial_pts.shape[0] - 1)]
                initial_line_set = o3d.geometry.LineSet(
                    points=o3d.utility.Vector3dVector(initial_pts),
                    lines=o3d.utility.Vector2iVector(initial_lines),
                )
                initial_line_set.paint_uniform_color([0, 1, 0])  # green initial path
                geometries.append(initial_line_set)
                print("   🟢 Initial OMPL route: green line")

            # Start/goal spheres for clarity
            def make_sphere(center, radius=voxel_size * 0.5, color=[0, 1, 0]):
                sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
                sphere.translate(center)
                sphere.paint_uniform_color(color)
                sphere.compute_vertex_normals()
                return sphere

            start_sphere = make_sphere(start_pos, color=[0, 1, 0])
            goal_sphere = make_sphere(goal_pos, color=[1, 0, 1])
            geometries.extend([start_sphere, goal_sphere])

            print("👀 Launching Open3D visualizer...")
            if has_initial_route:
                print("   🟢 Green line: Initial OMPL path")
                print("   🔵 Blue line: Optimized trajectory")
            else:
                print("   🔵 Blue line: Optimized trajectory")
            print("   🔴 Red cubes: Obstacles")
            print("   🟡 Yellow cubes: Safety dilation")
            print("   🟢 Green sphere: Start position")
            print("   🟣 Magenta sphere: Goal position")
            print("   Close the window to finish.")
            
            o3d.visualization.draw(geometries)
            # ----------------------------

        return  # end of example
    else:
        print("❌ Trajectory optimization failed!")
        return

if __name__ == "__main__":
    print("🚀 Running GCOPTER Python wrapper example...")
    example_usage()
    print("✅ Example finished. Close the visualization window to exit.") 