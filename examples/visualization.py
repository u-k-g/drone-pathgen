#!/usr/bin/env python3
"""
trajectory visualization example

demonstrates 3d visualization of:
1. obstacle voxels
2. planned trajectory 
3. safe flight corridors
4. start/goal markers

requires: uv add "git+https://github.com/u-k-g/drone-pathgen.git[viz]"
"""

import numpy as np
import gcopter_cpp as gc

def visualize_trajectory():
    """run trajectory planning and visualize results with open3d"""
    
    try:
        import open3d as o3d
    except ImportError:
        print("❌ open3d not installed")
        print("   install with: uv add \"git+https://github.com/u-k-g/drone-pathgen.git[viz]\"")
        return
    
    print("🚁 trajectory visualization example")
    
    # create api instance
    api = gc.GCopterAPI()

    # build voxel map with more obstacles for interesting visualization
    map_size     = np.array([30, 30, 15], dtype=np.int32)
    origin       = np.array([-7.5, -7.5, 0.0])
    voxel_scale  = 0.5
    
    # create a more complex obstacle field
    obstacles = [
        # central pillar
        np.array([0.0, 0.0, 2.0]),
        np.array([0.0, 0.0, 3.0]),
        np.array([0.0, 0.0, 4.0]),
        
        # side barriers
        np.array([-2.0, -2.0, 1.5]),
        np.array([2.0, 2.0, 1.5]),
        np.array([2.0, -2.0, 1.5]),
        np.array([-2.0, 2.0, 1.5]),
        
        # scattered obstacles
        np.array([1.0, -3.0, 2.0]),
        np.array([-1.0, 3.0, 2.0]),
        np.array([3.0, 1.0, 2.5]),
        np.array([-3.0, -1.0, 2.5]),
    ]
    
    print("📍 building complex 3d environment...")
    api.configure_map(map_size, origin, voxel_scale, obstacles, dilation_radius=3)

    # set challenging start/goal
    start_pos = np.array([-5.0, -5.0, 1.5])
    goal_pos  = np.array([5.0, 5.0, 3.0])
    api.set_endpoints(start=start_pos, goal=goal_pos)
    print(f"🎯 planning from {start_pos} to {goal_pos}")

    # run optimization
    print("⚡ optimizing trajectory...")
    success = api.run_inference(
        planning_timeout   = 10.0,
        time_weight        = 30.0,
        segment_length     = 1.5,
        smoothing_epsilon  = 1e-3,
        integral_resolution= 10,
        magnitude_bounds   = np.array([4., 8., np.pi/4, 4., 12.]),
        penalty_weights    = np.array([1, 1, 1, 1, 1]),
        physical_params    = np.array([1.2, 9.81, 0., 0., 0., 0.01])
    )

    if not success:
        print("❌ trajectory optimization failed!")
        return

    print("✅ trajectory generated successfully!")
    
    # get visualization data
    result = api.get_visualization_data(show_initial_route=True)
    success, trajectory_points, voxel_data, voxel_size, start, goal, initial_route = result
    
    if not success:
        print("❌ failed to get visualization data")
        return

    print("🎨 building 3d visualization...")
    
    # --- build visualization geometries ---
    geometries = []

    # helper function for voxel meshes
    def build_voxel_mesh(indices, voxel_size_val, origin, color):
        cube = o3d.geometry.TriangleMesh.create_box(
            width=voxel_size_val, height=voxel_size_val, depth=voxel_size_val
        )
        cube.compute_vertex_normals()
        cube.paint_uniform_color(color)

        mesh = o3d.geometry.TriangleMesh()
        for x, y, z in indices:
            cube_copy = cube.translate(
                origin + np.array([x, y, z]) * voxel_size_val, relative=False
            )
            mesh += cube_copy
        return mesh

    # extract voxel indices by type
    occupied_indices, dilated_indices = [], []
    for z, layer in enumerate(voxel_data):
        for y, row in enumerate(layer):
            for x, val in enumerate(row):
                if val == 1:  # occupied
                    occupied_indices.append((x, y, z))
                elif val == 2:  # dilated (safety margin)
                    dilated_indices.append((x, y, z))

    # create obstacle mesh (red)
    if occupied_indices:
        occupied_mesh = build_voxel_mesh(
            occupied_indices, voxel_size, origin, [0.8, 0.2, 0.2]
        )
        geometries.append(occupied_mesh)

    # create safety margin mesh (yellow/transparent)
    if dilated_indices:
        dilated_mesh = build_voxel_mesh(
            dilated_indices, voxel_size, origin, [0.9, 0.7, 0.2]
        )
        geometries.append(dilated_mesh)

    # trajectory line (blue)
    if len(trajectory_points) > 1:
        trajectory_line = o3d.geometry.LineSet()
        trajectory_line.points = o3d.utility.Vector3dVector(trajectory_points)
        lines = [[i, i + 1] for i in range(len(trajectory_points) - 1)]
        trajectory_line.lines = o3d.utility.Vector2iVector(lines)
        trajectory_line.paint_uniform_color([0.2, 0.5, 0.9])
        geometries.append(trajectory_line)

    # initial route (gray, dashed-like)
    if initial_route and len(initial_route) > 1:
        initial_line = o3d.geometry.LineSet()
        initial_line.points = o3d.utility.Vector3dVector(initial_route)
        lines = [[i, i + 1] for i in range(len(initial_route) - 1)]
        initial_line.lines = o3d.utility.Vector2iVector(lines)
        initial_line.paint_uniform_color([0.5, 0.5, 0.5])
        geometries.append(initial_line)

    # start marker (green sphere)
    start_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.3)
    start_sphere.translate(start)
    start_sphere.paint_uniform_color([0.2, 0.8, 0.2])
    geometries.append(start_sphere)

    # goal marker (purple sphere)
    goal_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.3)
    goal_sphere.translate(goal)
    goal_sphere.paint_uniform_color([0.8, 0.2, 0.8])
    geometries.append(goal_sphere)

    # coordinate frame
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    coord_frame.translate(origin)
    geometries.append(coord_frame)

    # launch visualization
    print("🚀 launching open3d viewer...")
    print("   🟢 green sphere = start")
    print("   🟣 purple sphere = goal") 
    print("   🔴 red cubes = obstacles")
    print("   🟡 yellow cubes = safety margins")
    print("   🔵 blue line = optimized trajectory")
    print("   ⚪ gray line = initial route")
    
    o3d.visualization.draw_geometries(
        geometries,
        window_name="gcopter trajectory visualization",
        width=1200,
        height=800
    )

if __name__ == "__main__":
    visualize_trajectory()