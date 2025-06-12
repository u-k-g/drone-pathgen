#!/usr/bin/env python3
"""
example demonstrating how to use the GCOPTER Python wrapper for drone trajectory planning.
"""

import sys

import numpy as np

# add the build directory to path to import our module
sys.path.append("build")
import gcopter_cpp


def main():
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

    # setup map with obstacles
    map_size = np.array([20, 20, 10], dtype=np.int32)
    origin = np.array([-5.0, -5.0, 0.0])
    voxel_scale = 0.5

    # add obstacles that block the direct path
    obstacles = [
        np.array([0.0, 0.0, 1.0]),
        np.array([-1.0, -1.0, 1.0]),
        np.array([1.0, 1.0, 1.0]),
    ]

    print("🗺️  configuring map...")
    api.configure_map(map_size, origin, voxel_scale, obstacles, 2)

    # set start and goal positions
    start_pos = np.array([-3.0, -3.0, 1.0])
    goal_pos = np.array([3.0, 3.0, 1.0])

    print("🎯 setting endpoints...")
    api.set_endpoints(start_pos, goal_pos)

    # optimization parameters
    planning_timeout = 20.0
    time_weight = 50.0
    segment_length = 2.0
    smoothing_epsilon = 1e-3
    integral_resolution = 8

    # magnitude bounds: [v_max, omega_max, theta_max, thrust_min, thrust_max]
    magnitude_bounds = np.array([5.0, 10.0, np.pi / 3, 5.0, 15.0])

    # penalty weights: [pos, vel, omega, theta, thrust]
    penalty_weights = np.array([1.0, 1.0, 1.0, 1.0, 1.0])  # back to normal values

    # physical params: [mass, grav, horiz_drag, vert_drag, parasitic_drag, speed_smooth]
    physical_params = np.array([1.0, 9.81, 0.0, 0.0, 0.0, 0.01])

    print("🚁 running trajectory optimization...")
    success = api.run_inference(
        planning_timeout,
        time_weight,
        segment_length,
        smoothing_epsilon,
        integral_resolution,
        magnitude_bounds,
        penalty_weights,
        physical_params,
    )

    if not success:
        print("❌ trajectory optimization failed!")
        return

    print("✅ trajectory optimization successful!")

    # get trajectory statistics
    stats = gcopter_cpp.TrajectoryStatistics()
    if api.get_statistics(stats):
        print(f"📊 trajectory duration: {stats.total_duration:.2f}s")
        print(f"   pieces: {stats.num_pieces}, cost: {stats.optimization_cost:.4f}")
        print(f"   max velocity: {stats.max_velocity:.2f} m/s")
        print(f"   max acceleration: {stats.max_acceleration:.2f} m/s²")

    # get state at midpoint
    state = gcopter_cpp.DroneState()
    test_time = stats.total_duration * 0.5
    if api.get_state_at_time(test_time, state):
        print(f"🎯 state at t={test_time:.2f}s:")
        print(
            f"   position: [{state.position[0]:.2f}, {state.position[1]:.2f}, {state.position[2]:.2f}]"
        )
        print(
            f"   velocity: [{state.velocity[0]:.2f}, {state.velocity[1]:.2f}, {state.velocity[2]:.2f}]"
        )

    # visualize with Open3D
    visualize_trajectory(api)


def visualize_trajectory(api):
    """visualize the trajectory and map using Open3D."""
    try:
        import open3d as o3d
    except ImportError:
        print("⚠️  Open3D not installed; skipping visualization.")
        return

    # get visualization data
    result = api.get_visualization_data(show_initial_route=True)
    (
        success,
        trajectory_points,
        voxel_data,
        voxel_size,
        start_pos,
        goal_pos,
        initial_route,
    ) = result

    if not success:
        print("❌ failed to get visualization data")
        return

    print(f"📈 visualizing {len(trajectory_points)} trajectory points")

    # create voxel meshes
    def build_voxel_mesh(indices, voxel_size, origin, color):
        cube = o3d.geometry.TriangleMesh.create_box(voxel_size, voxel_size, voxel_size)
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
    occupied_indices = []
    dilated_indices = []
    for z in range(len(voxel_data)):
        for y in range(len(voxel_data[z])):
            for x in range(len(voxel_data[z][y])):
                val = voxel_data[z][y][x]
                if val == 1:
                    occupied_indices.append((x, y, z))
                elif val == 2:
                    dilated_indices.append((x, y, z))

    # build geometries
    geometries = []

    # obstacle voxels (red)
    if occupied_indices:
        occupied_mesh = build_voxel_mesh(
            occupied_indices,
            voxel_size,
            np.array([-5.0, -5.0, 0.0]),
            [1, 0, 0],
        )
        geometries.append(occupied_mesh)

    # dilated voxels (yellow)
    if dilated_indices:
        dilated_mesh = build_voxel_mesh(
            dilated_indices,
            voxel_size,
            np.array([-5.0, -5.0, 0.0]),
            [1, 1, 0],
        )
        geometries.append(dilated_mesh)

    # trajectory line (blue)
    if len(trajectory_points) >= 2:
        pts = np.array(trajectory_points)
        lines = [[i, i + 1] for i in range(len(pts) - 1)]
        trajectory_line = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(pts),
            lines=o3d.utility.Vector2iVector(lines),
        )
        trajectory_line.paint_uniform_color([0, 0, 1])
        geometries.append(trajectory_line)

    # initial route line (green)
    if len(initial_route) >= 2:
        pts = np.array(initial_route)
        lines = [[i, i + 1] for i in range(len(pts) - 1)]
        route_line = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(pts),
            lines=o3d.utility.Vector2iVector(lines),
        )
        route_line.paint_uniform_color([0, 1, 0])
        geometries.append(route_line)

    # start/goal spheres
    start_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=voxel_size * 0.5)
    start_sphere.translate(start_pos)
    start_sphere.paint_uniform_color([0, 1, 0])
    start_sphere.compute_vertex_normals()

    goal_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=voxel_size * 0.5)
    goal_sphere.translate(goal_pos)
    goal_sphere.paint_uniform_color([1, 0, 1])
    goal_sphere.compute_vertex_normals()

    geometries.extend([start_sphere, goal_sphere])

    print("👀 launching Open3D visualizer...")
    print("   🟢 green line: initial OMPL path")
    print("   🔵 blue line: optimized trajectory")
    print("   🔴 red cubes: obstacles")
    print("   🟡 yellow cubes: safety dilation")
    print("   🟢 green sphere: start position")
    print("   🟣 magenta sphere: goal position")

    o3d.visualization.draw_geometries(geometries)


if __name__ == "__main__":
    print("🚀 GCOPTER Python wrapper example")
    main()
    print("✅ example complete")
