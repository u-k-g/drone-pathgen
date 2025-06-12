#!/usr/bin/env python3
"""a complete example demonstrating how to use the gcopier python wrapper."""

import sys
import numpy as np

# add the build directory to the python path to import our custom module
sys.path.append("build")
try:
    import gcopter_cpp
except ImportError:
    print("error: could not import 'gcopter_cpp' module.")
    print("please build the project first by running './scripts/build.sh'")
    sys.exit(1)


def main():
    """
    example demonstrating the full workflow for drone trajectory planning.

    steps:
    1.  create a `GCopterAPI` instance.
    2.  configure a 3d voxel map with obstacles.
    3.  set start and goal endpoints for the trajectory.
    4.  define optimization parameters and physical constraints.
    5.  run the trajectory optimization pipeline.
    6.  query and print statistics and state samples from the result.
    7.  visualize the map and trajectory in 3d using open3d.
    """
    print(" GCOPTER Python Wrapper Example ".center(80, "-"))
    print("🔧 creating GCopterAPI instance...")
    api = gcopter_cpp.GCopterAPI()

    # step 1: configure the 3d map
    # ------------------------------------
    # define the map dimensions: 20x20x10 voxels
    map_size = np.array([20, 20, 10], dtype=np.int32)
    # define the world coordinate of the map's origin
    origin = np.array([-5.0, -5.0, 0.0])
    # define the size of each voxel (0.5 meters)
    voxel_scale = 0.5

    # add some obstacles to block the direct path between start and goal
    obstacles = [
        np.array([0.0, 0.0, 1.0]),  # a pillar in the center
        np.array([-1.0, -1.0, 1.0]),  # a side obstacle
        np.array([1.0, 1.0, 1.0]),  # another side obstacle
    ]

    print("🗺️  configuring map with obstacles...")
    # dilation_radius adds a safety margin around obstacles
    api.configure_map(map_size, origin, voxel_scale, obstacles, 2)

    # step 2: set start and goal positions
    # ------------------------------------
    start_pos = np.array([-3.0, -3.0, 1.0])
    goal_pos = np.array([3.0, 3.0, 1.0])
    # note: you can also provide optional start/goal velocities, e.g.:
    # start_vel = np.array([0.0, 0.0, 0.0])
    # api.set_endpoints(start_pos, goal_pos, start_vel, goal_vel)
    print("🎯 setting endpoints...")
    api.set_endpoints(start_pos, goal_pos)

    # step 3: define optimization parameters
    # ------------------------------------
    planning_timeout = 20.0  # max time for the initial path planner (ompl)
    time_weight = 50.0  # penalty on total flight time (higher = faster trajectory)
    segment_length = 2.0  # desired length of each corridor segment
    smoothing_epsilon = 1e-3  # epsilon for safe corridor smoothing
    integral_resolution = 8  # number of points per segment for penalty checks

    # magnitude bounds: [v_max, omega_max, theta_max, thrust_min, thrust_max]
    magnitude_bounds = np.array([5.0, 10.0, np.pi / 3, 5.0, 15.0])

    # penalty weights for optimizer: [pos, vel, omega, theta, thrust]
    penalty_weights = np.array([1.0, 1.0, 1.0, 1.0, 1.0])

    # physical params: [mass, gravity, horiz_drag, vert_drag, parasitic_drag, speed_smooth]
    physical_params = np.array([1.0, 9.81, 0.0, 0.0, 0.0, 0.01])

    # step 4: run the optimization
    # ------------------------------------
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

    # step 5: get and print trajectory statistics
    # ------------------------------------
    stats = gcopter_cpp.TrajectoryStatistics()
    if api.get_statistics(stats):
        print("\n--- trajectory statistics ---")
        print(f"   duration: {stats.total_duration:.2f}s")
        print(f"   num segments: {stats.num_pieces}")
        print(f"   optimization cost: {stats.optimization_cost:.4f}")
        print(f"   max velocity: {stats.max_velocity:.2f} m/s")
        print(f"   max acceleration: {stats.max_acceleration:.2f} m/s²")
        print("---------------------------\n")

    # get state at the halfway point of the trajectory
    state = gcopter_cpp.DroneState()
    test_time = stats.total_duration * 0.5
    if api.get_state_at_time(test_time, state):
        print(f"querying state at t={test_time:.2f}s:")
        pos = state.position
        vel = state.velocity
        print(f"   position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
        print(f"   velocity: [{vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f}]")

    # step 6: visualize the result
    # ------------------------------------
    visualize_trajectory(api, origin)


def visualize_trajectory(api: gcopter_cpp.GCopterAPI, origin: np.ndarray):
    """
    visualizes the trajectory, obstacles, and flight corridors using open3d.
    
    this function will be skipped if open3d is not installed.
    """
    try:
        import open3d as o3d
    except ImportError:
        print("\n⚠️  open3d not installed. skipping visualization.")
        print("   install with: 'pip install open3d'")
        return

    print("\n🚀 launching open3d visualizer...")

    # retrieve all necessary data from the api
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
        print("❌ failed to get visualization data from api.")
        return

    # --- helper function to build voxel meshes ---
    def build_voxel_mesh(indices, voxel_size_val, origin, color):
        """creates a single trianglemesh from a list of voxel indices."""
        # create a single cube mesh to be copied for each voxel
        cube = o3d.geometry.TriangleMesh.create_box(
            width=voxel_size_val, height=voxel_size_val, depth=voxel_size_val
        )
        cube.compute_vertex_normals()
        cube.paint_uniform_color(color)

        mesh = o3d.geometry.TriangleMesh()
        for x, y, z in indices:
            # create a copy and translate it to the correct voxel position
            cube_copy = cube.translate(
                origin + np.array([x, y, z]) * voxel_size_val, relative=False
            )
            mesh += cube_copy
        return mesh

    # --- collect voxel indices by type ---
    occupied_indices, dilated_indices = [], []
    for z, layer in enumerate(voxel_data):
        for y, row in enumerate(layer):
            for x, val in enumerate(row):
                if val == 1:  # occupied
                    occupied_indices.append((x, y, z))
                elif val == 2:  # dilated
                    dilated_indices.append((x, y, z))

    # --- create geometries for visualization ---
    geometries = []

    # create red mesh for obstacles
    if occupied_indices:
        occupied_mesh = build_voxel_mesh(
            occupied_indices, voxel_size, origin, [0.8, 0.2, 0.2]
        )
        geometries.append(occupied_mesh)

    # create yellow mesh for safety dilation
    if dilated_indices:
        dilated_mesh = build_voxel_mesh(
            dilated_indices, voxel_size, origin, [0.9, 0.9, 0.2]
        )
        geometries.append(dilated_mesh)

    # create blue line for the final optimized trajectory
    if len(trajectory_points) >= 2:
        lineset = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(np.array(trajectory_points)),
            lines=o3d.utility.Vector2iVector(
                [[i, i + 1] for i in range(len(trajectory_points) - 1)]
            ),
        )
        lineset.paint_uniform_color([0.2, 0.2, 0.8])
        geometries.append(lineset)

    # create green line for the initial, un-optimized route
    if len(initial_route) >= 2:
        lineset = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(np.array(initial_route)),
            lines=o3d.utility.Vector2iVector(
                [[i, i + 1] for i in range(len(initial_route) - 1)]
            ),
        )
        lineset.paint_uniform_color([0.2, 0.8, 0.2])
        geometries.append(lineset)

    # add spheres for start and goal points
    start_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=voxel_size)
    start_sphere.translate(start_pos).paint_uniform_color([0.2, 0.8, 0.2])
    goal_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=voxel_size)
    goal_sphere.translate(goal_pos).paint_uniform_color([0.8, 0.2, 0.2])
    geometries.extend([start_sphere, goal_sphere])

    # --- run the visualizer ---
    o3d.visualization.draw_geometries(geometries)


if __name__ == "__main__":
    main()
