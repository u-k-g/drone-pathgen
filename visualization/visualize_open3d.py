import numpy as np
import open3d as o3d
import re


def parse_output_file(output_text):
    """Parse the console output into voxels, trajectory, and polyline points"""
    lines = output_text.strip().split("\n")
    
    voxels = []
    polyline_points = []
    trajectory_points = []
    
    parsing_mode = 'VOXELS'  # Can be VOXELS, POLYLINE, or TRAJECTORY
    current_layer = []
    
    for line in lines:
        if line.startswith("POLYLINE"):
            parsing_mode = 'POLYLINE'
            continue
        elif line.startswith("TRAJECTORY"):
            parsing_mode = 'TRAJECTORY'
            continue
            
        if parsing_mode == 'VOXELS':
            if line.startswith("Layer"):
                if current_layer:
                    voxels.append(current_layer)
                current_layer = []
            elif line and line[0].isdigit():
                row = [int(c) for c in line if c.isdigit()]
                current_layer.append(row)
        else:
            parts = line.split()
            if len(parts) == 3:
                try:
                    point = [float(p) for p in parts]
                    if parsing_mode == 'POLYLINE':
                        polyline_points.append(point)
                    elif parsing_mode == 'TRAJECTORY':
                        trajectory_points.append(point)
                except ValueError:
                    pass

    if current_layer:
        voxels.append(current_layer)
    
    return np.array(voxels), np.array(polyline_points), np.array(trajectory_points)


def create_voxel_mesh(voxel_array, voxel_size=1.0):
    """Create Open3D mesh geometries for occupied and dilated voxels"""
    occupied_meshes = []
    dilated_meshes = []
    
    # colors for different voxel types
    occupied_color = [0.8, 0.2, 0.2]  # red for occupied
    dilated_color = [1.0, 0.6, 0.0]   # orange for dilated
    
    for z in range(voxel_array.shape[0]):
        for y in range(voxel_array.shape[1]):
            for x in range(voxel_array.shape[2]):
                voxel_val = voxel_array[z, y, x]
                
                if voxel_val == 1 or voxel_val == 2:  # occupied or dilated
                    # create a cube mesh
                    cube = o3d.geometry.TriangleMesh.create_box(
                        width=voxel_size, 
                        height=voxel_size, 
                        depth=voxel_size
                    )
                    
                    # position the cube at the correct voxel location
                    cube.translate([x, y, z])
                    
                    if voxel_val == 1:  # occupied
                        cube.paint_uniform_color(occupied_color)
                        occupied_meshes.append(cube)
                    else:  # dilated
                        cube.paint_uniform_color(dilated_color)
                        dilated_meshes.append(cube)
    
    return occupied_meshes, dilated_meshes


def create_path_lineset(points, color):
    """Create Open3D LineSet from path points"""
    if len(points) < 2:
        return None
        
    # create line connections between consecutive points
    lines = [[i, i + 1] for i in range(len(points) - 1)]
    
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.paint_uniform_color(color)
    
    return line_set


def create_point_markers(positions, colors, sizes):
    """Create sphere markers for start/goal points"""
    markers = []
    
    for pos, color, size in zip(positions, colors, sizes):
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=size)
        sphere.translate(pos)
        sphere.paint_uniform_color(color)
        markers.append(sphere)
    
    return markers


def visualize_with_open3d(voxel_array, trajectory_points=None, polyline_points=None, 
                         start_pos=None, goal_pos=None):
    """Create interactive 3D visualization using Open3D"""
    
    # create the main visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="GCOPTER Path Planning Visualization", 
                      width=1200, height=800)
    
    # create voxel meshes
    print("Creating voxel meshes...")
    occupied_meshes, dilated_meshes = create_voxel_mesh(voxel_array)
    
    # add voxel meshes to visualizer
    for mesh in occupied_meshes:
        vis.add_geometry(mesh)
    for mesh in dilated_meshes:
        vis.add_geometry(mesh)
    
    # create and add polyline (initial OMPL path)
    if polyline_points is not None and len(polyline_points) > 1:
        print("Adding initial path (polyline)...")
        polyline_set = create_path_lineset(polyline_points, [0.5, 0.5, 0.5])  # gray
        if polyline_set:
            vis.add_geometry(polyline_set)
    
    # create and add trajectory (optimized path)  
    if trajectory_points is not None and len(trajectory_points) > 1:
        print("Adding optimized trajectory...")
        trajectory_set = create_path_lineset(trajectory_points, [0.0, 0.8, 1.0])  # cyan
        if trajectory_set:
            vis.add_geometry(trajectory_set)
    
    # create and add start/goal markers
    marker_positions = []
    marker_colors = []
    marker_sizes = []
    
    if start_pos is not None:
        marker_positions.append(start_pos)
        marker_colors.append([0.0, 0.8, 0.0])  # green
        marker_sizes.append(0.3)
        
    if goal_pos is not None:
        marker_positions.append(goal_pos)
        marker_colors.append([0.0, 0.0, 0.8])  # blue  
        marker_sizes.append(0.3)
    
    if marker_positions:
        print("Adding start/goal markers...")
        markers = create_point_markers(marker_positions, marker_colors, marker_sizes)
        for marker in markers:
            vis.add_geometry(marker)
    
    # set up the view
    ctr = vis.get_view_control()
    
    # set a nice viewing angle
    ctr.set_zoom(0.7)
    ctr.set_front([0.5, 0.5, -0.5])
    ctr.set_up([0, 0, 1])
    
    print("Visualization ready!")
    print("Controls:")
    print("- Mouse: Rotate view")
    print("- Mouse wheel: Zoom")
    print("- Right mouse: Pan")
    print("- Press 'Q' or close window to exit")
    
    # run the visualizer
    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    # check if test output file exists, if not run from console output
    try:
        with open("voxel_output.txt", "r") as f:
            output_text = f.read()
        print("Loaded data from voxel_output.txt")
    except FileNotFoundError:
        print("Error: voxel_output.txt not found.")
        print("Please run the C++ test executable first:")
        print("  cd build && ./test > ../voxel_output.txt")
        exit(1)
    
    # parse the data
    voxels, polyline, trajectory = parse_output_file(output_text)
    print(f"Parsed voxel grid shape: {voxels.shape}")
    if polyline.shape[0] > 0:
        print(f"Parsed polyline with {polyline.shape[0]} points")
    if trajectory.shape[0] > 0:
        print(f"Parsed trajectory with {trajectory.shape[0]} points")
    
    # extract start and goal positions
    start_pos = None
    goal_pos = None
    if trajectory.shape[0] > 0:
        start_pos = trajectory[0]
        goal_pos = trajectory[-1]
    elif polyline.shape[0] > 0:
        start_pos = polyline[0] 
        goal_pos = polyline[-1]
    
    # create the visualization
    visualize_with_open3d(voxels, trajectory, polyline, start_pos, goal_pos) 