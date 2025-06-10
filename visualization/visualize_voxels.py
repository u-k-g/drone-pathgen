import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as mpatches


def parse_output_file(output_text):
    """Parse the console output into voxels, trajectory, and polyline points"""
    lines = output_text.strip().split("\n")
    
    voxels = []
    polyline_points = []
    trajectory_points = []
    
    parsing_mode = 'VOXELS' # Can be VOXELS, POLYLINE, or TRAJECTORY
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


def visualize_voxels(
    voxel_array, trajectory_points=None, polyline_points=None, start_pos=None, goal_pos=None
):
    """Create 3D visualization of voxel grid and trajectory"""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Define colors for the voxel types
    colors = np.array(
        [
            [0, 0, 0, 0],  # 0: Unoccupied (transparent)
            [1, 0, 0, 0.8],  # 1: Occupied (red)
            [1, 0.65, 0, 0.6],  # 2: Dilated (orange)
        ]
    )

    # Create a color map for the voxels
    voxel_colors = colors[voxel_array]

    # Plot the voxels as cubes
    ax.voxels(voxel_array, facecolors=voxel_colors, edgecolor="k", linewidth=0.25)

    # Plot polyline if provided
    if polyline_points is not None and polyline_points.shape[0] > 1:
        ax.plot(
            polyline_points[:, 0],
            polyline_points[:, 1],
            polyline_points[:, 2],
            c="gray",
            ls="--",
            lw=2,
            label="Initial Path",
        )

    # Plot trajectory if provided
    if trajectory_points is not None and trajectory_points.shape[0] > 1:
        ax.plot(
            trajectory_points[:, 0],
            trajectory_points[:, 1],
            trajectory_points[:, 2],
            c="cyan",
            lw=3,
            label="Trajectory",
        )

    # Add start and goal points if provided
    if start_pos is not None:
        ax.scatter(
            *start_pos,
            c="green",
            s=200,
            marker="^",
            label="Start",
            edgecolors="black",
            linewidth=2,
        )

    if goal_pos is not None:
        ax.scatter(
            *goal_pos,
            c="blue",
            s=200,
            marker="*",
            label="Goal",
            edgecolors="black",
            linewidth=2,
        )

    # Set labels and title
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("GCOPTER Voxel Map Visualization")

    # Custom legend for voxel cubes
    occupied_patch = mpatches.Patch(color=colors[1], label="Occupied")
    dilated_patch = mpatches.Patch(color=colors[2], label="Dilated")
    # Re-create legend with handles from trajectory plot and custom patches
    handles, labels = ax.get_legend_handles_labels()
    # Check if trajectory was plotted and add its handle
    if any("Trajectory" in s for s in labels):
        traj_handle = handles[labels.index("Trajectory")]
        ax.legend(handles=[traj_handle, occupied_patch, dilated_patch])
    else:
        ax.legend(handles=[occupied_patch, dilated_patch])

    # Set equal aspect ratio to make cubes look like cubes
    x_size, y_size, z_size = (
        voxel_array.shape[2],
        voxel_array.shape[1],
        voxel_array.shape[0],
    )
    ax.set_xlim(0, x_size)
    ax.set_ylim(0, y_size)
    ax.set_zlim(0, z_size)
    # Create a new bounding box with equal sides for a 1:1:1 aspect ratio
    max_range = np.ptp(
        np.array([ax.get_xlim(), ax.get_ylim(), ax.get_zlim()]), axis=1
    ).max()
    mid_x = np.mean(ax.get_xlim())
    mid_y = np.mean(ax.get_ylim())
    mid_z = np.mean(ax.get_zlim())
    ax.set_xlim(mid_x - max_range / 2, mid_x + max_range / 2)
    ax.set_ylim(mid_y - max_range / 2, mid_y + max_range / 2)
    ax.set_zlim(mid_z - max_range / 2, mid_z + max_range / 2)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # Load the voxel data from the C++ output file
    try:
        with open("voxel_output.txt", "r") as f:
            output_text = f.read()
    except FileNotFoundError:
        print("Error: voxel_output.txt not found.")
        print("Please run the C++ test executable first to generate it.")
        exit()

    # Parse the voxel and trajectory data
    voxels, polyline, trajectory = parse_output_file(output_text)
    print(f"Parsed voxel grid shape: {voxels.shape}")
    if polyline.shape[0] > 0:
        print(f"Parsed polyline with {polyline.shape[0]} points.")
    if trajectory.shape[0] > 0:
        print(f"Parsed trajectory with {trajectory.shape[0]} points.")
    
    # set start and goal points by taking the first and last points of the trajectory or polyline
    if trajectory.shape[0] > 0:
        start_pos = trajectory[0]
        goal_pos = trajectory[-1]
    elif polyline.shape[0] > 0:
        start_pos = polyline[0]
        goal_pos = polyline[-1]
    else:
        start_pos = None
        goal_pos = None
    
    # Create visualizations
    print("Showing 3D visualization...")
    visualize_voxels(voxels, trajectory, polyline, start_pos, goal_pos)
