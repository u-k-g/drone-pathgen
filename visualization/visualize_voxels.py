import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as mpatches

def parse_output_file(output_text):
    """Parse the console output into a 3D numpy array for voxels and a list of trajectory points"""
    lines = output_text.strip().split('\n')
    
    voxels = []
    trajectory_points = []
    
    parsing_voxels = True
    current_layer = []
    
    for line in lines:
        if line.startswith('TRAJECTORY'):
            parsing_voxels = False
            continue
            
        if parsing_voxels:
            if line.startswith('Layer'):
                if current_layer:
                    voxels.append(current_layer)
                current_layer = []
            elif line and line[0].isdigit():
                row = [int(c) for c in line if c.isdigit()]
                current_layer.append(row)
        else:
            # parsing trajectory
            parts = line.split()
            if len(parts) == 3:
                try:
                    point = [float(p) for p in parts]
                    trajectory_points.append(point)
                except ValueError:
                    # ignore non-numeric lines
                    pass

    if current_layer:
        voxels.append(current_layer)
    
    return np.array(voxels), np.array(trajectory_points)

def visualize_voxels(voxel_array, trajectory_points=None, start_pos=None, goal_pos=None):
    """Create 3D visualization of voxel grid and trajectory"""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Define colors for the voxel types
    colors = np.array([
        [0, 0, 0, 0],       # 0: Unoccupied (transparent)
        [1, 0, 0, 0.8],     # 1: Occupied (red)
        [1, 0.65, 0, 0.6]   # 2: Dilated (orange)
    ])
    
    # Create a color map for the voxels
    voxel_colors = colors[voxel_array]

    # Plot the voxels as cubes
    ax.voxels(voxel_array, facecolors=voxel_colors, edgecolor='k', linewidth=0.25)
    
    # Plot trajectory if provided
    if trajectory_points is not None and trajectory_points.shape[0] > 1:
        ax.plot(trajectory_points[:, 0], 
                trajectory_points[:, 1], 
                trajectory_points[:, 2], 
                c='cyan', lw=3, label='Trajectory')
    
    # Add start and goal points if provided
    if start_pos is not None:
        ax.scatter(*start_pos, c='green', s=200, marker='^', 
                  label='Start', edgecolors='black', linewidth=2)
    
    if goal_pos is not None:
        ax.scatter(*goal_pos, c='blue', s=200, marker='*', 
                  label='Goal', edgecolors='black', linewidth=2)
    
    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y') 
    ax.set_zlabel('Z')
    ax.set_title('GCOPTER Voxel Map Visualization')

    # Custom legend for voxel cubes
    occupied_patch = mpatches.Patch(color=colors[1], label='Occupied')
    dilated_patch = mpatches.Patch(color=colors[2], label='Dilated')
    # Re-create legend with handles from trajectory plot and custom patches
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles=[handles[0], occupied_patch, dilated_patch] + handles[1:])
    
    # Set equal aspect ratio
    x_size, y_size, z_size = voxel_array.shape
    ax.set_box_aspect([x_size, y_size, z_size]) # aspect ratio is 1:1:1 in data space

    plt.tight_layout()
    plt.show()

def visualize_2d_layers(voxel_array):
    """Create 2D layer-by-layer visualization"""
    z_size = voxel_array.shape[0]
    cols = min(3, z_size)
    rows = (z_size + cols - 1) // cols
    
    fig, axes = plt.subplots(rows, cols, figsize=(4*cols, 4*rows))
    if z_size == 1:
        axes = [axes]
    elif rows == 1:
        axes = [axes]
    else:
        axes = axes.flatten()
    
    # Color map: 0=white, 1=red, 2=orange
    colors = ['white', 'red', 'orange']
    cmap = plt.matplotlib.colors.ListedColormap(colors)
    
    for i in range(z_size):
        ax = axes[i] if z_size > 1 else axes[0]
        im = ax.imshow(voxel_array[i], cmap=cmap, vmin=0, vmax=2)
        ax.set_title(f'Layer {i}')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        
        # Add grid
        ax.set_xticks(np.arange(-0.5, voxel_array.shape[2], 1), minor=True)
        ax.set_yticks(np.arange(-0.5, voxel_array.shape[1], 1), minor=True)
        ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5)
    
    # Hide unused subplots
    for i in range(z_size, len(axes)):
        axes[i].set_visible(False)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Load the voxel data from the C++ output file
    try:
        with open('voxel_output.txt', 'r') as f:
            output_text = f.read()
    except FileNotFoundError:
        print("Error: voxel_output.txt not found.")
        print("Please run the C++ test executable first to generate it.")
        exit()

    # Parse the voxel and trajectory data
    voxels, trajectory = parse_output_file(output_text)
    print(f"Parsed voxel grid shape: {voxels.shape}")
    if trajectory.shape[0] > 0:
        print(f"Parsed trajectory with {trajectory.shape[0]} points.")
    
    # Define start/goal positions (must match test.cpp)
    start_pos = (0.5, 0.5, 0.5)
    goal_pos = (8.0, 8.0, 2.0)
    
    # Create visualizations
    print("Showing 3D visualization...")
    visualize_voxels(voxels, trajectory, start_pos, goal_pos)
    
    print("Showing 2D layer visualization...")
    visualize_2d_layers(voxels)