import numpy as np


def parse_output_file(output_text):
    """Parse the console output into voxels, trajectory, and polyline points"""
    lines = output_text.strip().split("\n")
    
    voxels = []
    polyline_points = []
    trajectory_points = []
    
    parsing_mode = 'VOXELS'
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


def check_voxel_collisions(points, voxel_array, path_name):
    """Check if any points intersect with occupied or dilated voxels"""
    print(f"\n=== Checking {path_name} ===")
    
    collisions = []
    for i, point in enumerate(points):
        # Convert world coordinates to voxel indices
        vx, vy, vz = int(point[0]), int(point[1]), int(point[2])
        
        # Check bounds
        if (0 <= vx < voxel_array.shape[2] and 
            0 <= vy < voxel_array.shape[1] and 
            0 <= vz < voxel_array.shape[0]):
            
            voxel_value = voxel_array[vz, vy, vx]
            
            if voxel_value == 1:
                print(f"  Point {i}: {point} -> COLLISION with OCCUPIED voxel!")
                collisions.append((i, point, "OCCUPIED"))
            elif voxel_value == 2:
                print(f"  Point {i}: {point} -> COLLISION with DILATED voxel!")
                collisions.append((i, point, "DILATED"))
            elif voxel_value == 0:
                # Only print first and last few points to avoid spam
                if i < 3 or i >= len(points) - 3:
                    print(f"  Point {i}: {point} -> OK (free space)")
        else:
            print(f"  Point {i}: {point} -> OUT OF BOUNDS!")
            collisions.append((i, point, "OUT_OF_BOUNDS"))
    
    print(f"Total collisions in {path_name}: {len(collisions)}")
    return collisions


if __name__ == "__main__":
    # Load the data
    try:
        with open("voxel_output.txt", "r") as f:
            output_text = f.read()
    except FileNotFoundError:
        print("Error: voxel_output.txt not found.")
        exit(1)
    
    # Parse the data
    voxels, polyline, trajectory = parse_output_file(output_text)
    print(f"Voxel grid shape: {voxels.shape}")
    print(f"Polyline points: {len(polyline)}")
    print(f"Trajectory points: {len(trajectory)}")
    
    # Check for collisions
    polyline_collisions = check_voxel_collisions(polyline, voxels, "POLYLINE (OMPL path)")
    trajectory_collisions = check_voxel_collisions(trajectory[::10], voxels, "TRAJECTORY (every 10th point)")  # Sample every 10th point to avoid spam
    
    print(f"\n=== SUMMARY ===")
    print(f"OMPL path collisions: {len(polyline_collisions)}")
    print(f"Trajectory collisions: {len(trajectory_collisions)}")
    
    if len(polyline_collisions) > 0:
        print("❌ Issue is in OMPL path planning - initial path goes through obstacles!")
    elif len(trajectory_collisions) > 0:
        print("❌ Issue is in trajectory optimization - GCOPTER is cutting through obstacles!")
    else:
        print("✅ No collisions detected - might be a visualization issue.") 