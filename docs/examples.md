# Examples

This document provides practical examples for common use cases of the `drone-pathgen` library.

## Basic Examples

### 1. Simple Point-to-Point Navigation

```python
import numpy as np
import gcopter_cpp as gc

def simple_navigation():
    """Navigate from point A to point B with no obstacles"""
    
    api = gc.GCopterAPI()
    
    # Create empty environment
    map_size = np.array([20, 20, 10], dtype=np.int32)
    origin = np.array([-5.0, -5.0, 0.0])
    api.configure_map(map_size, origin, 0.5, [], dilation_radius=0)
    
    # Set endpoints
    start = np.array([[-4.0], [-4.0], [2.0]])
    goal = np.array([[4.0], [4.0], [2.0]])
    api.set_endpoints(start, goal)
    
    # Plan trajectory
    success = api.run_inference(
        planning_timeout=5.0,
        time_weight=50.0,
        segment_length=2.0,
        smoothing_epsilon=1e-3,
        integral_resolution=8,
        magnitude_bounds=np.array([3., 5., np.pi/4, 5., 10.]),
        penalty_weights=np.array([1, 1, 1, 1, 1]),
        physical_params=np.array([0.5, 9.81, 0., 0., 0., 0.01])
    )
    
    return api, success

if __name__ == "__main__":
    api, success = simple_navigation()
    if success:
        stats = gc.TrajectoryStatistics()
        api.get_statistics(stats)
        print(f"✅ Planned {stats.total_duration:.2f}s trajectory")
    else:
        print("❌ Planning failed")
```

### 2. Obstacle Avoidance

```python
def obstacle_avoidance():
    """Navigate around obstacles"""
    
    api = gc.GCopterAPI()
    
    # Create environment with obstacles
    map_size = np.array([30, 20, 10], dtype=np.int32)
    origin = np.array([-7.5, -5.0, 0.0])
    
    # Create wall obstacle
    obstacles = []
    for z in range(5):  # Wall height
        for y in range(-1, 2):  # Wall width
            obstacles.append(np.array([0.0, y * 0.5, z * 0.5 + 1.0]))
    
    api.configure_map(map_size, origin, 0.5, obstacles, dilation_radius=2)
    
    # Plan around the wall
    start = np.array([[-6.0], [0.0], [2.0]])
    goal = np.array([[6.0], [0.0], [2.0]])
    api.set_endpoints(start, goal)
    
    success = api.run_inference(
        planning_timeout=10.0,
        time_weight=30.0,
        segment_length=1.5,
        smoothing_epsilon=1e-4,
        integral_resolution=10,
        magnitude_bounds=np.array([4., 8., np.pi/3, 5., 12.]),
        penalty_weights=np.array([1, 1, 1, 1, 1]),
        physical_params=np.array([1.0, 9.81, 0., 0., 0., 0.01])
    )
    
    return api, success
```

### 3. Multi-Level Navigation

```python
def multi_level_navigation():
    """Navigate between different altitude levels"""
    
    api = gc.GCopterAPI()
    
    # Create multi-level environment
    map_size = np.array([25, 25, 15], dtype=np.int32)
    origin = np.array([-6.25, -6.25, 0.0])
    
    # Create platforms at different levels
    obstacles = []
    
    # Ground level obstacles
    for x in range(-2, 3):
        for y in range(-2, 3):
            obstacles.append(np.array([x * 1.0, y * 1.0, 0.5]))
    
    # Mid-level platform
    for x in range(-1, 2):
        for y in range(-1, 2):
            obstacles.append(np.array([x * 1.0, y * 1.0, 4.0]))
    
    # High-level obstacles
    for x in range(-3, 4):
        for y in range(-1, 2):
            obstacles.append(np.array([x * 1.0, y * 1.0, 7.0]))
    
    api.configure_map(map_size, origin, 0.5, obstacles, dilation_radius=1)
    
    # Navigate from low to high altitude
    start = np.array([[-5.0], [-5.0], [1.0]])   # Low altitude
    goal = np.array([[5.0], [5.0], [6.0]])      # High altitude
    api.set_endpoints(start, goal)
    
    success = api.run_inference(
        planning_timeout=15.0,
        time_weight=40.0,
        segment_length=2.0,
        smoothing_epsilon=1e-3,
        integral_resolution=8,
        magnitude_bounds=np.array([5., 10., np.pi/3, 5., 15.]),
        penalty_weights=np.array([1, 1, 1, 1, 1]),
        physical_params=np.array([1.2, 9.81, 0., 0., 0., 0.01])
    )
    
    return api, success
```

## Advanced Examples

### 4. Dynamic Waypoint Navigation

```python
def waypoint_navigation(waypoints):
    """Navigate through multiple waypoints"""
    
    trajectories = []
    
    for i in range(len(waypoints) - 1):
        api = gc.GCopterAPI()
        
        # Setup environment (reuse for all segments)
        map_size = np.array([40, 40, 20], dtype=np.int32)
        origin = np.array([-10.0, -10.0, 0.0])
        
        # Add some random obstacles
        np.random.seed(42)  # Reproducible
        obstacles = []
        for _ in range(15):
            x = np.random.uniform(-8, 8)
            y = np.random.uniform(-8, 8) 
            z = np.random.uniform(1, 8)
            obstacles.append(np.array([x, y, z]))
        
        api.configure_map(map_size, origin, 0.5, obstacles, dilation_radius=2)
        
        # Plan segment
        start = waypoints[i].reshape(3, 1)
        goal = waypoints[i + 1].reshape(3, 1)
        api.set_endpoints(start, goal)
        
        success = api.run_inference(
            planning_timeout=8.0,
            time_weight=50.0,
            segment_length=2.0,
            smoothing_epsilon=1e-3,
            integral_resolution=8,
            magnitude_bounds=np.array([4., 8., np.pi/3, 5., 12.]),
            penalty_weights=np.array([1, 1, 1, 1, 1]),
            physical_params=np.array([1.0, 9.81, 0., 0., 0., 0.01])
        )
        
        if success:
            trajectories.append(api)
            print(f"✅ Segment {i+1}/{len(waypoints)-1} planned")
        else:
            print(f"❌ Segment {i+1} failed")
            break
    
    return trajectories

# Usage
waypoints = np.array([
    [-8, -8, 2],   # Start
    [-4, 0, 4],    # Waypoint 1
    [0, 4, 3],     # Waypoint 2  
    [4, 0, 5],     # Waypoint 3
    [8, 8, 2]      # Goal
])

trajectories = waypoint_navigation(waypoints)
```

### 5. Velocity-Constrained Planning

```python
def velocity_constrained_planning():
    """Plan trajectory with specific velocity constraints"""
    
    api = gc.GCopterAPI()
    
    # Setup environment
    map_size = np.array([30, 30, 15], dtype=np.int32)
    origin = np.array([-7.5, -7.5, 0.0])
    
    # Create speed bumps (areas requiring slower flight)
    obstacles = []
    for x in range(-2, 3):
        for y in range(-2, 3):
            obstacles.append(np.array([x * 1.0, y * 1.0, 2.0]))
    
    api.configure_map(map_size, origin, 0.5, obstacles, dilation_radius=3)
    
    # Set endpoints with initial and final velocities
    start_pos = np.array([[-6.0], [0.0], [2.0]])
    goal_pos = np.array([[6.0], [0.0], [2.0]])
    
    # Start with some initial velocity
    start_vel = np.array([[1.0], [0.0], [0.0]])  # 1 m/s forward
    goal_vel = np.array([[0.0], [0.0], [0.0]])   # Stop at goal
    
    api.set_endpoints(start_pos, goal_pos, start_vel, goal_vel)
    
    # Use conservative velocity limits
    success = api.run_inference(
        planning_timeout=12.0,
        time_weight=20.0,  # Less emphasis on speed
        segment_length=1.0,
        smoothing_epsilon=1e-4,
        integral_resolution=12,
        magnitude_bounds=np.array([2., 4., np.pi/6, 3., 8.]),  # Conservative
        penalty_weights=np.array([1, 1, 1, 1, 1]),
        physical_params=np.array([0.8, 9.81, 0., 0., 0., 0.01])
    )
    
    return api, success
```

### 6. Precision Landing Approach

```python
def precision_landing():
    """Plan precision approach to landing pad"""
    
    api = gc.GCopterAPI()
    
    # Create landing environment
    map_size = np.array([25, 25, 12], dtype=np.int32)
    origin = np.array([-6.25, -6.25, 0.0])
    
    # Create obstacles around landing pad
    obstacles = []
    
    # Perimeter obstacles (trees, buildings)
    for angle in np.linspace(0, 2*np.pi, 12):
        x = 4.0 * np.cos(angle)
        y = 4.0 * np.sin(angle)
        for z in range(1, 6):
            obstacles.append(np.array([x, y, z * 0.5]))
    
    # Some approach hazards
    obstacles.extend([
        np.array([2.0, 0.0, 1.5]),
        np.array([-2.0, 0.0, 1.5]),
        np.array([0.0, 2.0, 1.5]),
        np.array([0.0, -2.0, 1.5]),
    ])
    
    api.configure_map(map_size, origin, 0.5, obstacles, dilation_radius=2)
    
    # Approach from altitude to landing pad
    start_pos = np.array([[-5.0], [-5.0], [8.0]])  # High approach
    goal_pos = np.array([[0.0], [0.0], [0.2]])     # Landing pad
    
    # Final approach should be slow and controlled
    goal_vel = np.array([[0.0], [0.0], [-0.5]])  # Slow descent
    
    api.set_endpoints(start_pos, goal_pos, goal_vel=goal_vel)
    
    success = api.run_inference(
        planning_timeout=15.0,
        time_weight=10.0,  # Prioritize smoothness for precision
        segment_length=1.0,
        smoothing_epsilon=1e-5,  # Very smooth
        integral_resolution=12,
        magnitude_bounds=np.array([2., 3., np.pi/8, 2., 6.]),  # Gentle limits
        penalty_weights=np.array([1, 1, 1, 1, 1]),
        physical_params=np.array([1.0, 9.81, 0., 0., 0., 0.01])
    )
    
    return api, success
```

## Simulation Examples

### 7. Real-Time Trajectory Following

```python
import time
import matplotlib.pyplot as plt

def real_time_simulation(api, stats):
    """Simulate real-time trajectory following"""
    
    dt = 0.1  # 10 Hz control loop
    current_time = 0.0
    
    positions = []
    velocities = []
    control_inputs = []
    
    state = gc.DroneState()
    controls = gc.ControlInputs()
    
    print("🚀 Starting real-time simulation...")
    
    while current_time <= stats.total_duration:
        # Get current state
        api.get_state_at_time(current_time, state)
        api.get_control_inputs(current_time, controls)
        
        # Log data
        positions.append(state.position.copy())
        velocities.append(state.velocity.copy())
        control_inputs.append({
            'thrust': controls.thrust,
            'quaternion': controls.quaternion.copy(),
            'angular_velocity': controls.angular_velocity.copy()
        })
        
        # Simulate control delay
        time.sleep(dt)
        current_time += dt
        
        # Progress feedback
        if int(current_time * 10) % 10 == 0:
            print(f"  Time: {current_time:.1f}s / {stats.total_duration:.1f}s")
    
    print("✅ Simulation complete!")
    
    # Convert to numpy arrays
    positions = np.array(positions)
    velocities = np.array(velocities)
    
    return positions, velocities, control_inputs

# Usage after planning
if success:
    stats = gc.TrajectoryStatistics()
    api.get_statistics(stats)
    positions, velocities, controls = real_time_simulation(api, stats)
```

### 8. Trajectory Analysis and Validation

```python
def analyze_trajectory(api, stats):
    """Comprehensive trajectory analysis"""
    
    # Sample trajectory densely
    n_samples = int(stats.total_duration * 50)  # 50 Hz sampling
    times = np.linspace(0, stats.total_duration, n_samples)
    
    states = []
    controls = []
    
    state = gc.DroneState()
    control = gc.ControlInputs()
    
    for t in times:
        api.get_state_at_time(t, state)
        api.get_control_inputs(t, control)
        
        states.append({
            'time': t,
            'position': state.position.copy(),
            'velocity': state.velocity.copy(),
            'acceleration': state.acceleration.copy(),
            'jerk': state.jerk.copy()
        })
        
        controls.append({
            'thrust': control.thrust,
            'tilt': np.arccos(control.quaternion[0]) * 2,  # Approximate tilt
            'angular_velocity': np.linalg.norm(control.angular_velocity)
        })
    
    # Analysis
    positions = np.array([s['position'] for s in states])
    velocities = np.array([s['velocity'] for s in states])
    accelerations = np.array([s['acceleration'] for s in states])
    jerks = np.array([s['jerk'] for s in states])
    
    # Compute metrics
    vel_magnitudes = np.linalg.norm(velocities, axis=1)
    acc_magnitudes = np.linalg.norm(accelerations, axis=1)
    jerk_magnitudes = np.linalg.norm(jerks, axis=1)
    
    thrusts = np.array([c['thrust'] for c in controls])
    tilts = np.array([c['tilt'] for c in controls])
    
    # Safety validation
    max_vel = np.max(vel_magnitudes)
    max_acc = np.max(acc_magnitudes)
    max_jerk = np.max(jerk_magnitudes)
    max_thrust = np.max(thrusts)
    max_tilt = np.max(tilts)
    
    print(f"\n📊 Trajectory Analysis:")
    print(f"  Duration: {stats.total_duration:.2f} s")
    print(f"  Path length: {np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1)):.2f} m")
    print(f"  Max velocity: {max_vel:.2f} m/s")
    print(f"  Max acceleration: {max_acc:.2f} m/s²")
    print(f"  Max jerk: {max_jerk:.2f} m/s³")
    print(f"  Max thrust: {max_thrust:.2f} N")
    print(f"  Max tilt: {np.degrees(max_tilt):.1f}°")
    
    # Smoothness metrics
    vel_smoothness = np.std(np.diff(vel_magnitudes))
    acc_smoothness = np.std(np.diff(acc_magnitudes))
    
    print(f"  Velocity smoothness: {vel_smoothness:.4f}")
    print(f"  Acceleration smoothness: {acc_smoothness:.4f}")
    
    return {
        'times': times,
        'positions': positions,
        'velocities': velocities,
        'accelerations': accelerations,
        'jerks': jerks,
        'controls': controls,
        'metrics': {
            'max_velocity': max_vel,
            'max_acceleration': max_acc,
            'max_jerk': max_jerk,
            'max_thrust': max_thrust,
            'max_tilt': max_tilt,
            'velocity_smoothness': vel_smoothness,
            'acceleration_smoothness': acc_smoothness
        }
    }
```

### 9. Batch Processing Multiple Scenarios

```python
def batch_planning_scenarios():
    """Process multiple planning scenarios"""
    
    scenarios = [
        {
            'name': 'Fast Transit',
            'start': np.array([[-8, 0, 2]]).T,
            'goal': np.array([[8, 0, 2]]).T,
            'params': {
                'time_weight': 100.0,
                'magnitude_bounds': np.array([6., 12., np.pi/2, 6., 20.])
            }
        },
        {
            'name': 'Precision Flight',
            'start': np.array([[-8, 0, 2]]).T,
            'goal': np.array([[8, 0, 2]]).T,
            'params': {
                'time_weight': 5.0,
                'magnitude_bounds': np.array([2., 4., np.pi/6, 3., 8.]),
                'smoothing_epsilon': 1e-5
            }
        },
        {
            'name': 'Vertical Ascent',
            'start': np.array([[0, 0, 1]]).T, 
            'goal': np.array([[0, 0, 8]]).T,
            'params': {
                'time_weight': 30.0,
                'magnitude_bounds': np.array([3., 6., np.pi/4, 4., 12.])
            }
        }
    ]
    
    results = []
    
    for scenario in scenarios:
        print(f"\n🎯 Testing scenario: {scenario['name']}")
        
        api = gc.GCopterAPI()
        
        # Standard environment
        map_size = np.array([20, 20, 12], dtype=np.int32)
        origin = np.array([-5.0, -5.0, 0.0])
        obstacles = [np.array([0.0, 0.0, 4.0])]  # Single central obstacle
        
        api.configure_map(map_size, origin, 0.5, obstacles, dilation_radius=2)
        api.set_endpoints(scenario['start'], scenario['goal'])
        
        # Default parameters
        params = {
            'planning_timeout': 10.0,
            'time_weight': 50.0,
            'segment_length': 2.0,
            'smoothing_epsilon': 1e-3,
            'integral_resolution': 8,
            'magnitude_bounds': np.array([4., 8., np.pi/3, 5., 12.]),
            'penalty_weights': np.array([1, 1, 1, 1, 1]),
            'physical_params': np.array([1.0, 9.81, 0., 0., 0., 0.01])
        }
        
        # Override with scenario-specific parameters
        params.update(scenario['params'])
        
        # Plan trajectory
        start_time = time.time()
        success = api.run_inference(**params)
        planning_time = time.time() - start_time
        
        if success:
            stats = gc.TrajectoryStatistics()
            api.get_statistics(stats)
            
            result = {
                'scenario': scenario['name'],
                'success': True,
                'planning_time': planning_time,
                'duration': stats.total_duration,
                'max_velocity': stats.max_velocity,
                'max_acceleration': stats.max_acceleration,
                'pieces': stats.num_pieces,
                'cost': stats.optimization_cost
            }
            
            print(f"  ✅ Success in {planning_time:.2f}s")
            print(f"     Duration: {stats.total_duration:.2f}s")
            print(f"     Max vel: {stats.max_velocity:.2f} m/s")
            
        else:
            result = {
                'scenario': scenario['name'],
                'success': False,
                'planning_time': planning_time
            }
            print(f"  ❌ Failed after {planning_time:.2f}s")
        
        results.append(result)
    
    return results

# Generate comparison report
results = batch_planning_scenarios()

print(f"\n📈 Batch Planning Summary:")
print(f"{'Scenario':<20} {'Success':<8} {'Plan Time':<10} {'Duration':<10} {'Max Vel':<10}")
print("-" * 70)

for r in results:
    if r['success']:
        print(f"{r['scenario']:<20} {'✅':<8} {r['planning_time']:<10.2f} {r['duration']:<10.2f} {r['max_velocity']:<10.2f}")
    else:
        print(f"{r['scenario']:<20} {'❌':<8} {r['planning_time']:<10.2f} {'N/A':<10} {'N/A':<10}")
```

These examples cover the most common use cases and provide a foundation for building more complex applications with the `drone-pathgen` library.
