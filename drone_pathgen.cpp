#include "api/gcopter_api.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/sfc_gen.hpp"
#include "gcopter/trajectory.hpp"
#include <Eigen/Eigen>
#include <iostream>

class DronePathGenerator {
private:
  GCopterAPI gc_api_;

public:
  bool configureMap(const Eigen::Vector3i &map_size,
                    const Eigen::Vector3d &origin, double voxel_scale,
                    const std::vector<Eigen::Vector3d> &obstacle_points,
                    int dilation_radius = 1) {
    std::cout << "Configuring map..." << std::endl;
    gc_api_.configure_map(map_size, origin, voxel_scale, obstacle_points,
                          dilation_radius);
    return true;
  }
  bool setEndpoints(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                    const Eigen::Vector3d &start_vel = Eigen::Vector3d::Zero(),
                    const Eigen::Vector3d &goal_vel = Eigen::Vector3d::Zero()) {
    gc_api_.set_endpoints(start, goal, start_vel, goal_vel);
    return true;
  }

  bool runInference(double planning_timeout, double time_weight,
                    double segment_length, double smoothing_epsilon,
                    int integral_resolution,
                    const Eigen::VectorXd &magnitude_bounds,
                    const Eigen::VectorXd &penalty_weights,
                    const Eigen::VectorXd &physical_params,
                    Trajectory<5> &out_traj) {
    return gc_api_.run_inference(planning_timeout, time_weight, segment_length,
                                 smoothing_epsilon, integral_resolution,
                                 magnitude_bounds, penalty_weights,
                                 physical_params, out_traj);
  }
};

// Implement GCopterAPI methods
void GCopterAPI::configure_map(
    const Eigen::Vector3i &map_size, const Eigen::Vector3d &origin,
    double voxel_scale, const std::vector<Eigen::Vector3d> &obstacle_points,
    int dilation_radius) {
  // Allocate the VoxelMap
  map_ = std::make_unique<voxel_map::VoxelMap>(map_size, origin, voxel_scale);
  // Mark each obstacle
  for (const auto &pt : obstacle_points) {
    map_->setOccupied(pt);
  }
  // Apply safety dilation
  map_->dilate(dilation_radius);

  // Output labeled voxel grid (0=Unoccupied, 1=Occupied, 2=Dilated)
  const auto size = map_->getSize();
  const auto &voxels = map_->getVoxels();
  for (int z = 0; z < size(2); ++z) {
    std::cout << "Layer " << z << ":\n";
    for (int y = 0; y < size(1); ++y) {
      for (int x = 0; x < size(0); ++x) {
        int idx = x + y * size(0) + z * size(0) * size(1);
        std::cout << static_cast<int>(voxels[idx]);
      }
      std::cout << '\n';
    }
  }
}

void GCopterAPI::set_endpoints(const Eigen::Vector3d &start_pos,
                               const Eigen::Vector3d &goal_pos,
                               const Eigen::Vector3d &start_vel,
                               const Eigen::Vector3d &goal_vel) {
  // Check if map has been configured first
  if (!map_) {
    std::cerr << "Error: Must call configure_map() before set_endpoints()"
              << std::endl;
    return;
  }

  // Manually check if start and goal are inside the map boundaries
  Eigen::Vector3d map_origin = map_->getOrigin();
  Eigen::Vector3d map_corner = map_->getCorner();
  bool start_is_inside = (start_pos.array() >= map_origin.array()).all() &&
                         (start_pos.array() < map_corner.array()).all();
  bool goal_is_inside = (goal_pos.array() >= map_origin.array()).all() &&
                        (goal_pos.array() < map_corner.array()).all();

  if (!start_is_inside || !goal_is_inside) {
    std::cerr << "ERROR: Start or goal position is outside the map bounds."
              << std::endl;
    if (!start_is_inside) {
      std::cerr << "  Start: " << start_pos.transpose() << " is outside."
                << std::endl;
    }
    if (!goal_is_inside) {
      std::cerr << "  Goal: " << goal_pos.transpose() << " is outside."
                << std::endl;
    }
    std::cerr << "  Map bounds are from " << map_origin.transpose() << " to "
              << map_corner.transpose() << std::endl;
    endpoints_set_ = false;
    return;
  }

  // Store endpoint data
  start_position_ = start_pos;
  goal_position_ = goal_pos;
  start_velocity_ = start_vel;
  goal_velocity_ = goal_vel;
  endpoints_set_ = true;

  std::cout << "Endpoints set:" << std::endl;
  std::cout << "  Start: " << start_position_.transpose() << std::endl;
  std::cout << "  Goal:  " << goal_position_.transpose() << std::endl;
  std::cout << "  Start vel: " << start_velocity_.transpose() << std::endl;
  std::cout << "  Goal vel:  " << goal_velocity_.transpose() << std::endl;
}

bool GCopterAPI::run_inference(double planning_timeout, double time_weight,
                               double segment_length, double smoothing_epsilon,
                               int integral_resolution,
                               const Eigen::VectorXd &magnitude_bounds,
                               const Eigen::VectorXd &penalty_weights,
                               const Eigen::VectorXd &physical_params,
                               Trajectory<5> &out_traj) {

  // Step 0: Basic validation checks
  if (!map_) {
    std::cerr << "ERROR: Map not configured. Call configure_map() first."
              << std::endl;
    return false;
  }
  if (!endpoints_set_) {
    std::cerr << "ERROR: Endpoints not set. Call set_endpoints() first."
              << std::endl;
    return false;
  }

  // Step 1: Find an initial geometric path using OMPL
  std::vector<Eigen::Vector3d> initial_path;
  const double path_cost = sfc_gen::planPath(
      start_position_, goal_position_, map_->getOrigin(), map_->getCorner(),
      map_.get(), planning_timeout, initial_path);

  if (std::isinf(path_cost)) {
    std::cerr << "ERROR: OMPL failed to find a path in " << planning_timeout
              << "s." << std::endl;
    return false;
  }
  std::cout << "OMPL found a path with cost: " << path_cost << std::endl;

  // Print the initial path (polyline) for visualization
  std::cout << "POLYLINE" << std::endl;
  for (const auto &point : initial_path) {
    std::cout << point.x() << " " << point.y() << " " << point.z() << std::endl;
  }

  // Step 2: Generate a Safe Flight Corridor (SFC) around the path
  std::vector<Eigen::MatrixX4d> h_polytopes;
  std::vector<Eigen::Vector3d> surface_points;
  map_->getSurf(surface_points);
  // Use extremely conservative parameters to avoid obstacles
  double progress_step = segment_length * 0.5;   // very small segments for maximum control
  double corridor_range = segment_length * 0.1;  // extremely narrow corridors to force tight following of path
  sfc_gen::convexCover(initial_path, surface_points, map_->getOrigin(),
                       map_->getCorner(), progress_step, corridor_range,
                       h_polytopes);
  sfc_gen::shortCut(h_polytopes);
  std::cout << "Generated " << h_polytopes.size()
            << " convex polytopes for the SFC." << std::endl;

  // Validate that we have a reasonable number of polytopes
  if (h_polytopes.size() < 2) {
    std::cerr << "ERROR: Too few corridor polytopes generated (" << h_polytopes.size() 
              << "). Cannot create safe trajectory." << std::endl;
    return false;
  }

  // Step 3: Setup the GCOPTER optimizer
  gcopter::GCOPTER_PolytopeSFC sfc_optimizer;

  // Define initial and terminal states (P, V, A)
  Eigen::Matrix3d initial_state, final_state;
  initial_state.col(0) = start_position_;
  initial_state.col(1) = start_velocity_;
  initial_state.col(2) =
      Eigen::Vector3d::Zero(); // Assume zero initial acceleration

  final_state.col(0) = goal_position_;
  final_state.col(1) = goal_velocity_;
  final_state.col(2) =
      Eigen::Vector3d::Zero(); // Assume zero final acceleration

  if (!sfc_optimizer.setup(time_weight, initial_state, final_state, h_polytopes,
                           segment_length, smoothing_epsilon,
                           integral_resolution, magnitude_bounds,
                           penalty_weights, physical_params)) {
    std::cerr << "ERROR: Optimizer setup failed." << std::endl;
    return false;
  }

  // Step 4: Run the optimization
  std::cout << "Running trajectory optimization..." << std::endl;
  // The second argument is a relative cost tolerance for stopping
  const double final_cost = sfc_optimizer.optimize(out_traj, 1.0e-5);

  if (std::isinf(final_cost)) {
    std::cerr << "ERROR: Trajectory optimization failed to converge."
              << std::endl;
    return false;
  }

  std::cout << "Optimization successful! Final cost: " << final_cost
            << std::endl;

  // Step 5: Validate the trajectory against the voxel map
  std::cout << "Validating trajectory against voxel map..." << std::endl;
  int violations = 0;
  double total_duration = out_traj.getTotalDuration();
  double dt = 0.1;  // Check every 0.1 seconds
  
  for (double t = 0.0; t <= total_duration; t += dt) {
    Eigen::Vector3d pos = out_traj.getPos(t);
    if (map_->query(pos) != 0) {  // Should be 0 (unoccupied)
      violations++;
      if (violations <= 5) {  // Only print first few violations
        std::cout << "  WARNING: Trajectory violates voxel map at t=" << t 
                  << ", pos=" << pos.transpose() 
                  << ", voxel_value=" << static_cast<int>(map_->query(pos)) << std::endl;
      }
    }
  }
  
  if (violations > 0) {
    std::cout << "⚠️  TRAJECTORY VALIDATION FAILED: " << violations 
              << " violations detected!" << std::endl;
    std::cout << "   Trajectory passes through " << violations 
              << " occupied/dilated voxels." << std::endl;
  } else {
    std::cout << "✅ Trajectory validation passed - no voxel violations." << std::endl;
  }

  return true;
}