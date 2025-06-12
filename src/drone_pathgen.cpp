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

  // debug: verify obstacle placement in voxel coordinates
  std::cout << "Debug: Verifying obstacle placement..." << std::endl;
  std::cout << "Map origin: " << map_->getOrigin().transpose() << std::endl;
  std::cout << "Map corner: " << map_->getCorner().transpose() << std::endl;
  std::cout << "Voxel scale: " << map_->getScale() << std::endl;
  
  for (size_t i = 0; i < obstacle_points.size(); ++i) {
    const auto& obs = obstacle_points[i];
    // convert world coordinate to voxel index
    Eigen::Vector3d voxel_pos = (obs - map_->getOrigin()) / map_->getScale();
    Eigen::Vector3i voxel_idx = voxel_pos.cast<int>();
    uint8_t voxel_val = map_->query(obs);
    
    std::cout << "  Obstacle " << i << ": world=" << obs.transpose() 
              << " → voxel_pos=" << voxel_pos.transpose()
              << " → voxel_idx=" << voxel_idx.transpose()
              << " → query_result=" << static_cast<int>(voxel_val) << std::endl;
  }

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

  // step 0: basic validation checks
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

  // store physical parameters for later use in control computation
  physical_params_ = physical_params;

  // step 1: find an initial geometric path using OMPL
  std::vector<Eigen::Vector3d> initial_path;
  
  std::cout << "Running OMPL path planning..." << std::endl;
  std::cout << "  Start: " << start_position_.transpose() << std::endl;
  std::cout << "  Goal: " << goal_position_.transpose() << std::endl;
  std::cout << "  Timeout: " << planning_timeout << "s" << std::endl;
  
  const double path_cost = sfc_gen::planPath(
      start_position_, goal_position_, map_->getOrigin(), map_->getCorner(),
      map_.get(), planning_timeout, initial_path);

  if (std::isinf(path_cost)) {
    std::cerr << "ERROR: OMPL failed to find a path in " << planning_timeout
              << "s." << std::endl;
    return false;
  }
  std::cout << "OMPL found a path with cost: " << path_cost << std::endl;
  std::cout << "Path has " << initial_path.size() << " waypoints." << std::endl;

  // print the initial path (polyline) for visualization
  std::cout << "POLYLINE" << std::endl;
  for (const auto &point : initial_path) {
    std::cout << point.x() << " " << point.y() << " " << point.z() << std::endl;
  }

  // step 2: generate a Safe Flight Corridor (SFC) around the path
  std::vector<Eigen::MatrixX4d> h_polytopes;
  std::vector<Eigen::Vector3d> surface_points;
  map_->getSurf(surface_points);
  
  // use parameters matching the original GCOPTER example
  double progress_step = 7.0;  // from original example
  double corridor_range = 3.0; // from original example
  
  std::cout << "Corridor generation parameters:" << std::endl;
  std::cout << "  progress_step: " << progress_step << "m" << std::endl;
  std::cout << "  corridor_range: " << corridor_range << "m" << std::endl;
  std::cout << "  surface_points: " << surface_points.size() << std::endl;
  
  // debug: print first few surface points
  std::cout << "  First few surface points:" << std::endl;
  for (size_t i = 0; i < std::min(surface_points.size(), size_t(5)); ++i) {
    const auto& sp = surface_points[i];
    std::cout << "    [" << i << "]: " << sp.transpose() << std::endl;
  }

  sfc_gen::convexCover(initial_path, surface_points, map_->getOrigin(),
                       map_->getCorner(), progress_step, corridor_range,
                       h_polytopes);
  sfc_gen::shortCut(h_polytopes);
  std::cout << "Generated " << h_polytopes.size()
            << " convex polytopes for the SFC." << std::endl;

  // debug: print some polytope information
  for (size_t i = 0; i < std::min(h_polytopes.size(), size_t(3)); ++i) {
    std::cout << "  Polytope " << i << " has " << h_polytopes[i].rows() 
              << " constraints (halfspaces)" << std::endl;
  }

  // validate that we have a reasonable number of polytopes
  if (h_polytopes.size() < 2) {
    std::cerr << "ERROR: Too few corridor polytopes generated ("
              << h_polytopes.size() << "). Cannot create safe trajectory."
              << std::endl;
    return false;
  }

  // debug: validate the initial OMPL path against voxel map
  std::cout << "Validating initial OMPL path against voxel map..." << std::endl;
  int initial_violations = 0;
  for (size_t i = 0; i < initial_path.size(); ++i) {
    const auto& pos = initial_path[i];
    uint8_t voxel_val = map_->query(pos);
    if (voxel_val != 0) {
      initial_violations++;
      if (initial_violations <= 3) {
        std::cout << "  WARNING: Initial path point " << i << " at " 
                  << pos.transpose() << " has voxel value " 
                  << static_cast<int>(voxel_val) << std::endl;
      }
    }
  }
  if (initial_violations > 0) {
    std::cout << "⚠️  Initial OMPL path has " << initial_violations 
              << " violations! This suggests OMPL planning failed." << std::endl;
  } else {
    std::cout << "✅ Initial OMPL path is collision-free." << std::endl;
  }

  // step 3: setup the GCOPTER optimizer
  gcopter::GCOPTER_PolytopeSFC sfc_optimizer;

  // define initial and terminal states (P, V, A)
  Eigen::Matrix3d initial_state, final_state;
  initial_state.col(0) = start_position_;
  initial_state.col(1) = start_velocity_;
  initial_state.col(2) =
      Eigen::Vector3d::Zero(); // assume zero initial acceleration

  final_state.col(0) = goal_position_;
  final_state.col(1) = goal_velocity_;
  final_state.col(2) =
      Eigen::Vector3d::Zero(); // assume zero final acceleration

  if (!sfc_optimizer.setup(time_weight, initial_state, final_state, h_polytopes,
                           INFINITY, smoothing_epsilon,
                           integral_resolution, magnitude_bounds,
                           penalty_weights, physical_params)) {
    std::cerr << "ERROR: Optimizer setup failed." << std::endl;
    return false;
  }

  // step 4: run the optimization
  std::cout << "Running trajectory optimization..." << std::endl;
  // the second argument is a relative cost tolerance for stopping
  const double final_cost = sfc_optimizer.optimize(out_traj, 1.0e-3);

  if (std::isinf(final_cost)) {
    std::cerr << "ERROR: Trajectory optimization failed to converge."
              << std::endl;
    return false;
  }

  std::cout << "Optimization successful! Final cost: " << final_cost
            << std::endl;

  // step 5: validate the trajectory against the voxel map
  std::cout << "Validating trajectory against voxel map..." << std::endl;
  int violations = 0;
  double total_duration = out_traj.getTotalDuration();
  double dt = 0.1; // check every 0.1 seconds

  for (double t = 0.0; t <= total_duration; t += dt) {
    Eigen::Vector3d pos = out_traj.getPos(t);
    if (map_->query(pos) != 0) { // should be 0 (unoccupied)
      violations++;
      if (violations <= 5) { // only print first few violations
        std::cout << "  WARNING: Trajectory violates voxel map at t=" << t
                  << ", pos=" << pos.transpose()
                  << ", voxel_value=" << static_cast<int>(map_->query(pos))
                  << std::endl;
      }
    }
  }

  if (violations > 0) {
    std::cout << "⚠️  TRAJECTORY VALIDATION FAILED: " << violations
              << " violations detected!" << std::endl;
    std::cout << "   Trajectory passes through " << violations
              << " occupied/dilated voxels." << std::endl;
  } else {
    std::cout << "✅ Trajectory validation passed - no voxel violations."
              << std::endl;
  }

  // step 6: store results for later queries
  trajectory_ = out_traj;
  trajectory_computed_ = true;
  optimization_cost_ = final_cost;

  // store the initial path for visualization
  initial_route_ = initial_path;

  // initialize flatness mapper with physical parameters
  flatness_map_.reset(physical_params(0),  // mass
                      physical_params(1),  // gravitational acceleration
                      physical_params(2),  // horizontal drag coeff
                      physical_params(3),  // vertical drag coeff
                      physical_params(4),  // parasitic drag coeff
                      physical_params(5)); // speed smooth factor

  return true;
}

bool GCopterAPI::getStateAtTime(double time, DroneState &state) const {
  if (!trajectory_computed_) {
    std::cerr << "ERROR: No trajectory computed. Call run_inference() first."
              << std::endl;
    return false;
  }

  double total_duration = trajectory_.getTotalDuration();
  if (time < 0.0 || time > total_duration) {
    std::cerr << "ERROR: Time " << time << " is outside trajectory bounds [0, "
              << total_duration << "]" << std::endl;
    return false;
  }

  state.time = time;
  state.position = trajectory_.getPos(time);
  state.velocity = trajectory_.getVel(time);
  state.acceleration = trajectory_.getAcc(time);
  state.jerk = trajectory_.getJer(time);

  return true;
}

bool GCopterAPI::getControlInputs(double time, ControlInputs &inputs,
                                  double yaw, double yaw_rate) const {
  if (!trajectory_computed_) {
    std::cerr << "ERROR: No trajectory computed. Call run_inference() first."
              << std::endl;
    return false;
  }

  double total_duration = trajectory_.getTotalDuration();
  if (time < 0.0 || time > total_duration) {
    std::cerr << "ERROR: Time " << time << " is outside trajectory bounds [0, "
              << total_duration << "]" << std::endl;
    return false;
  }

  // get trajectory derivatives at time t
  Eigen::Vector3d vel = trajectory_.getVel(time);
  Eigen::Vector3d acc = trajectory_.getAcc(time);
  Eigen::Vector3d jer = trajectory_.getJer(time);

  // compute control inputs using flatness mapping
  flatness_map_.forward(vel, acc, jer, yaw, yaw_rate, inputs.thrust,
                        inputs.quaternion, inputs.angular_velocity);

  // store the input parameters
  inputs.yaw_angle = yaw;
  inputs.yaw_rate = yaw_rate;

  return true;
}

bool GCopterAPI::getStatistics(TrajectoryStatistics &stats) const {
  if (!trajectory_computed_) {
    std::cerr << "ERROR: No trajectory computed. Call run_inference() first."
              << std::endl;
    return false;
  }

  stats.total_duration = trajectory_.getTotalDuration();
  stats.num_pieces = trajectory_.getPieceNum();
  stats.optimization_cost = optimization_cost_;
  stats.max_velocity = trajectory_.getMaxVelRate();
  stats.max_acceleration = trajectory_.getMaxAccRate();
  stats.start_pos = start_position_;
  stats.goal_pos = goal_position_;

  return true;
}

void GCopterAPI::print_voxel_map() const {
  if (!map_) {
    std::cerr << "ERROR: No map configured. Call configure_map() first."
              << std::endl;
    return;
  }

  const auto size = map_->getSize();
  const auto &voxels = map_->getVoxels();

  std::cout << "VoxelMap Size: " << size.transpose() << std::endl;
  std::cout << "Origin: " << map_->getOrigin().transpose() << std::endl;
  std::cout << "Scale: " << map_->getScale() << std::endl;
  std::cout << "Corner: " << map_->getCorner().transpose() << std::endl;

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

bool GCopterAPI::get_visualization_data(
    std::vector<Eigen::Vector3d> &trajectory_points,
    std::vector<std::vector<std::vector<int>>> &voxel_data,
    double &voxel_size,
    Eigen::Vector3d &start_pos,
    Eigen::Vector3d &goal_pos,
    bool show_initial_route,
    std::vector<Eigen::Vector3d> *initial_route) const {
  
  // check if we have computed trajectory and map
  if (!trajectory_computed_ || !map_) {
    std::cerr << "ERROR: No trajectory computed or map not configured." << std::endl;
    return false;
  }

  // extract trajectory points (sample every 0.1 seconds)
  trajectory_points.clear();
  double total_duration = trajectory_.getTotalDuration();
  double dt = 0.1;
  
  for (double t = 0.0; t <= total_duration; t += dt) {
    trajectory_points.push_back(trajectory_.getPos(t));
  }
  // add final point
  if (total_duration > 0.0) {
    trajectory_points.push_back(trajectory_.getPos(total_duration));
  }

  // extract voxel data
  const auto size = map_->getSize();
  const auto &voxels = map_->getVoxels();
  voxel_size = map_->getScale();
  
  // convert to 3d vector format expected by python
  voxel_data.resize(size(2));
  for (int z = 0; z < size(2); ++z) {
    voxel_data[z].resize(size(1));
    for (int y = 0; y < size(1); ++y) {
      voxel_data[z][y].resize(size(0));
      for (int x = 0; x < size(0); ++x) {
        int idx = x + y * size(0) + z * size(0) * size(1);
        voxel_data[z][y][x] = static_cast<int>(voxels[idx]);
      }
    }
  }

  // set start and goal positions
  start_pos = start_position_;
  goal_pos = goal_position_;

  // optionally include initial route
  if (show_initial_route && initial_route && !initial_route_.empty()) {
    *initial_route = initial_route_;
  }

  return true;
}

bool GCopterAPI::get_initial_route(std::vector<Eigen::Vector3d> &route) const {
  if (initial_route_.empty()) {
    std::cerr << "ERROR: No initial route available. Call run_inference() first." << std::endl;
    return false;
  }
  
  route = initial_route_;
  return true;
}