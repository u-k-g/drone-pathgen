#include "api/gcopter_api.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/sfc_gen.hpp"
#include "gcopter/trajectory.hpp"
#include <Eigen/Eigen>
#include <iostream>

// implement gcopter api methods
void GCopterAPI::configure_map(
    const Eigen::Vector3i &map_size, const Eigen::Vector3d &origin,
    double voxel_scale, const std::vector<Eigen::Vector3d> &obstacle_points,
    int dilation_radius) {
  // allocate the voxelmap
  map_ = std::make_unique<voxel_map::VoxelMap>(map_size, origin, voxel_scale);
  // mark each obstacle
  for (const auto &pt : obstacle_points) {
    map_->setOccupied(pt);
  }
  // apply safety dilation
  map_->dilate(dilation_radius);

  // debug: verify obstacle placement in voxel coordinates
  std::cout << "debug: verifying obstacle placement..." << std::endl;
  std::cout << "map origin: " << map_->getOrigin().transpose() << std::endl;
  std::cout << "map corner: " << map_->getCorner().transpose() << std::endl;
  std::cout << "voxel scale: " << map_->getScale() << std::endl;

  for (size_t i = 0; i < obstacle_points.size(); ++i) {
    const auto &obs = obstacle_points[i];
    // convert world coordinate to voxel index
    Eigen::Vector3d voxel_pos = (obs - map_->getOrigin()) / map_->getScale();
    Eigen::Vector3i voxel_idx = voxel_pos.cast<int>();
    uint8_t voxel_val = map_->query(obs);

    std::cout << "  obstacle " << i << ": world=" << obs.transpose()
              << " → voxel_pos=" << voxel_pos.transpose()
              << " → voxel_idx=" << voxel_idx.transpose()
              << " → query_result=" << static_cast<int>(voxel_val) << std::endl;
  }

  // output labeled voxel grid (0=unoccupied, 1=occupied, 2=dilated)
  const auto size = map_->getSize();
  const auto &voxels = map_->getVoxels();
  for (int z = 0; z < size(2); ++z) {
    std::cout << "layer " << z << ":\n";
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
  // check if map has been configured first
  if (!map_) {
    std::cerr << "error: must call configure_map() before set_endpoints()"
              << std::endl;
    return;
  }

  // check if start and goal are inside the map boundaries
  Eigen::Vector3d map_origin = map_->getOrigin();
  Eigen::Vector3d map_corner = map_->getCorner();
  bool start_is_inside = (start_pos.array() >= map_origin.array()).all() &&
                         (start_pos.array() < map_corner.array()).all();
  bool goal_is_inside = (goal_pos.array() >= map_origin.array()).all() &&
                        (goal_pos.array() < map_corner.array()).all();

  if (!start_is_inside || !goal_is_inside) {
    std::cerr << "error: start or goal position is outside the map bounds."
              << std::endl;
    if (!start_is_inside) {
      std::cerr << "  start: " << start_pos.transpose() << " is outside."
                << std::endl;
    }
    if (!goal_is_inside) {
      std::cerr << "  goal: " << goal_pos.transpose() << " is outside."
                << std::endl;
    }
    std::cerr << "  map bounds are from " << map_origin.transpose() << " to "
              << map_corner.transpose() << std::endl;
    endpoints_set_ = false;
    return;
  }

  // store endpoint data
  start_position_ = start_pos;
  goal_position_ = goal_pos;
  start_velocity_ = start_vel;
  goal_velocity_ = goal_vel;
  endpoints_set_ = true;

  std::cout << "endpoints set:" << std::endl;
  std::cout << "  start: " << start_position_.transpose() << std::endl;
  std::cout << "  goal:  " << goal_position_.transpose() << std::endl;
  std::cout << "  start vel: " << start_velocity_.transpose() << std::endl;
  std::cout << "  goal vel:  " << goal_velocity_.transpose() << std::endl;
}

bool GCopterAPI::run_inference(double planning_timeout, double time_weight,
                               double segment_length, double smoothing_epsilon,
                               int integral_resolution,
                               const Eigen::VectorXd &magnitude_bounds,
                               const Eigen::VectorXd &penalty_weights,
                               const Eigen::VectorXd &physical_params,
                               Trajectory<5> &out_traj) {

  // basic validation checks
  if (!map_) {
    std::cerr << "error: map not configured. call configure_map() first."
              << std::endl;
    return false;
  }
  if (!endpoints_set_) {
    std::cerr << "error: endpoints not set. call set_endpoints() first."
              << std::endl;
    return false;
  }

  // store physical parameters for later use in control computation
  physical_params_ = physical_params;

  // find an initial geometric path using ompl
  std::vector<Eigen::Vector3d> initial_path;

  std::cout << "running ompl path planning..." << std::endl;
  std::cout << "  start: " << start_position_.transpose() << std::endl;
  std::cout << "  goal: " << goal_position_.transpose() << std::endl;
  std::cout << "  timeout: " << planning_timeout << "s" << std::endl;

  const double path_cost = sfc_gen::planPath(
      start_position_, goal_position_, map_->getOrigin(), map_->getCorner(),
      map_.get(), planning_timeout, initial_path);

  if (std::isinf(path_cost)) {
    std::cerr << "error: ompl failed to find a path in " << planning_timeout
              << "s." << std::endl;
    return false;
  }
  std::cout << "ompl found a path with cost: " << path_cost << std::endl;
  std::cout << "path has " << initial_path.size() << " waypoints." << std::endl;

  // generate a safe flight corridor (sfc) around the path
  std::vector<Eigen::MatrixX4d> h_polytopes;
  std::vector<Eigen::Vector3d> surface_points;
  map_->getSurf(surface_points);

  double progress_step = 7.0;
  double corridor_range = 3.0;

  std::cout << "corridor generation parameters:" << std::endl;
  std::cout << "  progress_step: " << progress_step << "m" << std::endl;
  std::cout << "  corridor_range: " << corridor_range << "m" << std::endl;
  std::cout << "  surface_points: " << surface_points.size() << std::endl;

  sfc_gen::convexCover(initial_path, surface_points, map_->getOrigin(),
                       map_->getCorner(), progress_step, corridor_range,
                       h_polytopes);
  sfc_gen::shortCut(h_polytopes);
  std::cout << "generated " << h_polytopes.size()
            << " convex polytopes for the sfc." << std::endl;

  // validate that we have a reasonable number of polytopes
  if (h_polytopes.size() < 2) {
    std::cerr << "error: too few corridor polytopes generated ("
              << h_polytopes.size() << "). cannot create safe trajectory."
              << std::endl;
    return false;
  }

  // validate the initial ompl path against voxel map
  std::cout << "validating initial ompl path against voxel map..." << std::endl;
  int initial_violations = 0;
  for (size_t i = 0; i < initial_path.size(); ++i) {
    const auto &pos = initial_path[i];
    uint8_t voxel_val = map_->query(pos);
    if (voxel_val != 0) {
      initial_violations++;
    }
  }
  if (initial_violations > 0) {
    std::cout << "⚠️  initial ompl path has " << initial_violations
              << " violations! this suggests ompl planning failed."
              << std::endl;
  } else {
    std::cout << "✅ initial ompl path is collision-free." << std::endl;
  }

  // setup the gcopter optimizer
  gcopter::GCOPTER_PolytopeSFC sfc_optimizer;

  // define initial and terminal states (position, velocity, acceleration)
  Eigen::Matrix3d initial_state, final_state;
  initial_state.col(0) = start_position_;
  initial_state.col(1) = start_velocity_;
  initial_state.col(2) = Eigen::Vector3d::Zero(); // zero initial acceleration

  final_state.col(0) = goal_position_;
  final_state.col(1) = goal_velocity_;
  final_state.col(2) = Eigen::Vector3d::Zero(); // zero final acceleration

  if (!sfc_optimizer.setup(time_weight, initial_state, final_state, h_polytopes,
                           INFINITY, smoothing_epsilon, integral_resolution,
                           magnitude_bounds, penalty_weights,
                           physical_params)) {
    std::cerr << "error: optimizer setup failed." << std::endl;
    return false;
  }

  // run the optimization
  std::cout << "running trajectory optimization..." << std::endl;
  const double final_cost = sfc_optimizer.optimize(out_traj, 1.0e-3);

  if (std::isinf(final_cost)) {
    std::cerr << "error: trajectory optimization failed to converge."
              << std::endl;
    return false;
  }

  std::cout << "optimization successful! final cost: " << final_cost
            << std::endl;

  // validate the trajectory against the voxel map
  std::cout << "validating trajectory against voxel map..." << std::endl;
  int violations = 0;
  double total_duration = out_traj.getTotalDuration();
  double dt = 0.1; // check every 0.1 seconds

  for (double t = 0.0; t <= total_duration; t += dt) {
    Eigen::Vector3d pos = out_traj.getPos(t);
    if (map_->query(pos) != 0) { // should be 0 (unoccupied)
      violations++;
      if (violations <= 5) { // only print first few violations
        std::cout << "  warning: trajectory violates voxel map at t=" << t
                  << ", pos=" << pos.transpose()
                  << ", voxel_value=" << static_cast<int>(map_->query(pos))
                  << std::endl;
      }
    }
  }

  if (violations > 0) {
    std::cout << "⚠️  trajectory validation failed: " << violations
              << " violations detected!" << std::endl;
    std::cout << "   trajectory passes through " << violations
              << " occupied/dilated voxels." << std::endl;
  } else {
    std::cout << "✅ trajectory validation passed - no voxel violations."
              << std::endl;
  }

  // store results for later queries
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
    std::cerr << "error: no trajectory computed. call run_inference() first."
              << std::endl;
    return false;
  }

  double total_duration = trajectory_.getTotalDuration();
  if (time < 0.0 || time > total_duration) {
    std::cerr << "error: time " << time << " is outside trajectory bounds [0, "
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
    std::cerr << "error: no trajectory computed. call run_inference() first."
              << std::endl;
    return false;
  }

  double total_duration = trajectory_.getTotalDuration();
  if (time < 0.0 || time > total_duration) {
    std::cerr << "error: time " << time << " is outside trajectory bounds [0, "
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
    std::cerr << "error: no trajectory computed. call run_inference() first."
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
    std::cerr << "error: no map configured. call configure_map() first."
              << std::endl;
    return;
  }

  const auto size = map_->getSize();
  const auto &voxels = map_->getVoxels();

  std::cout << "voxelmap size: " << size.transpose() << std::endl;
  std::cout << "origin: " << map_->getOrigin().transpose() << std::endl;
  std::cout << "scale: " << map_->getScale() << std::endl;
  std::cout << "corner: " << map_->getCorner().transpose() << std::endl;

  for (int z = 0; z < size(2); ++z) {
    std::cout << "layer " << z << ":\n";
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
    std::vector<std::vector<std::vector<int>>> &voxel_data, double &voxel_size,
    Eigen::Vector3d &start_pos, Eigen::Vector3d &goal_pos,
    bool show_initial_route,
    std::vector<Eigen::Vector3d> *initial_route) const {

  // check if we have computed trajectory and map
  if (!trajectory_computed_ || !map_) {
    std::cerr << "error: no trajectory computed or map not configured."
              << std::endl;
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
    std::cerr
        << "error: no initial route available. call run_inference() first."
        << std::endl;
    return false;
  }

  route = initial_route_;
  return true;
}