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
  // create a new voxelmap instance with the specified dimensions and scale
  map_ = std::make_unique<voxel_map::VoxelMap>(map_size, origin, voxel_scale);

  // mark all specified obstacle points as occupied in the voxel map
  for (const auto &pt : obstacle_points) {
    map_->setOccupied(pt);
  }

  // apply a safety margin by dilating the occupied voxels
  map_->dilate(dilation_radius);
}

void GCopterAPI::set_endpoints(const Eigen::Vector3d &start_pos,
                               const Eigen::Vector3d &goal_pos,
                               const Eigen::Vector3d &start_vel,
                               const Eigen::Vector3d &goal_vel) {
  // ensure the map is configured before setting endpoints
  if (!map_) {
    std::cerr << "error: must call configure_map() before set_endpoints()"
              << std::endl;
    return;
  }

  // check if start and goal positions are within the map boundaries
  Eigen::Vector3d map_origin = map_->getOrigin();
  Eigen::Vector3d map_corner = map_->getCorner();
  bool start_is_inside = (start_pos.array() >= map_origin.array()).all() &&
                         (start_pos.array() < map_corner.array()).all();
  bool goal_is_inside = (goal_pos.array() >= map_origin.array()).all() &&
                        (goal_pos.array() < map_corner.array()).all();

  // if either point is outside, report an error and do not set endpoints
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

  // store endpoint data and mark them as set
  start_position_ = start_pos;
  goal_position_ = goal_pos;
  start_velocity_ = start_vel;
  goal_velocity_ = goal_vel;
  endpoints_set_ = true;
}

bool GCopterAPI::run_inference(double planning_timeout, double time_weight,
                               double segment_length, double smoothing_epsilon,
                               int integral_resolution,
                               const Eigen::VectorXd &magnitude_bounds,
                               const Eigen::VectorXd &penalty_weights,
                               const Eigen::VectorXd &physical_params,
                               Trajectory<5> &out_traj) {

  // fail if the map or endpoints have not been set up
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

  // store physical parameters for use in control (flatness) calculations
  physical_params_ = physical_params;

  // step 1: find an initial geometric path using an ompl-based planner
  std::cout << "1. running ompl path planning..." << std::endl;
  const double path_cost = sfc_gen::planPath(
      start_position_, goal_position_, map_->getOrigin(), map_->getCorner(),
      map_.get(), planning_timeout, initial_route_);

  if (std::isinf(path_cost)) {
    std::cerr << "error: ompl failed to find a path in " << planning_timeout
              << "s." << std::endl;
    return false;
  }
  std::cout << "   ompl found a path with " << initial_route_.size()
            << " waypoints." << std::endl;

  // step 2: generate a safe flight corridor (sfc) around the path
  std::cout << "2. generating safe flight corridor..." << std::endl;
  std::vector<Eigen::MatrixX4d> h_polytopes;
  std::vector<Eigen::Vector3d> surface_points;
  map_->getSurf(surface_points);

  sfc_gen::convexCover(initial_route_, surface_points, map_->getOrigin(),
                       map_->getCorner(), 7.0, 3.0, h_polytopes);
  sfc_gen::shortCut(h_polytopes);
  std::cout << "   generated " << h_polytopes.size()
            << " convex polytopes for the sfc." << std::endl;

  // the optimizer requires at least two polytopes to define a trajectory
  if (h_polytopes.size() < 2) {
    std::cerr << "error: corridor generation failed (too few polytopes)."
              << std::endl;
    return false;
  }

  // step 3: setup and run the gcopter optimizer
  std::cout << "3. optimizing trajectory..." << std::endl;
  gcopter::GCOPTER_PolytopeSFC sfc_optimizer;

  // define initial and final state constraints for the optimizer
  Eigen::Matrix3d initial_state, final_state;
  initial_state.col(0) = start_position_;
  initial_state.col(1) = start_velocity_;
  initial_state.col(2) = Eigen::Vector3d::Zero(); // initial acceleration is zero

  final_state.col(0) = goal_position_;
  final_state.col(1) = goal_velocity_;
  final_state.col(2) = Eigen::Vector3d::Zero(); // final acceleration is zero

  // setup the optimizer
  if (!sfc_optimizer.setup(time_weight, initial_state, final_state, h_polytopes,
                           INFINITY, smoothing_epsilon, integral_resolution,
                           magnitude_bounds, penalty_weights, physical_params)) {
    std::cerr << "error: optimizer setup failed." << std::endl;
    return false;
  }

  // execute the optimization
  const double final_cost = sfc_optimizer.optimize(out_traj, 1.0e-3);

  if (std::isinf(final_cost)) {
    std::cerr << "error: trajectory optimization failed to converge."
              << std::endl;
    return false;
  }

  std::cout << "   trajectory optimized successfully. final cost: " << final_cost
            << std::endl;

  // store results and mark trajectory as computed
  trajectory_ = out_traj;
  optimization_cost_ = final_cost;
  trajectory_computed_ = true;

  // initialize flatness mapper for control calculations
  flatness_map_.reset(physical_params(0), physical_params(1),
                      physical_params(2), physical_params(3),
                      physical_params(4), physical_params(5));

  return true;
}

/**
 * @brief retrieves the drone's state at a specified time along the trajectory.
 *
 * this function retrieves the drone's state (position, velocity, acceleration,
 * and jerk) at a specified time along the computed trajectory.
 *
 * @param time desired query time along the trajectory.
 * @param state output reference to store the drone's state.
 * @return true if successful, false if time is out of bounds.
 */
bool GCopterAPI::getStateAtTime(double time, DroneState &state) const {
  if (!trajectory_computed_) {
    return false;
  }

  // check if the requested time is within the trajectory's duration
  if (time < 0.0 || time > trajectory_.getTotalDuration()) {
    std::cerr << "error: requested time " << time
              << " is outside the trajectory duration of "
              << trajectory_.getTotalDuration() << std::endl;
    return false;
  }

  // retrieve state from the 5th-order trajectory polynomial
  state.time = time;
  state.position = trajectory_.getPos(time);
  state.velocity = trajectory_.getVel(time);
  state.acceleration = trajectory_.getAcc(time);
  state.jerk = trajectory_.getJer(time);
  return true;
}

/**
 * @brief computes control inputs using flatness mapping.
 *
 * this function translates the high-level trajectory derivatives (vel, acc,
 * jerk) into low-level commands (thrust, attitude) that a drone can execute.
 *
 * @param time desired query time along the trajectory.
 * @param inputs output reference to store the control inputs.
 * @param yaw desired yaw angle.
 * @param yaw_rate desired yaw rate.
 * @return true if successful, false if time is out of bounds.
 */
bool GCopterAPI::getControlInputs(double time, ControlInputs &inputs,
                                  double yaw, double yaw_rate) const {
  if (!trajectory_computed_) {
    return false;
  }

  // check if the requested time is within the trajectory's duration
  if (time < 0.0 || time > trajectory_.getTotalDuration()) {
    std::cerr << "error: requested time " << time
              << " is outside the trajectory duration of "
              << trajectory_.getTotalDuration() << std::endl;
    return false;
  }

  // get trajectory derivatives at the specified time
  const Eigen::Vector3d vel = trajectory_.getVel(time);
  const Eigen::Vector3d acc = trajectory_.getAcc(time);
  const Eigen::Vector3d jerk = trajectory_.getJer(time);

  // compute control inputs from derivatives
  flatness_map_.forward(vel, acc, jerk, yaw, yaw_rate, inputs.thrust,
                        inputs.quaternion, inputs.angular_velocity);

  // populate the output struct
  inputs.yaw_angle = yaw;
  inputs.yaw_rate = yaw_rate;

  return true;
}

/**
 * @brief retrieves key statistics about the computed trajectory.
 * @param stats output reference to store the statistics.
 * @return true if a trajectory has been computed, false otherwise.
 */
bool GCopterAPI::getStatistics(TrajectoryStatistics &stats) const {
  if (!trajectory_computed_) {
    return false;
  }

  // populate the statistics struct from the trajectory object
  stats.total_duration = trajectory_.getTotalDuration();
  stats.num_pieces = trajectory_.getPieceNum();
  stats.optimization_cost = optimization_cost_;
  stats.max_velocity = trajectory_.getMaxVelRate();
  stats.max_acceleration = trajectory_.getMaxAccRate();
  stats.start_pos = start_position_;
  stats.goal_pos = goal_position_;

  return true;
}

/**
 * @brief prints a simple 2d projection of the voxel map to the console.
 * '1' represents an occupied or dilated voxel.
 * '0' represents free space.
 */
void GCopterAPI::print_voxel_map() const {
  if (!map_) {
    std::cerr << "error: map not configured." << std::endl;
    return;
  }
  const auto size = map_->getSize();
  const auto &voxels = map_->getVoxels();
  for (int z = 0; z < size(2); ++z) {
    std::cout << "layer " << z << ":\n";
    for (int y = 0; y < size(1); ++y) {
      for (int x = 0; x < size(0); ++x) {
        int idx = x + y * size(0) + z * size(0) * size(1);
        std::cout << (static_cast<int>(voxels[idx]) > 0 ? "1" : "0");
      }
      std::cout << '\n';
    }
  }
}

/**
 * @brief collects all necessary data for visualization.
 *
 * this function samples the trajectory and gathers map data to be used with
 * an external plotting library like open3d.
 *
 * @param trajectory_points output vector of 3d points along the trajectory.
 * @param voxel_data output 3d grid of voxel states.
 * @param voxel_size output voxel edge length.
 * @param start_pos output start position.
 * @param goal_pos output goal position.
 * @param show_initial_route flag to include the un-optimized path.
 * @param initial_route output vector for the initial path points.
 * @return true if successful, false if no trajectory is available.
 */
bool GCopterAPI::get_visualization_data(
    std::vector<Eigen::Vector3d> &trajectory_points,
    std::vector<std::vector<std::vector<int>>> &voxel_data, double &voxel_size,
    Eigen::Vector3d &start_pos, Eigen::Vector3d &goal_pos,
    bool show_initial_route,
    std::vector<Eigen::Vector3d> *initial_route_ptr) const {
  if (!trajectory_computed_ || !map_) {
    return false;
  }

  // sample trajectory points at a fixed interval for plotting
  trajectory_points.clear();
  double dt = 0.1;
  for (double t = 0.0; t < trajectory_.getTotalDuration(); t += dt) {
    trajectory_points.push_back(trajectory_.getPos(t));
  }
  if (trajectory_.getTotalDuration() > 0) {
    trajectory_points.push_back(trajectory_.getPos(trajectory_.getTotalDuration()));
  }

  // copy voxel data for visualization
  voxel_data.clear();
  const auto size = map_->getSize();
  const auto &voxels = map_->getVoxels();
  voxel_data.resize(size(2),
                    std::vector<std::vector<int>>(
                        size(1), std::vector<int>(size(0), 0)));

  for (int z = 0; z < size(2); ++z) {
    for (int y = 0; y < size(1); ++y) {
      for (int x = 0; x < size(0); ++x) {
        int idx = x + y * size(0) + z * size(0) * size(1);
        voxel_data[z][y][x] = static_cast<int>(voxels[idx]);
      }
    }
  }

  // provide other relevant data
  voxel_size = map_->getScale();
  start_pos = start_position_;
  goal_pos = goal_position_;

  // include the initial route if requested
  if (show_initial_route && initial_route_ptr) {
    *initial_route_ptr = initial_route_;
  }

  return true;
}

/**
 * @brief retrieves the initial, un-optimized path from ompl.
 * @param route output reference to store the path points.
 * @return true if the route is available, false otherwise.
 */
bool GCopterAPI::get_initial_route(std::vector<Eigen::Vector3d> &route) const {
  if (initial_route_.empty()) {
    return false;
  }
  route = initial_route_;
  return true;
}