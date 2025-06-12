#include "gcopter/flatness.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/sfc_gen.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/voxel_map.hpp"
#include <Eigen/Eigen>
#include <memory>
#include <string>
#include <vector>

// an api for the gcopter library, providing a high-level interface for
// trajectory planning. this class manages the state of the map, endpoints,
// and optimization parameters. it is not dependent on ros.

// holds the complete kinematic state of the drone at a specific time.
struct DroneState {
  Eigen::Vector3d position;     // position [m]
  Eigen::Vector3d velocity;     // velocity [m/s]
  Eigen::Vector3d acceleration; // acceleration [m/s^2]
  Eigen::Vector3d jerk;         // jerk [m/s^3]
  double time;                  // time from trajectory start [s]
};

// holds the computed control commands for the drone.
struct ControlInputs {
  double thrust;                    // required thrust command [n]
  Eigen::Vector4d quaternion;       // attitude quaternion [w, x, y, z]
  Eigen::Vector3d angular_velocity; // body angular velocity [rad/s]
  double yaw_angle;                 // desired yaw angle [rad]
  double yaw_rate;                  // desired yaw rate [rad/s]
};

// holds performance and summary metrics for a computed trajectory.
struct TrajectoryStatistics {
  double total_duration;    // total trajectory time [s]
  int num_pieces;           // number of polynomial pieces in the trajectory
  double optimization_cost; // final cost from the optimization
  double max_velocity;      // maximum velocity magnitude [m/s]
  double max_acceleration;  // maximum acceleration magnitude [m/s^2]
  Eigen::Vector3d start_pos; // trajectory start position [m]
  Eigen::Vector3d goal_pos;  // trajectory goal position [m]
};

/**
 * @class GCopterAPI
 * @brief provides a simplified, high-level interface for gcopter functionality.
 *
 * this class handles the setup of the environment (voxel map), defines the
 * trajectory planning problem (start/goal), runs the optimization, and provides
 * methods to query the results.
 */
class GCopterAPI {
public:
  /**
   * Build a VoxelMap from raw obstacle data and apply safety dilation.
   *
   * @param map_size         number of voxels along each axis (x, y, z).
   * @param origin           world coordinate of the map's origin (0,0,0) corner.
   * @param voxel_scale      edge length of a single voxel cube (meters).
   * @param obstacle_points  a list of 3d points representing obstacles.
   * @param dilation_radius  number of voxels to dilate around each obstacle for
   * safety.
   */
  void configure_map(const Eigen::Vector3i &map_size,
                     const Eigen::Vector3d &origin, double voxel_scale,
                     const std::vector<Eigen::Vector3d> &obstacle_points,
                     int dilation_radius = 1);
  /**
   * sets the start and goal points for the trajectory.
   *
   * @param start_pos    starting position in world coordinates [m].
   * @param goal_pos     goal position in world coordinates [m].
   * @param start_vel    initial velocity [m/s] (optional, defaults to zero).
   * @param goal_vel     final velocity [m/s] (optional, defaults to zero).
   */
  void set_endpoints(const Eigen::Vector3d &start_pos,
                     const Eigen::Vector3d &goal_pos,
                     const Eigen::Vector3d &start_vel = Eigen::Vector3d::Zero(),
                     const Eigen::Vector3d &goal_vel = Eigen::Vector3d::Zero());

  /**
   * runs the full trajectory planning and optimization pipeline.
   *
   * this process involves:
   * 1. finding an initial geometric path (ompl).
   * 2. generating a safe flight corridor (sfc) around the path.
   * 3. optimizing a polynomial trajectory within the sfc.
   *
   * @param planning_timeout     max time in seconds for the initial ompl
   * planner.
   * @param time_weight          penalty weight on total flight time in the
   * optimizer.
   * @param segment_length       desired length (meters) of each corridor
   * segment.
   * @param smoothing_epsilon    epsilon value for corridor smoothing algorithm.
   * @param integral_resolution  number of integration points per segment for
   * checking penalties.
   * @param magnitude_bounds     vector(5): [v_max, omega_max, theta_max,
   * thrust_min, thrust_max].
   * @param penalty_weights      vector(5): weights for [pos, vel, omega, theta,
   * thrust] penalties.
   * @param physical_params      vector(6): [mass, gravity, horiz_drag,
   * vert_drag, parasitic_drag, speed_smooth_factor].
   * @param out_traj             output reference to store the resulting
   * trajectory.
   * @return                     true if planning and optimization succeed, false
   * otherwise.
   */
  bool run_inference(double planning_timeout, double time_weight,
                     double segment_length, double smoothing_epsilon,
                     int integral_resolution,
                     const Eigen::VectorXd &magnitude_bounds,
                     const Eigen::VectorXd &penalty_weights,
                     const Eigen::VectorXd &physical_params,
                     Trajectory<5> &out_traj);

  /**
   * gets the complete drone kinematic state (pos, vel, acc, jerk) at a given
   * time.
   *
   * @param time    time along the trajectory, from 0 to total_duration.
   * @param state   output reference to store the drone's state.
   * @return        true if successful, false if no trajectory is computed or
   * time is out of bounds.
   */
  bool getStateAtTime(double time, DroneState &state) const;

  /**
   * computes the required control inputs (thrust, attitude) for a given state.
   *
   * @param time     time along the trajectory, from 0 to total_duration.
   * @param inputs   output reference to store the computed control inputs.
   * @param yaw      desired yaw angle [rad] (optional, default 0).
   * @param yaw_rate desired yaw rate [rad/s] (optional, default 0).
   * @return         true if successful, false if no trajectory is computed or
   * time is out of bounds.
   */
  bool getControlInputs(double time, ControlInputs &inputs, double yaw = 0.0,
                        double yaw_rate = 0.0) const;

  /**
   * retrieves performance metrics and statistics for the computed trajectory.
   *
   * @param stats   output reference to store trajectory statistics.
   * @return        true if successful, false if no trajectory has
   * been computed.
   */
  bool getStatistics(TrajectoryStatistics &stats) const;

  /**
   * @brief prints a 2d representation of the voxel map to the console.
   */
  void print_voxel_map() const;

  /**
   * retrieves all necessary data for visualizing the trajectory and map.
   *
   * @param trajectory_points  output vector to store 3d points of the
   * trajectory.
   * @param voxel_data         output 3d grid of voxel occupancy states (0=free,
   * 1=occ, 2=dilated).
   * @param voxel_size         output for the size of each voxel.
   * @param start_pos          output for the start position.
   * @param goal_pos           output for the goal position.
   * @param show_initial_route flag to also retrieve the un-optimized ompl path.
   * @param initial_route      output vector to store the initial path (if
   * requested).
   * @return                   true if successful, false if no trajectory has
   * been computed.
   */
  bool get_visualization_data(
      std::vector<Eigen::Vector3d> &trajectory_points,
      std::vector<std::vector<std::vector<int>>> &voxel_data,
      double &voxel_size, Eigen::Vector3d &start_pos, Eigen::Vector3d &goal_pos,
      bool show_initial_route = false,
      std::vector<Eigen::Vector3d> *initial_route = nullptr) const;

  /**
   * retrieves the initial, un-optimized path generated by ompl.
   *
   * @param route   output vector to store the initial route points.
   * @return        true if successful, false if no route has been computed.
   */
  bool get_initial_route(std::vector<Eigen::Vector3d> &route) const;

private:
  // pointer to the voxel map, which holds the 3d environment representation.
  std::unique_ptr<voxel_map::VoxelMap> map_;

  // trajectory start and goal definitions.
  Eigen::Vector3d start_position_;
  Eigen::Vector3d goal_position_;
  Eigen::Vector3d start_velocity_;
  Eigen::Vector3d goal_velocity_;
  bool endpoints_set_ = false; // flag to ensure endpoints are configured before use.

  // stores the final optimized trajectory.
  Trajectory<5> trajectory_;
  bool trajectory_computed_ = false; // flag to track if optimization was successful.
  double optimization_cost_ = 0.0;
  Eigen::VectorXd physical_params_; // vehicle physical params for control calculation.

  // flatness mapping utility to convert trajectory derivatives to control inputs.
  mutable flatness::FlatnessMap flatness_map_;

  // stores the initial, un-optimized path from ompl.
  std::vector<Eigen::Vector3d> initial_route_;
};