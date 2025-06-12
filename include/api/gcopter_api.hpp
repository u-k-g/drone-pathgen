#include "gcopter/flatness.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/sfc_gen.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/voxel_map.hpp"
#include <Eigen/Eigen>
#include <memory>
#include <string>
#include <vector>

// structure to hold drone state at any time
struct DroneState {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  Eigen::Vector3d jerk;
  double time;
};

// structure to hold control inputs for drone
struct ControlInputs {
  double thrust;                    // thrust command [N]
  Eigen::Vector4d quaternion;       // attitude quaternion [w,x,y,z]
  Eigen::Vector3d angular_velocity; // body angular velocity [rad/s]
  double yaw_angle;                 // yaw angle [rad]
  double yaw_rate;                  // yaw rate [rad/s]
};

// structure to hold trajectory statistics
struct TrajectoryStatistics {
  double total_duration;     // total trajectory time [s]
  int num_pieces;            // number of polynomial pieces
  double optimization_cost;  // final optimization cost
  double max_velocity;       // maximum velocity magnitude [m/s]
  double max_acceleration;   // maximum acceleration magnitude [m/s^2]
  Eigen::Vector3d start_pos; // start position
  Eigen::Vector3d goal_pos;  // goal position
};

class GCopterAPI {
public:
  /**
   * Build a VoxelMap from raw obstacle data and apply safety dilation.
   *
   * @param map_size         Number of voxels along x, y, z.
   * @param origin           World-coordinate of the grid's (0,0,0) corner.
   * @param voxel_scale      Edge length of each voxel (in meters).
   * @param obstacle_points  List of obstacle positions in world coords.
   * @param dilation_radius  Number of voxels to dilate occupied cells.
   */
  void configure_map(const Eigen::Vector3i &map_size,
                     const Eigen::Vector3d &origin, double voxel_scale,
                     const std::vector<Eigen::Vector3d> &obstacle_points,
                     int dilation_radius = 1);
  /**
   * Set start and goal positions.
   *
   * @param start_pos    Starting position in world coordinates
   * @param goal_pos     Goal position in world coordinates
   * @param start_vel    Initial velocity (optional, defaults to zero)
   * @param goal_vel     Final velocity (optional, defaults to zero)
   */
  void set_endpoints(const Eigen::Vector3d &start_pos,
                     const Eigen::Vector3d &goal_pos,
                     const Eigen::Vector3d &start_vel = Eigen::Vector3d::Zero(),
                     const Eigen::Vector3d &goal_vel = Eigen::Vector3d::Zero());

  /**
   * Plan and optimize a minimum-time trajectory through the configured map and
   * endpoints.
   *
   * @param planning_timeout     Maximum time (seconds) for OMPL planning.
   * @param time_weight          Weight on total flight time in the optimizer.
   * @param segment_length       Nominal length (meters) per corridor segment.
   * @param smoothing_epsilon    Epsilon for safe corridor smoothing.
   * @param integral_resolution  Number of integration points per segment for
   * penalty terms.
   * @param magnitude_bounds     Eigen::VectorXd(5): [v_max, omega_max,
   * theta_max, thrust_min, thrust_max].
   * @param penalty_weights      Eigen::VectorXd(5): [pos_weight, vel_weight,
   * omega_weight, theta_weight, thrust_weight].
   * @param physical_params      Eigen::VectorXd(6): [mass, grav_accel,
   * horiz_drag, vert_drag, parasitic_drag, speed_smooth_factor].
   * @param out_traj             Output parameter to receive the optimized
   * 5th-order trajectory.
   * @return                     True if planning and optimization succeeded,
   * false otherwise.
   */
  bool run_inference(double planning_timeout, double time_weight,
                     double segment_length, double smoothing_epsilon,
                     int integral_resolution,
                     const Eigen::VectorXd &magnitude_bounds,
                     const Eigen::VectorXd &penalty_weights,
                     const Eigen::VectorXd &physical_params,
                     Trajectory<5> &out_traj);

  /**
   * Get complete drone state (position, velocity, acceleration, jerk) at a
   * specific time.
   *
   * @param time    Time along trajectory [0, total_duration]
   * @param state   Output parameter to receive drone state
   * @return        True if successful, false if no trajectory computed or time
   * out of bounds
   */
  bool getStateAtTime(double time, DroneState &state) const;

  /**
   * Get control inputs (thrust, attitude, angular velocity) at a specific time.
   *
   * @param time     Time along trajectory [0, total_duration]
   * @param inputs   Output parameter to receive control inputs
   * @param yaw      Desired yaw angle [rad] (optional, defaults to 0)
   * @param yaw_rate Desired yaw rate [rad/s] (optional, defaults to 0)
   * @return         True if successful, false if no trajectory computed or time
   * out of bounds
   */
  bool getControlInputs(double time, ControlInputs &inputs, double yaw = 0.0,
                        double yaw_rate = 0.0) const;

  /**
   * Get trajectory performance statistics and metrics.
   *
   * @param stats   Output parameter to receive trajectory statistics
   * @return        True if successful, false if no trajectory computed
   */
  bool getStatistics(TrajectoryStatistics &stats) const;

  /**
   * @brief Prints the voxel map to the console for visualization.
   */
  void print_voxel_map() const;

  /**
   * Get trajectory data for Open3D visualization.
   * 
   * @param trajectory_points  Output parameter to receive trajectory positions
   * @param voxel_data        Output parameter to receive voxel occupancy grid
   * @param voxel_size        Voxel size for scaling 
   * @param start_pos         Output parameter for start position
   * @param goal_pos          Output parameter for goal position
   * @param show_initial_route Flag to include initial OMPL route in output
   * @param initial_route     Output parameter for initial route (if requested)
   * @return                  True if successful, false if no trajectory computed
   */
  bool get_visualization_data(std::vector<Eigen::Vector3d> &trajectory_points,
                              std::vector<std::vector<std::vector<int>>> &voxel_data,
                              double &voxel_size,
                              Eigen::Vector3d &start_pos,
                              Eigen::Vector3d &goal_pos,
                              bool show_initial_route = false,
                              std::vector<Eigen::Vector3d> *initial_route = nullptr) const;

  /**
   * Get the initial route.
   *
   * @param route   Output parameter to receive the initial route
   * @return        True if successful, false if no route computed
   */
  bool get_initial_route(std::vector<Eigen::Vector3d> &route) const;

private:
  std::unique_ptr<voxel_map::VoxelMap> map_;
  Eigen::Vector3d start_position_;
  Eigen::Vector3d goal_position_;
  Eigen::Vector3d start_velocity_;
  Eigen::Vector3d goal_velocity_;
  bool endpoints_set_ = false; // track if endpoints have been configured

  // trajectory and optimization results
  Trajectory<5> trajectory_;
  bool trajectory_computed_ = false;
  double optimization_cost_ = 0.0;
  Eigen::VectorXd physical_params_; // store for flatness mapping

  // flatness mapper for control computation
  mutable flatness::FlatnessMap flatness_map_;

  // initial route
  std::vector<Eigen::Vector3d> initial_route_;
};