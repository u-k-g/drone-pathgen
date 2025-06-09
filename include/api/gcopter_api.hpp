#include "gcopter/flatness.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/sfc_gen.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/gcopter.hpp"
#include <Eigen/Eigen>
#include <memory>
#include <string>
#include <vector>

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
  void set_endpoints(
      const Eigen::Vector3d& start_pos,
      const Eigen::Vector3d& goal_pos,
      const Eigen::Vector3d& start_vel = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d& goal_vel = Eigen::Vector3d::Zero()
  );

  /**
   * Plan and optimize a minimum-time trajectory through the configured map and endpoints.
   *
   * @param planning_timeout     Maximum time (seconds) for OMPL planning.
   * @param time_weight          Weight on total flight time in the optimizer.
   * @param segment_length       Nominal length (meters) per corridor segment.
   * @param smoothing_epsilon    Epsilon for safe corridor smoothing.
   * @param integral_resolution  Number of integration points per segment for penalty terms.
   * @param magnitude_bounds     Eigen::VectorXd(5): [v_max, omega_max, theta_max, thrust_min, thrust_max].
   * @param penalty_weights      Eigen::VectorXd(5): [pos_weight, vel_weight, omega_weight, theta_weight, thrust_weight].
   * @param physical_params      Eigen::VectorXd(6): [mass, grav_accel, horiz_drag, vert_drag, parasitic_drag, speed_smooth_factor].
   * @param out_traj             Output parameter to receive the optimized 5th-order trajectory.
   * @return                     True if planning and optimization succeeded, false otherwise.
   */
  bool run_inference(
      double planning_timeout,
      double time_weight,
      double segment_length,
      double smoothing_epsilon,
      int integral_resolution,
      const Eigen::VectorXd& magnitude_bounds,
      const Eigen::VectorXd& penalty_weights,
      const Eigen::VectorXd& physical_params,
      Trajectory<5>& out_traj
  );

private:
  std::unique_ptr<voxel_map::VoxelMap> map_;
  Eigen::Vector3d start_position_;
  Eigen::Vector3d goal_position_;
  Eigen::Vector3d start_velocity_;
  Eigen::Vector3d goal_velocity_;
  bool endpoints_set_ = false;  // Track if endpoints have been configured
};