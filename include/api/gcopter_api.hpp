#include "gcopter/flatness.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/sfc_gen.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/voxel_map.hpp"
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

private:
  std::unique_ptr<voxel_map::VoxelMap> map_;
  
  // Endpoint data for trajectory planning
  Eigen::Vector3d start_position_;
  Eigen::Vector3d goal_position_;
  Eigen::Vector3d start_velocity_;
  Eigen::Vector3d goal_velocity_;
  bool endpoints_set_ = false;  // Track if endpoints have been configured
};