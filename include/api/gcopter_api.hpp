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
  void set_endpoints();
  void run_inference();
  void get_visualization_data();

private:
  std::unique_ptr<voxel_map::VoxelMap> map_;
};