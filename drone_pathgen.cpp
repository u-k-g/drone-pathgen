#include <Eigen/Eigen>
#include <iostream>
#include "api/gcopter_api.hpp"

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
    std::cout << "Error: Must call configure_map() before set_endpoints()"
              << std::endl;
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