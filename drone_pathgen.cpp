#include <Eigen/Eigen>
#include <iostream>
#include <ompl/geometric/SimpleSetup.h>

// GCOPTER includes
#include "gcopter/firi.hpp"
#include "api/gcopter_api.hpp"


class DronePathGenerator {
private:
  GCopterAPI gc_api_;
public:
  bool configureMap(const Eigen::Vector3i& map_size,
                    const Eigen::Vector3d& origin,
                    double voxel_scale,
                    const std::vector<Eigen::Vector3d>& obstacle_points,
                    int dilation_radius = 1) {
    std::cout << "Configuring map..." << std::endl;
    gc_api_.configure_map(map_size,
                          origin,
                          voxel_scale,
                          obstacle_points,
                          dilation_radius);
    return true;
  }

  // Set start and end points
  bool setEndpoints(const Eigen::Vector3d &start, const Eigen::Vector3d &end) {
    std::cout << "Setting endpoints: " << start.transpose() << " -> "
              << end.transpose() << std::endl;
    return true;
  }

  // Run trajectory optimization
  bool runInference() {
    std::cout << "Running trajectory optimization..." << std::endl;
    return true;
  }

  // Visualize results (could return trajectory data)
  bool visualizeResults() {
    std::cout << "Visualizing results..." << std::endl;
    return true;
  }
};

// Implement GCopterAPI methods
void GCopterAPI::configure_map(
    const Eigen::Vector3i& map_size,
    const Eigen::Vector3d& origin,
    double voxel_scale,
    const std::vector<Eigen::Vector3d>& obstacle_points,
    int dilation_radius) {
  // Allocate the VoxelMap
  map_ = std::make_unique<voxel_map::VoxelMap>(map_size, origin, voxel_scale);
  // Mark each obstacle
  for (const auto& pt : obstacle_points) {
    map_->setOccupied(pt);
  }
  // Apply safety dilation
  map_->dilate(dilation_radius);

  // Output labeled voxel grid (0=Unoccupied, 1=Occupied, 2=Dilated)
  const auto size = map_->getSize();
  const auto& voxels = map_->getVoxels();
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