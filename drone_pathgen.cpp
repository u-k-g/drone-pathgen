#include <Eigen/Eigen>
#include <iostream>
#include <ompl/geometric/SimpleSetup.h>

// GCOPTER includes
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/trajectory.hpp"

class DronePathGenerator {
private:
  // GCOPTER-related members will be added here

public:
  // Configure the 3D map/environment
  bool configureMap(const std::string &mapData) {
    std::cout << "Configuring map..." << std::endl;
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

void say_hello() {
  std::cout << "Hello, from drone_pathgen!\n";

  // Test the basic API
  DronePathGenerator generator;
  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d end(10, 10, 5);

  generator.configureMap("test_map");
  generator.setEndpoints(start, end);
  generator.runInference();
  generator.visualizeResults();
}
