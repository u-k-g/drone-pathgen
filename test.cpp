#include "api/gcopter_api.hpp"
#include <iostream>
#include <vector>
#include <Eigen/Eigen>

int main() {
    GCopterAPI gen;
    Eigen::Vector3i map_size(10, 10, 5);
    Eigen::Vector3d origin(0, 0, 0);
    double scale = 1;

    std::vector<Eigen::Vector3d> obstacles = {
        {2.0, 2.0, 1.0},
        {5.0, 5.0, 3.0}
    };

    // 01. configure the map
    gen.configure_map(map_size, origin, scale, obstacles, /*dilation=*/1);

    Eigen::Vector3d start(1.0, 1.0, 0.5);
    Eigen::Vector3d goal(8.0, 8.0, 2.0);
    // 02. set endpoints for the path
    gen.set_endpoints(start, goal);

    return 0;
}