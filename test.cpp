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

    // 02. set endpoints for the path
    Eigen::Vector3d start(0.5, 0.5, 0.5);
    Eigen::Vector3d goal(8.0, 8.0, 2.0);
    gen.set_endpoints(start, goal);

    // 03. test run_inference with sample parameters
    double planning_timeout = 2.0;          // seconds
    double time_weight = 1.0;               // relative weight on flight time
    double segment_length = 1.0;            // meters per corridor segment
    double smoothing_epsilon = 0.1;         // smoothing parameter
    int integral_resolution = 10;           // integration points per segment
    // magnitude bounds: [v_max, omega_max, theta_max, thrust_min, thrust_max]
    Eigen::VectorXd magnitude_bounds(5);
    magnitude_bounds << 5.0, 2.0, 0.785, 0.0, 10.0;
    // penalty weights: [pos, vel, omega, theta, thrust]
    Eigen::VectorXd penalty_weights(5);
    penalty_weights << 1.0, 1.0, 1.0, 1.0, 1.0;
    // physical params: [mass, grav, horiz_drag, vert_drag, parasitic_drag, speed_smooth]
    Eigen::VectorXd physical_params(6);
    physical_params << 1.0, 9.81, 0.1, 0.1, 0.01, 0.1;
    // output trajectory
    Trajectory<5> traj;
    bool success = gen.run_inference(
        planning_timeout,
        time_weight,
        segment_length,
        smoothing_epsilon,
        integral_resolution,
        magnitude_bounds,
        penalty_weights,
        physical_params,
        traj
    );
    std::cout << "run_inference returned: " << (success ? "true" : "false") << std::endl;
    if (success) {
        std::cout << "Trajectory duration: " << traj.getTotalDuration() << std::endl;
        std::cout << "Number of pieces: " << traj.getPieceNum() << std::endl;
    }

    return 0;
}