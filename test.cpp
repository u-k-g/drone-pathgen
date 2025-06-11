#include "api/gcopter_api.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <vector>

int main() {
  GCopterAPI gen;
  Eigen::Vector3i map_size(20, 20, 10);
  Eigen::Vector3d origin(0, 0, 0);
  double scale = 1;

  std::vector<Eigen::Vector3d> obstacles = {
      {5.5, 5.5, 3.5}, {10.5, 8.5, 5.5}, {15.5, 15.5, 7.5}};

  // 01. configure the map with obstacles
  gen.configure_map(map_size, origin, scale, obstacles, 2);

  // 02. set endpoints INSIDE the obstacles (this should fail if map works
  // correctly)
  Eigen::Vector3d start(1.5, 1.5, 1.5);
  Eigen::Vector3d goal(18.5, 18.5, 8.5);
  gen.set_endpoints(start, goal);

  // 03. test run_inference with sample parameters
  double planning_timeout = 2.0;   // seconds
  double time_weight = 1.0;        // relative weight on flight time
  double segment_length = 0.8;     // smaller segments for tighter corridors
  double smoothing_epsilon = 0.08; // balanced smoothing parameter
  int integral_resolution = 10;    // integration points per segment
  // magnitude bounds: [v_max, omega_max, theta_max, thrust_min, thrust_max]
  Eigen::VectorXd magnitude_bounds(5);
  magnitude_bounds << 5.0, 2.0, 0.785, 0.0, 10.0;
  // penalty weights: [pos, vel, omega, theta, thrust]
  Eigen::VectorXd penalty_weights(5);
  penalty_weights << 10.0, 1.0, 1.0, 1.0, 1.0; // Much higher position penalty
  // physical params: [mass, grav, horiz_drag, vert_drag, parasitic_drag,
  // speed_smooth]
  Eigen::VectorXd physical_params(6);
  physical_params << 1.0, 9.81, 0.1, 0.1, 0.01, 0.1;
  // output trajectory
  Trajectory<5> traj;
  bool success = gen.run_inference(planning_timeout, time_weight,
                                   segment_length, smoothing_epsilon,
                                   integral_resolution, magnitude_bounds,
                                   penalty_weights, physical_params, traj);
  std::cout << "run_inference returned: " << (success ? "true" : "false")
            << std::endl;
  if (success) {
    std::cout << "Trajectory duration: " << traj.getTotalDuration()
              << std::endl;
    std::cout << "Number of pieces: " << traj.getPieceNum() << std::endl;

    // test  API functions

    // 04. test getStatistics
    TrajectoryStatistics stats;
    if (gen.getStatistics(stats)) {
      std::cout << "Statistics:" << std::endl;
      std::cout << "  Duration: " << stats.total_duration << "s" << std::endl;
      std::cout << "  Pieces: " << stats.num_pieces << std::endl;
      std::cout << "  Optimization cost: " << stats.optimization_cost
                << std::endl;
      std::cout << "  Max velocity: " << stats.max_velocity << " m/s"
                << std::endl;
      std::cout << "  Max acceleration: " << stats.max_acceleration << " m/s^2"
                << std::endl;
      std::cout << "  Start: " << stats.start_pos.transpose() << std::endl;
      std::cout << "  Goal: " << stats.goal_pos.transpose() << std::endl;
    }

    // 05. test getStateAtTime at several points
    std::cout << "\nDrone states at key times:" << std::endl;
    std::vector<double> test_times = {
        0.0, stats.total_duration * 0.25, stats.total_duration * 0.5,
        stats.total_duration * 0.75, stats.total_duration};

    for (double t : test_times) {
      DroneState state;
      if (gen.getStateAtTime(t, state)) {
        std::cout << "  t=" << t << "s: pos=" << state.position.transpose()
                  << ", vel=" << state.velocity.transpose()
                  << ", |vel|=" << state.velocity.norm() << " m/s" << std::endl;
      }
    }

    // 06. test getControlInputs at midpoint
    std::cout << "\nControl inputs at trajectory midpoint:" << std::endl;
    double mid_time = stats.total_duration * 0.5;
    ControlInputs controls;
    if (gen.getControlInputs(mid_time, controls)) {
      std::cout << "  Thrust: " << controls.thrust << " N" << std::endl;
      std::cout << "  Quaternion [w,x,y,z]: " << controls.quaternion.transpose()
                << std::endl;
      std::cout << "  Angular velocity: "
                << controls.angular_velocity.transpose() << " rad/s"
                << std::endl;
      std::cout << "  |Angular velocity|: " << controls.angular_velocity.norm()
                << " rad/s" << std::endl;
    }

    // print trajectory points for visualization
    std::cout << "\nTRAJECTORY" << std::endl;
    double dt = 0.1;
    for (double t = 0.0; t < traj.getTotalDuration(); t += dt) {
      Eigen::Vector3d pos = traj.getPos(t);
      std::cout << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
    }
    // also print the final point
    Eigen::Vector3d pos = traj.getPos(traj.getTotalDuration());
    std::cout << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
  }

  return 0;
}