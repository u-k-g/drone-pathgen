#include "api/gcopter_api.hpp"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(gcopter_cpp, m) {
  // module docstring
  m.doc() = "pybind11 bindings for the GCOPTER C++ API, providing a "
            "non-ROS interface for drone trajectory planning.";

  // expose the DroneState struct to python
  py::class_<DroneState>(m, "DroneState",
                       "holds the complete kinematic state of the drone.")
      .def(py::init<>())
      .def_readonly("time", &DroneState::time, "time from trajectory start [s]")
      .def_readonly("position", &DroneState::position, "position vector [m]")
      .def_readonly("velocity", &DroneState::velocity, "velocity vector [m/s]")
      .def_readonly("acceleration", &DroneState::acceleration,
                    "acceleration vector [m/s^2]")
      .def_readonly("jerk", &DroneState::jerk, "jerk vector [m/s^3]");

  // expose the ControlInputs struct to python
  py::class_<ControlInputs>(
      m, "ControlInputs",
      "holds the computed control commands (thrust, attitude) for the drone.")
      .def(py::init<>())
      .def_readonly("thrust", &ControlInputs::thrust, "thrust command [n]")
      .def_readonly("quaternion", &ControlInputs::quaternion,
                    "attitude quaternion [w, x, y, z]")
      .def_readonly("angular_velocity", &ControlInputs::angular_velocity,
                    "body angular velocity [rad/s]")
      .def_readonly("yaw_angle", &ControlInputs::yaw_angle, "yaw angle [rad]")
      .def_readonly("yaw_rate", &ControlInputs::yaw_rate, "yaw rate [rad/s]");

  // expose the TrajectoryStatistics struct to python
  py::class_<TrajectoryStatistics>(
      m, "TrajectoryStatistics",
      "holds performance and summary metrics for a computed trajectory.")
      .def(py::init<>())
      .def_readonly("total_duration", &TrajectoryStatistics::total_duration,
                    "total trajectory time [s]")
      .def_readonly("num_pieces", &TrajectoryStatistics::num_pieces,
                    "number of polynomial pieces")
      .def_readonly("optimization_cost",
                    &TrajectoryStatistics::optimization_cost,
                    "final optimization cost")
      .def_readonly("max_velocity", &TrajectoryStatistics::max_velocity,
                    "maximum velocity magnitude [m/s]")
      .def_readonly("max_acceleration",
                    &TrajectoryStatistics::max_acceleration,
                    "maximum acceleration magnitude [m/s^2]")
      .def_readonly("start_pos", &TrajectoryStatistics::start_pos,
                    "start position [m]")
      .def_readonly("goal_pos", &TrajectoryStatistics::goal_pos,
                    "goal position [m]");

  // expose the main GCopterAPI class and its methods
  py::class_<GCopterAPI>(m, "GCopterAPI", "The main API for GCOPTER.")
      .def(py::init<>())
      .def("configure_map", &GCopterAPI::configure_map, py::arg("map_size"),
           py::arg("origin"), py::arg("voxel_scale"),
           py::arg("obstacle_points"), py::arg("dilation_radius") = 1,
           "configures the 3d voxel map with obstacles.")
      .def("set_endpoints", &GCopterAPI::set_endpoints, py::arg("start_pos"),
           py::arg("goal_pos"), py::arg("start_vel") = Eigen::Vector3d::Zero(),
           py::arg("goal_vel") = Eigen::Vector3d::Zero(),
           "sets the start and goal positions and optional velocities.")
      .def(
          "run_inference",
          // this lambda is a workaround because the c++ version uses a non-const
          // reference for the output trajectory, which pybind11 doesn't automatically
          // handle as a return value.
          [](GCopterAPI &self, double planning_timeout, double time_weight,
             double segment_length, double smoothing_epsilon,
             int integral_resolution, const Eigen::VectorXd &magnitude_bounds,
             const Eigen::VectorXd &penalty_weights,
             const Eigen::VectorXd &physical_params) {
            Trajectory<5> traj; // create a trajectory object to be filled
            bool success = self.run_inference(
                planning_timeout, time_weight, segment_length,
                smoothing_epsilon, integral_resolution, magnitude_bounds,
                penalty_weights, physical_params, traj);
            return success;
          },
          py::arg("planning_timeout"), py::arg("time_weight"),
          py::arg("segment_length"), py::arg("smoothing_epsilon"),
          py::arg("integral_resolution"), py::arg("magnitude_bounds"),
          py::arg("penalty_weights"), py::arg("physical_params"),
          "plans and optimizes the trajectory, returning true on success.")
      .def("get_state_at_time", &GCopterAPI::getStateAtTime, py::arg("time"),
           py::arg("state"), "retrieves drone kinematic state at a given time.")
      .def("get_control_inputs", &GCopterAPI::getControlInputs, py::arg("time"),
           py::arg("inputs"), py::arg("yaw") = 0.0, py::arg("yaw_rate") = 0.0,
           "computes thrust and attitude commands for a given state.")
      .def("get_statistics", &GCopterAPI::getStatistics, py::arg("stats"),
           "retrieves trajectory performance statistics.")
      .def("print_voxel_map", &GCopterAPI::print_voxel_map,
           "prints a 2d projection of the voxel map to the console.")
      .def(
          "get_visualization_data",
          // this lambda handles the optional 'initial_route' output, returning
          // a variable-length tuple depending on the 'show_initial_route' flag.
          [](const GCopterAPI &self, bool show_initial_route) {
            std::vector<Eigen::Vector3d> trajectory_points;
            std::vector<std::vector<std::vector<int>>> voxel_data;
            double voxel_size;
            Eigen::Vector3d start_pos;
            Eigen::Vector3d goal_pos;
            std::vector<Eigen::Vector3d> initial_route;

            bool success = self.get_visualization_data(
                trajectory_points, voxel_data, voxel_size, start_pos, goal_pos,
                show_initial_route,
                show_initial_route ? &initial_route : nullptr);

            if (show_initial_route) {
              return py::make_tuple(success, trajectory_points, voxel_data,
                                    voxel_size, start_pos, goal_pos,
                                    initial_route);
            }
            // if not showing initial route, return a tuple without it.
            // using py::cast to ensure proper type conversion for the python side.
            return py::make_tuple(success, py::cast(trajectory_points),
                                  py::cast(voxel_data), voxel_size,
                                  py::cast(start_pos), py::cast(goal_pos));
          },
          py::arg("show_initial_route") = false,
          "extracts all data needed for visualization.\n"
          "returns a tuple: (success, trajectory_points, voxel_data, "
          "voxel_size, start_pos, goal_pos[, initial_route])");
}