#include "api/gcopter_api.hpp"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(gcopter_cpp, m) {
  m.doc() = "pybind11 bindings for GCOPTER C++ API (non-ROS)";

  // expose simple structs first
  py::class_<DroneState>(m, "DroneState")
      .def(py::init<>())
      .def_readonly("time", &DroneState::time)
      .def_readonly("position", &DroneState::position)
      .def_readonly("velocity", &DroneState::velocity)
      .def_readonly("acceleration", &DroneState::acceleration)
      .def_readonly("jerk", &DroneState::jerk);

  py::class_<ControlInputs>(m, "ControlInputs")
      .def(py::init<>())
      .def_readonly("thrust", &ControlInputs::thrust)
      .def_readonly("quaternion", &ControlInputs::quaternion)
      .def_readonly("angular_velocity", &ControlInputs::angular_velocity)
      .def_readonly("yaw_angle", &ControlInputs::yaw_angle)
      .def_readonly("yaw_rate", &ControlInputs::yaw_rate);

  py::class_<TrajectoryStatistics>(m, "TrajectoryStatistics")
      .def(py::init<>())
      .def_readonly("total_duration", &TrajectoryStatistics::total_duration)
      .def_readonly("num_pieces", &TrajectoryStatistics::num_pieces)
      .def_readonly("optimization_cost",
                    &TrajectoryStatistics::optimization_cost)
      .def_readonly("max_velocity", &TrajectoryStatistics::max_velocity)
      .def_readonly("max_acceleration", &TrajectoryStatistics::max_acceleration)
      .def_readonly("start_pos", &TrajectoryStatistics::start_pos)
      .def_readonly("goal_pos", &TrajectoryStatistics::goal_pos);

  // expose GCopterAPI class
  py::class_<GCopterAPI>(m, "GCopterAPI")
      .def(py::init<>())
      .def("configure_map", &GCopterAPI::configure_map, py::arg("map_size"),
           py::arg("origin"), py::arg("voxel_scale"),
           py::arg("obstacle_points"), py::arg("dilation_radius") = 1,
           "configure voxel map with obstacles and dilation radius")
      .def("set_endpoints", &GCopterAPI::set_endpoints, py::arg("start_pos"),
           py::arg("goal_pos"), py::arg("start_vel") = Eigen::Vector3d::Zero(),
           py::arg("goal_vel") = Eigen::Vector3d::Zero(),
           "set start and goal positions and optional velocities")
      .def(
          "run_inference",
          [](GCopterAPI &self, double planning_timeout, double time_weight,
             double segment_length, double smoothing_epsilon,
             int integral_resolution, const Eigen::VectorXd &magnitude_bounds,
             const Eigen::VectorXd &penalty_weights,
             const Eigen::VectorXd &physical_params) {
            Trajectory<5> traj;
            bool ok = self.run_inference(
                planning_timeout, time_weight, segment_length,
                smoothing_epsilon, integral_resolution, magnitude_bounds,
                penalty_weights, physical_params, traj);
            return ok;
          },
          py::arg("planning_timeout"), py::arg("time_weight"),
          py::arg("segment_length"), py::arg("smoothing_epsilon"),
          py::arg("integral_resolution"), py::arg("magnitude_bounds"),
          py::arg("penalty_weights"), py::arg("physical_params"),
          "plan path and optimize trajectory; returns true on success")
      .def("get_state_at_time", &GCopterAPI::getStateAtTime, py::arg("time"),
           py::arg("state"), "retrieve drone kinematic state at time")
      .def("get_control_inputs", &GCopterAPI::getControlInputs, py::arg("time"),
           py::arg("inputs"), py::arg("yaw") = 0.0, py::arg("yaw_rate") = 0.0,
           "compute thrust and attitude commands at time")
      .def("get_statistics", &GCopterAPI::getStatistics, py::arg("stats"),
           "retrieve trajectory statistics")
      .def("print_voxel_map", &GCopterAPI::print_voxel_map,
           "print voxel occupancy grid to stdout")
      .def("get_initial_route", &GCopterAPI::get_initial_route,
           py::arg("route"), "get the initial OMPL path before optimization")
      .def(
          "get_visualization_data",
          [](const GCopterAPI &self, bool show_initial_route = false) {
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
            } else {
              return py::make_tuple(success, trajectory_points, voxel_data,
                                    voxel_size, start_pos, goal_pos);
            }
          },
          py::arg("show_initial_route") = false,
          "extract trajectory and voxel data for open3d visualization. "
          "returns a tuple: (success, trajectory_points, voxel_data, "
          "voxel_size, start_pos, goal_pos[, initial_route])");
}