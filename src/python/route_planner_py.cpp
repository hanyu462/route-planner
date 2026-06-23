// pybind11 bindings for RoutePlannerPipeline
//
// Python usage:
//   import route_planner_py as rp
//   import sys
//
//   rp.init(sys.argv)
//   pipeline = rp.RoutePlannerPipeline(
//       processor_config="config/pointcloud_processor.yaml",
//       grid_config="config/occupancy_grid_builder.yaml",
//       costmap_config="config/costmap_builder.yaml",
//       astar_config="config/astar.yaml",
//       pipeline_config="config/pipeline.yaml",
//   )
//   pipeline.start()
//   pipeline.set_goal(5.17, 0.7)
//   waypoints = pipeline.get_path()   # [(x, y), ...]
//   pipeline.stop()
//   rp.shutdown()

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <rclcpp/rclcpp.hpp>

#include "route_planner/pipeline/route_planner_pipeline.hpp"

namespace py = pybind11;
using namespace pybind11::literals;
using route_planner::pipeline::RoutePlannerPipeline;

static std::string pkg_config(const std::string& filename)
{
    return std::string(ROUTE_PLANNER_CONFIG_DIR) + "/" + filename;
}

PYBIND11_MODULE(route_planner_py, m)
{
    m.doc() = "route_planner Python bindings — ROS2 in, pybind11 out";

    // ── rclcpp lifecycle ──────────────────────────────────────────────────────
    m.def("init",
        [](std::vector<std::string> args) {
            std::vector<char*> argv;
            argv.reserve(args.size());
            for (auto& s : args) argv.push_back(s.data());
            int argc = static_cast<int>(argv.size());
            rclcpp::init(argc, argc > 0 ? argv.data() : nullptr);
        },
        py::arg("args") = std::vector<std::string>{},
        "Initialize rclcpp. Call once before creating RoutePlannerPipeline.");

    m.def("shutdown", []() { rclcpp::shutdown(); },
        "Shutdown rclcpp. Call after pipeline.stop().");

    m.def("ok", []() { return rclcpp::ok(); },
        "Return True if rclcpp context is valid.");

    // ── RoutePlannerPipeline ──────────────────────────────────────────────────
    py::class_<RoutePlannerPipeline>(m, "RoutePlannerPipeline")
        .def(py::init([](
            const std::string& processor_config,
            const std::string& grid_config,
            const std::string& costmap_config,
            const std::string& astar_config,
            const std::string& pipeline_config)
        {
            return std::make_unique<RoutePlannerPipeline>(RoutePlannerPipeline::Config{
                processor_config,
                grid_config,
                costmap_config,
                astar_config,
                pipeline_config,
            });
        }),
        py::arg("processor_config") = pkg_config("pointcloud_processor.yaml"),
        py::arg("grid_config")       = pkg_config("occupancy_grid_builder.yaml"),
        py::arg("costmap_config")    = pkg_config("costmap_builder.yaml"),
        py::arg("astar_config")      = pkg_config("astar.yaml"),
        py::arg("pipeline_config")   = pkg_config("pipeline.yaml"),
        "Create the pipeline. rp.init() must be called first.\n"
        "Config paths default to the installed package share directory.")

        .def("start", &RoutePlannerPipeline::start,
            "Start T1-T6 background threads.")

        .def("stop", &RoutePlannerPipeline::stop,
            "Stop all threads and join them.")

        .def("set_goal",
            &RoutePlannerPipeline::set_goal,
            py::arg("map_x"), py::arg("map_y"),
            "Set A* goal in map-frame coordinates. Thread-safe.")

        .def("get_path",
            [](const RoutePlannerPipeline& self) {
                std::vector<std::pair<float, float>> result;
                if (auto snap = self.path_buffer()->read()) {
                    result = snap->value->waypoints;
                }
                return result;  // list of (x, y) tuples; empty if no path yet
            },
            "Return latest path as list of (x, y) tuples in map frame. "
            "Returns [] if A* has not produced a path yet.")

        .def("thread_hz",
            [](const RoutePlannerPipeline& self) {
                const auto hz = self.thread_hz();
                return py::dict(
                    "t1_pointcloud2_spin"_a = hz.t1_pointcloud2_spin,
                    "t2_pose_spin"_a        = hz.t2_pose_spin,
                    "t3_processor"_a        = hz.t3_processor,
                    "t4_occupancy_grid"_a   = hz.t4_occupancy_grid,
                    "t5_costmap"_a          = hz.t5_costmap,
                    "t6_astar"_a            = hz.t6_astar
                );
            },
            "Return dict of measured Hz for each thread.");
}
