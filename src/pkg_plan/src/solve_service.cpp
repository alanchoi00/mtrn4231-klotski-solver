#include "pkg_plan/solve_service_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<SolveServiceNode>();

    RCLCPP_INFO(rclcpp::get_logger("main"),
                "Starting Klotski solve service...");
    rclcpp::spin(node);

  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
    return 1;
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Unknown exception in main");
    return 1;
  }

  RCLCPP_INFO(rclcpp::get_logger("main"),
              "Shutting down Klotski solve service");
  rclcpp::shutdown();
  return 0;
}
