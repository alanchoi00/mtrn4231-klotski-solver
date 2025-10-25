#pragma once

#include <memory>

#include "klotski_interfaces/srv/solve_board.hpp"
#include "pkg_plan/klotski_solver.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief ROS2 service node for solving Klotski puzzles
 *
 * This node provides a service that accepts BoardState and Board goal
 * and returns a MoveList containing the solution path.
 */
class SolveServiceNode : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Solve Service Node
   */
  SolveServiceNode();

 private:
  std::shared_ptr<klotski::KlotskiSolver> solver_;
  rclcpp::Service<klotski_interfaces::srv::SolveBoard>::SharedPtr service_;

  /**
   * @brief Validate board state input
   * @param state The board state to validate
   * @return true if valid, false otherwise
   */
  bool validateBoardState(
      const klotski_interfaces::msg::BoardState& state) const;

  /**
   * @brief Validate board input
   * @param board The board to validate
   * @return true if valid, false otherwise
   */
  bool validateBoard(const klotski_interfaces::msg::Board& board) const;

  /**
   * @brief Convert a Board message to a pattern string
   * @param board The board to convert
   * @return Pattern string representation
   */
  std::string boardToPatternString(
      const klotski_interfaces::msg::Board& board) const;

  /**
   * @brief Compare two boards for equality using pattern string comparison
   * @param board1 First board to compare
   * @param board2 Second board to compare
   * @return true if boards are equal, false otherwise
   */
  bool boardsEqual(const klotski_interfaces::msg::Board& board1,
                   const klotski_interfaces::msg::Board& board2) const;

  /**
   * @brief Handle solve board service requests
   * @param request The service request containing state and goal
   * @param response The service response containing moves and solved flag
   */
  void handleSolveBoard(
      const std::shared_ptr<klotski_interfaces::srv::SolveBoard::Request>
          request,
      std::shared_ptr<klotski_interfaces::srv::SolveBoard::Response> response);
};
