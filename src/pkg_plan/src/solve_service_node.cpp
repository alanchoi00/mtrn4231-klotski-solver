#include "pkg_plan/solve_service_node.hpp"

#include <chrono>

SolveServiceNode::SolveServiceNode() : Node("solve_service_node") {
  solver_ = std::make_shared<klotski::KlotskiSolver>();

  // Create the service
  service_ = this->create_service<klotski_interfaces::srv::SolveBoard>(
      "solve_board", std::bind(&SolveServiceNode::handleSolveBoard, this,
                               std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "SolveServiceNode has been started.");
  RCLCPP_INFO(this->get_logger(),
              "Service 'solve_board' is ready to receive requests.");
}

bool SolveServiceNode::validateBoardState(
    const klotski_interfaces::msg::BoardState& state) const {
  // Check if board has valid dimensions
  if (state.board.spec.cols != klotski::KlotskiSolver::W ||
      state.board.spec.rows != klotski::KlotskiSolver::H) {
    return false;
  }

  // Check if board has pieces
  if (state.board.pieces.empty()) {
    return false;
  }

  return true;
}

bool SolveServiceNode::validateBoard(
    const klotski_interfaces::msg::Board& board) const {
  // Check if board has valid dimensions
  if (board.spec.cols != klotski::KlotskiSolver::W ||
      board.spec.rows != klotski::KlotskiSolver::H) {
    return false;
  }

  // Check if board has pieces
  if (board.pieces.empty()) {
    return false;
  }

  return true;
}

void SolveServiceNode::handleSolveBoard(
    const std::shared_ptr<klotski_interfaces::srv::SolveBoard::Request> request,
    std::shared_ptr<klotski_interfaces::srv::SolveBoard::Response> response) {
  auto start_time = std::chrono::high_resolution_clock::now();

  RCLCPP_INFO(this->get_logger(), "Received solve board request");

  try {
    // Validate input
    if (!validateBoardState(request->state)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid board state provided");
      response->solved = false;
      response->moves = klotski_interfaces::msg::MoveList();
      return;
    }

    if (!validateBoard(request->goal)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid goal board provided");
      response->solved = false;
      response->moves = klotski_interfaces::msg::MoveList();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Solving board puzzle...");

    // Attempt to solve the board
    auto move_list = solver_->solveMoveList(request->state, request->goal);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);

    // Set response
    response->moves = move_list;
    response->solved = !move_list.moves.empty();

    if (response->solved) {
      RCLCPP_INFO(this->get_logger(),
                  "Board solved successfully with %zu moves in %ld ms",
                  move_list.moves.size(), duration.count());
    } else {
      RCLCPP_WARN(this->get_logger(), "No solution found after %ld ms",
                  duration.count());
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Exception occurred while solving board: %s", e.what());
    response->solved = false;
    response->moves = klotski_interfaces::msg::MoveList();
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(),
                 "Unknown exception occurred while solving board");
    response->solved = false;
    response->moves = klotski_interfaces::msg::MoveList();
  }
}
