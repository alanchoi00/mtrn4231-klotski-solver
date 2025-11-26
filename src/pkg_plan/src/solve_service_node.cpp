#include "pkg_plan/solve_service_node.hpp"

#include <chrono>

SolveServiceNode::SolveServiceNode() : Node("solve_service_node") {
  solver_ = std::make_shared<klotski::KlotskiSolver>();

  // Create the service
  service_ = this->create_service<klotski_interfaces::srv::SolveBoard>(
      "/plan/solve", std::bind(&SolveServiceNode::handleSolveBoard, this,
                               std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "SolveServiceNode has been started.");
  RCLCPP_INFO(this->get_logger(),
              "Service '/plan/solve' is ready to receive requests.");
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

std::string SolveServiceNode::boardToPatternString(
    const klotski_interfaces::msg::Board& board) const {
  const int W = board.spec.cols;
  const int H = board.spec.rows;

  // Create top-origin grid of digits (0 = empty)
  std::vector<std::vector<int>> grid(H, std::vector<int>(W, 0));

  // Fill grid with piece types at their cell positions
  for (const auto& piece : board.pieces) {
    for (const auto& cell : piece.cells) {
      int col = cell.col;
      int row = cell.row;

      // Convert bottom-left origin (ROS) to top-origin (pattern display)
      int top_row = H - 1 - row;
      int left_col = col;

      if (top_row >= 0 && top_row < H && left_col >= 0 && left_col < W) {
        grid[top_row][left_col] = piece.type;
      }
    }
  }

  // Flatten to string (row-major, top row first)
  std::string result;
  result.reserve(W * H);
  for (int r = 0; r < H; ++r) {
    for (int c = 0; c < W; ++c) {
      result += std::to_string(grid[r][c]);
    }
  }

  return result;
}

bool SolveServiceNode::boardsEqual(
    const klotski_interfaces::msg::Board& board1,
    const klotski_interfaces::msg::Board& board2) const {
  std::string pattern1 = boardToPatternString(board1);
  std::string pattern2 = boardToPatternString(board2);

  RCLCPP_INFO(this->get_logger(), "Comparing board patterns:");
  RCLCPP_INFO(this->get_logger(), "Pattern 1: %s", pattern1.c_str());
  RCLCPP_INFO(this->get_logger(), "Pattern 2: %s", pattern2.c_str());

  bool equal = (pattern1 == pattern2);
  RCLCPP_INFO(this->get_logger(), "Patterns match: %s",
              equal ? "true" : "false");

  return equal;
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
      response->plan = klotski_interfaces::msg::MoveList();
      return;
    }

    if (!validateBoard(request->goal)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid goal board provided");
      response->solved = false;
      response->plan = klotski_interfaces::msg::MoveList();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Solving board puzzle...");

    if (boardsEqual(request->state.board, request->goal)) {
      RCLCPP_INFO(this->get_logger(), "Initial state is the goal state");
      response->solved = true;
      response->plan = klotski_interfaces::msg::MoveList();
      return;
    }

    // Attempt to solve the board
    auto move_list = solver_->solveMoveList(request->state, request->goal);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);

    // Set response
    response->plan = move_list;
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
    response->plan = klotski_interfaces::msg::MoveList();
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(),
                 "Unknown exception occurred while solving board");
    response->solved = false;
    response->plan = klotski_interfaces::msg::MoveList();
  }
}
