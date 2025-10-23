#pragma once
#include <algorithm>
#include <array>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "klotski_interfaces/msg/cell.hpp"
#include "klotski_interfaces/msg/move.hpp"
#include "klotski_interfaces/msg/move_list.hpp"
#include "klotski_interfaces/msg/piece.hpp"
#include "klotski_interfaces/msg/board_state.hpp"
#include "klotski_interfaces/msg/board.hpp"
#include "klotski_interfaces/msg/board_spec.hpp"

namespace klotski {

class KlotskiSolver {
 public:
  static constexpr int W = 4;
  static constexpr int H = 5;
  static constexpr int N = W * H;

  enum class Dir { Up, Down, Left, Right };

  struct Move {
    char piece;  // 'a'..'z'
    Dir dir;     // unit step direction
    int steps;   // 1 for unit steps; can be >1 after coalesce()
  };

  struct Step {
    std::string rep;  // next representation
    char piece;
    Dir dir;
  };

  /// Solve shortest path (unweighted BFS) from start->goal. Returns UNIT steps.
  /// start/goal are 20-char strings, row-major, TOP row first.
  std::vector<Move> solve(const std::string& start,
                          const std::string& goal) const;

  /// Solve using BoardState and Board
  klotski_interfaces::msg::MoveList solveMoveList(
      const klotski_interfaces::msg::BoardState& state,
      const klotski_interfaces::msg::Board& goal) const;

  /// Coalesce consecutive identical (piece, dir) into single entries
  /// (cosmetic).
  static std::vector<Move> coalesce(const std::vector<Move>& unitMoves) {
    if (unitMoves.empty()) return {};
    std::vector<Move> out;
    out.reserve(unitMoves.size());
    Move cur = unitMoves.front();
    for (size_t i = 1; i < unitMoves.size(); ++i) {
      const auto& m = unitMoves[i];
      if (m.piece == cur.piece && m.dir == cur.dir) {
        cur.steps += 1;
      } else {
        out.push_back(cur);
        cur = m;
      }
    }
    out.push_back(cur);
    return out;
  }

  /// Utility: apply a single unit step to a board (throws if illegal).
  static std::string applyStep(const std::string& rep, char piece, Dir dir);

  /// Human-readable direction name.
  static const char* dirName(Dir d) {
    switch (d) {
      case Dir::Up:
        return "up";
      case Dir::Down:
        return "down";
      case Dir::Left:
        return "left";
      case Dir::Right:
        return "right";
    }
    return "?";
  }

  static int rowOf(int idx) { return idx / W; }  // top-origin
  static int colOf(int idx) { return idx % W; }
  static int idxOf(int r, int c) { return r * W + c; }

  /// Convert from top-origin row to bottom-left origin row
  static int toBottomLeftRow(int topRow) { return H - 1 - topRow; }

  /// Convert Board message to internal string representation
  static std::string boardToString(const klotski_interfaces::msg::Board& board);

 private:
  static void assertRep(const std::string& rep);
  static bool inBounds(int r, int c);
  static std::vector<std::pair<int, int>> pieceCells(const std::string& rep,
                                                     char ch);
  static bool canSlide(const std::string& rep, char ch, int dr, int dc);
  static std::string slide(const std::string& rep, char ch, int dr, int dc);
  static std::vector<Step> neighbors(const std::string& rep);

  /// Convert internal Move format to ROS MoveList format
  klotski_interfaces::msg::MoveList convertToMoveList(
      const std::vector<Move>& moves, const std::string& startRep) const;
};

}  // namespace klotski
