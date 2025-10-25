// pkg_plan/include/pkg_plan/klotski_solver.hpp
#pragma once
#include <array>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "klotski_interfaces/msg/board.hpp"
#include "klotski_interfaces/msg/board_spec.hpp"
#include "klotski_interfaces/msg/board_state.hpp"
#include "klotski_interfaces/msg/cell.hpp"
#include "klotski_interfaces/msg/move.hpp"
#include "klotski_interfaces/msg/move_list.hpp"
#include "klotski_interfaces/msg/piece.hpp"

namespace klotski {

class KlotskiSolver {
 public:
  static constexpr int W = 4;
  static constexpr int H = 5;
  using Grid = std::array<uint8_t, W * H>;  // 0=empty, 1..4=types

  enum class Dir { Up, Down, Left, Right };

  struct UnitMove {
    // A single unit slide of one piece (identified by its cells-before-move).
    // No global ID; we carry the cells we moved from and the direction.
    std::vector<std::pair<int, int>> from_cells;  // (r,c) top-origin
    Dir dir;
  };

  // Public API
  std::vector<UnitMove> solve(const Grid& start, const Grid& goal) const;
  klotski_interfaces::msg::MoveList solveMoveList(
      const klotski_interfaces::msg::BoardState& state,
      const klotski_interfaces::msg::Board& goal) const;

  // Helpers to bridge ROS <-> internal
  static Grid boardToGrid(const klotski_interfaces::msg::Board& b);
  static Grid stateToGrid(const klotski_interfaces::msg::BoardState& s);
  static std::string debugString(const Grid& g);  // pretty print
  static const char* dirName(Dir dir);  // human-readable direction name

 private:
  struct ParentEdge {
    Grid prev;
    std::vector<std::pair<int, int>> from_cells;
    Dir dir;
  };

  static inline int idx(int r, int c) { return r * W + c; }
  static inline bool inB(int r, int c) {
    return r >= 0 && r < H && c >= 0 && c < W;
  }

  static void findPieces(const Grid& g,
                         std::vector<std::vector<std::pair<int, int>>>& pieces,
                         std::vector<uint8_t>& piece_type);

  static bool canSlide(const Grid& g,
                       const std::vector<std::pair<int, int>>& cells, int dr,
                       int dc);
  static Grid slide(const Grid& g,
                    const std::vector<std::pair<int, int>>& cells, int dr,
                    int dc);

  static std::vector<std::tuple<Grid, std::vector<std::pair<int, int>>, Dir>>
  neighbors(const Grid& g);

  static klotski_interfaces::msg::MoveList toMoveList(
      const std::vector<UnitMove>& unit, const Grid& start);
};

}  // namespace klotski
