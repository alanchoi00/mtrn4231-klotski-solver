#include "pkg_plan/klotski_solver.hpp"

#include <algorithm>
#include <deque>
#include <unordered_map>

namespace klotski {

static uint8_t cellType(const klotski_interfaces::msg::Piece& p) {
  return p.type;  // already 1..4 per your enum
}

static int toTopRow(int bottom_row) {
  return KlotskiSolver::H - 1 - bottom_row;
}
static int toBottomRow(int top_row) { return KlotskiSolver::H - 1 - top_row; }

KlotskiSolver::Grid KlotskiSolver::boardToGrid(
    const klotski_interfaces::msg::Board& b) {
  Grid g{};
  g.fill(0);
  for (const auto& piece : b.pieces) {
    const uint8_t t = cellType(piece);
    for (const auto& c : piece.cells) {
      const int r = toTopRow(c.row);
      const int cc = static_cast<int>(c.col);
      if (!inB(r, cc)) continue;
      g[idx(r, cc)] = t;
    }
  }
  return g;
}

KlotskiSolver::Grid KlotskiSolver::stateToGrid(
    const klotski_interfaces::msg::BoardState& s) {
  return boardToGrid(s.board);
}

std::string KlotskiSolver::debugString(const Grid& g) {
  std::string out;
  for (int r = 0; r < H; ++r) {
    for (int c = 0; c < W; ++c) {
      uint8_t t = g[idx(r, c)];
      out.push_back(t == 0 ? '.' : char('0' + t));
    }
    out.push_back('\n');
  }
  return out;
}

// Replace the flood-fill version with this type-aware extractor.
void KlotskiSolver::findPieces(
    const Grid& g, std::vector<std::vector<std::pair<int, int>>>& pieces,
    std::vector<uint8_t>& piece_type) {
  using PieceMsg = klotski_interfaces::msg::Piece;

  std::array<bool, W * H> vis{};
  vis.fill(false);

  auto at = [&](int r, int c) -> uint8_t { return g[idx(r, c)]; };
  auto mark = [&](int r, int c) { vis[idx(r, c)] = true; };
  auto used = [&](int r, int c) -> bool { return vis[idx(r, c)]; };

  for (int r = 0; r < H; ++r) {
    for (int c = 0; c < W; ++c) {
      if (used(r, c)) continue;
      uint8_t T = at(r, c);
      if (T == 0) continue;

      std::vector<std::pair<int, int>> cells;

      switch (T) {
        case PieceMsg::TYPE_1_1: {
          // exactly one cell
          cells = {{r, c}};
          break;
        }
        case PieceMsg::TYPE_1_2: {
          // must be (r,c) and (r,c+1), and both not already used
          if (!(inB(r, c + 1) && at(r, c + 1) == T && !used(r, c + 1))) {
            throw std::runtime_error(
                "Invalid board: 1x2 expected but not found at anchor");
          }
          cells = {{r, c}, {r, c + 1}};
          break;
        }
        case PieceMsg::TYPE_2_1: {
          // must be (r,c) and (r+1,c)
          if (!(inB(r + 1, c) && at(r + 1, c) == T && !used(r + 1, c))) {
            throw std::runtime_error(
                "Invalid board: 2x1 expected but not found at anchor");
          }
          cells = {{r, c}, {r + 1, c}};
          break;
        }
        case PieceMsg::TYPE_2_2: {
          // must be the 2x2 block at (r,c)
          if (!(inB(r, c + 1) && inB(r + 1, c) && inB(r + 1, c + 1) &&
                at(r, c + 1) == T && at(r + 1, c) == T &&
                at(r + 1, c + 1) == T && !used(r, c + 1) && !used(r + 1, c) &&
                !used(r + 1, c + 1))) {
            throw std::runtime_error(
                "Invalid board: 2x2 expected but not found at anchor");
          }
          cells = {{r, c}, {r, c + 1}, {r + 1, c}, {r + 1, c + 1}};
          break;
        }
        default:
          throw std::runtime_error("Unknown piece type in grid");
      }

      // Ensure none of these cells were already consumed
      for (auto [rr, cc] : cells) {
        if (used(rr, cc))
          throw std::runtime_error("Overlapping pieces in grid");
      }
      for (auto [rr, cc] : cells) mark(rr, cc);

      pieces.push_back(std::move(cells));
      piece_type.push_back(T);
    }
  }
}

// Can translate a component by (dr,dc) if all new cells in-bounds and either
// empty or within the piece.
bool KlotskiSolver::canSlide(const Grid& g,
                             const std::vector<std::pair<int, int>>& cells,
                             int dr, int dc) {
  // mark cells for quick self-collision allowance
  std::array<bool, W * H> isSelf{};
  isSelf.fill(false);
  for (auto [r, c] : cells) isSelf[idx(r, c)] = true;

  for (auto [r, c] : cells) {
    int nr = r + dr, nc = c + dc;
    if (!inB(nr, nc)) return false;
    int J = idx(nr, nc);
    if (g[J] != 0 && !isSelf[J]) return false;
  }
  return true;
}

KlotskiSolver::Grid KlotskiSolver::slide(
    const Grid& g, const std::vector<std::pair<int, int>>& cells, int dr,
    int dc) {
  Grid out = g;
  // clear old
  uint8_t T = 0;
  for (auto [r, c] : cells) {
    T = g[idx(r, c)];
    out[idx(r, c)] = 0;
  }
  // place new
  for (auto [r, c] : cells) {
    int nr = r + dr, nc = c + dc;
    out[idx(nr, nc)] = T;
  }
  return out;
}

std::vector<std::tuple<KlotskiSolver::Grid, std::vector<std::pair<int, int>>,
                       KlotskiSolver::Dir>>
KlotskiSolver::neighbors(const Grid& g) {
  std::vector<std::tuple<Grid, std::vector<std::pair<int, int>>, Dir>> out;
  std::vector<std::vector<std::pair<int, int>>> pieces;
  std::vector<uint8_t> types;
  findPieces(g, pieces, types);

  auto emit = [&](const std::vector<std::pair<int, int>>& pc, int dr, int dc,
                  Dir d) {
    if (canSlide(g, pc, dr, dc)) {
      out.emplace_back(slide(g, pc, dr, dc), pc, d);
    }
  };

  for (auto& pc : pieces) {
    emit(pc, -1, 0, Dir::Up);
    emit(pc, 1, 0, Dir::Down);
    emit(pc, 0, -1, Dir::Left);
    emit(pc, 0, 1, Dir::Right);
  }

  // optional dedup by next-grid
  // (rare but harmless; keep grids as keys if needed)
  return out;
}

// Unweighted shortest path by BFS on grids
std::vector<KlotskiSolver::UnitMove> KlotskiSolver::solve(
    const Grid& start, const Grid& goal) const {
  if (start == goal) return {};
  auto hasher = [](const Grid& g) {
    // cheap hash of 20 bytes
    size_t h = 1469598103934665603ull;
    for (uint8_t v : g) {
      h ^= v;
      h *= 1099511628211ull;
    }
    return h;
  };
  struct GridEq {
    bool operator()(const Grid& a, const Grid& b) const { return a == b; }
  };

  std::unordered_map<Grid, ParentEdge, decltype(hasher), GridEq> parent(
      0, hasher, GridEq{});
  std::queue<Grid> q;

  parent.emplace(start, ParentEdge{Grid{}, {}, Dir::Up});
  q.push(start);

  while (!q.empty()) {
    Grid cur = q.front();
    q.pop();
    for (auto& nb : neighbors(cur)) {
      const Grid& nxt = std::get<0>(nb);
      if (parent.find(nxt) != parent.end()) continue;
      parent.emplace(nxt, ParentEdge{cur, std::get<1>(nb), std::get<2>(nb)});
      if (nxt == goal) {
        // reconstruct unit-step moves
        std::vector<UnitMove> seq;
        Grid t = nxt;
        while (true) {
          auto it = parent.find(t);
          if (it == parent.end()) break;
          const auto& P = it->second;
          if (P.prev == Grid{}) break;  // start sentinel
          seq.push_back(UnitMove{P.from_cells, P.dir});
          t = P.prev;
        }
        std::reverse(seq.begin(), seq.end());
        return seq;
      }
      q.push(nxt);
    }
  }
  return {};  // unreachable
}

klotski_interfaces::msg::MoveList KlotskiSolver::toMoveList(
    const std::vector<UnitMove>& unit, const Grid& start) {
  using MoveMsg = klotski_interfaces::msg::Move;
  using CellMsg = klotski_interfaces::msg::Cell;

  auto gridAfter = start;
  klotski_interfaces::msg::MoveList out;

  auto bottomRow = [](int top) { return toBottomRow(top); };

  for (const auto& u : unit) {
    // capture current piece cells and type
    uint8_t T = 0;
    for (auto [r, c] : u.from_cells) {
      T = gridAfter[idx(r, c)];
    }
    // compute next grid and next cells
    int dr = 0, dc = 0;
    if (u.dir == Dir::Up) dr = -1;
    if (u.dir == Dir::Down) dr = +1;
    if (u.dir == Dir::Left) dc = -1;
    if (u.dir == Dir::Right) dc = +1;

    auto next = gridAfter;  // copy
    for (auto [r, c] : u.from_cells) next[idx(r, c)] = 0;
    std::vector<std::pair<int, int>> to_cells;
    to_cells.reserve(u.from_cells.size());
    for (auto [r, c] : u.from_cells) {
      int nr = r + dr, nc = c + dc;
      next[idx(nr, nc)] = T;
      to_cells.push_back({nr, nc});
    }

    // Build Move message
    MoveMsg m;
    m.piece.type = T;   // set geometric type
    m.piece.color = 0;  // unknown/not used here
    // from-cells (in bottom-left origin)
    m.piece.cells.reserve(u.from_cells.size());
    for (auto [r, c] : u.from_cells) {
      CellMsg cl;
      cl.col = c;
      cl.row = bottomRow(r);
      m.piece.cells.push_back(cl);
    }
    // to_cell: pick the min (r,c) after move (or any consistent convention)
    auto [r0, c0] = *std::min_element(to_cells.begin(), to_cells.end());
    m.to_cell.col = c0;
    m.to_cell.row = bottomRow(r0);

    out.moves.push_back(std::move(m));
    gridAfter = std::move(next);
  }
  return out;
}

klotski_interfaces::msg::MoveList KlotskiSolver::solveMoveList(
    const klotski_interfaces::msg::BoardState& state,
    const klotski_interfaces::msg::Board& goal) const {
  const Grid start = stateToGrid(state);
  const Grid dest = boardToGrid(goal);
  auto unit = solve(start, dest);
  return toMoveList(unit, start);
}

const char* KlotskiSolver::dirName(Dir dir) {
  switch (dir) {
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

}  // namespace klotski
