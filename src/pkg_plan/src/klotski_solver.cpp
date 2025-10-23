#include "pkg_plan/klotski_solver.hpp"

namespace klotski {

std::vector<KlotskiSolver::Move> KlotskiSolver::solve(
    const std::string& start, const std::string& goal) const {
  assertRep(start);
  assertRep(goal);
  if (start == goal) return {};

  std::queue<std::string> q;
  q.push(start);

  struct Parent {
    std::string prev;
    char piece;
    Dir dir;
  };
  std::unordered_map<std::string, Parent> parent;
  parent.reserve(32768);
  parent.emplace(start, Parent{std::string{}, 0, Dir::Up});

  while (!q.empty()) {
    std::string cur = std::move(q.front());
    q.pop();

    for (const auto& nb : neighbors(cur)) {
      if (parent.find(nb.rep) != parent.end()) continue;
      parent.emplace(nb.rep, Parent{cur, nb.piece, nb.dir});
      if (nb.rep == goal) {
        // reconstruct unit-step moves
        std::vector<Move> out;
        std::string t = nb.rep;
        while (true) {
          auto it = parent.find(t);
          if (it == parent.end()) break;
          const Parent& P = it->second;
          if (P.prev.empty()) break;
          out.push_back(Move{P.piece, P.dir, 1});
          t = P.prev;
        }
        std::reverse(out.begin(), out.end());
        return out;
      }
      q.push(nb.rep);
    }
  }
  // unreachable
  return {};
}

klotski_interfaces::msg::MoveList KlotskiSolver::solveMoveList(
    const klotski_interfaces::msg::BoardState& state,
    const klotski_interfaces::msg::Board& goal) const {
  std::string startRep = boardToString(state.board);
  std::string goalRep = boardToString(goal);
  auto moves = solve(startRep, goalRep);
  return convertToMoveList(moves, startRep);
}

std::string KlotskiSolver::applyStep(const std::string& rep, char piece,
                                     Dir dir) {
  int dr = 0, dc = 0;
  switch (dir) {
    case Dir::Up:
      dr = -1;
      break;
    case Dir::Down:
      dr = 1;
      break;
    case Dir::Left:
      dc = -1;
      break;
    case Dir::Right:
      dc = 1;
      break;
  }
  if (!canSlide(rep, piece, dr, dc))
    throw std::runtime_error("Illegal unit step");
  return slide(rep, piece, dr, dc);
}

void KlotskiSolver::assertRep(const std::string& rep) {
  if ((int)rep.size() != N)
    throw std::runtime_error("representation must be 20 chars (4x5)");
}

bool KlotskiSolver::inBounds(int r, int c) {
  return (0 <= r && r < H && 0 <= c && c < W);
}

std::vector<std::pair<int, int>> KlotskiSolver::pieceCells(
    const std::string& rep, char ch) {
  std::vector<std::pair<int, int>> cells;
  cells.reserve(4);
  for (int i = 0; i < N; ++i)
    if (rep[i] == ch) cells.emplace_back(rowOf(i), colOf(i));
  return cells;
}

bool KlotskiSolver::canSlide(const std::string& rep, char ch, int dr, int dc) {
  const auto cells = pieceCells(rep, ch);
  for (auto [r, c] : cells) {
    int nr = r + dr, nc = c + dc;
    if (!inBounds(nr, nc)) return false;
    char tgt = rep[idxOf(nr, nc)];
    if (tgt != '.' && tgt != ch) return false;
  }
  return true;
}

std::string KlotskiSolver::slide(const std::string& rep, char ch, int dr,
                                 int dc) {
  std::string out = rep;
  // clear old cells
  for (int i = 0; i < N; ++i)
    if (out[i] == ch) out[i] = '.';
  // place new cells
  for (int i = 0; i < N; ++i)
    if (rep[i] == ch) {
      int r = rowOf(i) + dr, c = colOf(i) + dc;
      out[idxOf(r, c)] = ch;
    }
  return out;
}

std::vector<KlotskiSolver::Step> KlotskiSolver::neighbors(
    const std::string& rep) {
  assertRep(rep);
  std::vector<Step> result;
  result.reserve(16);

  std::array<bool, 256> present{};
  present.fill(false);
  for (char ch : rep)
    if (ch != '.') present[(unsigned char)ch] = true;

  auto try_dir = [&](char ch, int dr, int dc, Dir d) {
    if (canSlide(rep, ch, dr, dc)) {
      result.push_back(Step{slide(rep, ch, dr, dc), ch, d});
    }
  };

  for (int c = 0; c < 256; ++c)
    if (present[c]) {
      char ch = static_cast<char>(c);
      try_dir(ch, -1, 0, Dir::Up);
      try_dir(ch, 1, 0, Dir::Down);
      try_dir(ch, 0, -1, Dir::Left);
      try_dir(ch, 0, 1, Dir::Right);
    }

  // Deduplicate by next state
  std::unordered_map<std::string, Step> uniq;
  uniq.reserve(result.size() * 2);
  for (auto& s : result)
    if (!uniq.count(s.rep)) uniq.emplace(s.rep, s);

  result.clear();
  result.reserve(uniq.size());
  for (auto& kv : uniq) result.push_back(std::move(kv.second));
  return result;
}

klotski_interfaces::msg::MoveList KlotskiSolver::convertToMoveList(
    const std::vector<Move>& moves, const std::string& startRep) const {
  klotski_interfaces::msg::MoveList moveList;

  std::string currentRep = startRep;

  for (const auto& move : moves) {
    // Get current piece position (before move)
    auto currentCells = pieceCells(currentRep, move.piece);

    // Apply the move to get new position
    std::string nextRep = applyStep(currentRep, move.piece, move.dir);
    auto nextCells = pieceCells(nextRep, move.piece);

    // Create ROS Move message
    klotski_interfaces::msg::Move rosMove;

    // Set piece information
    rosMove.piece.id = std::string(1, move.piece);  // Convert char to string
    rosMove.piece.color = "default";  // Default color, could be customized

    // Convert current piece cells to ROS format (bottom-left origin)
    rosMove.piece.cells.reserve(currentCells.size());
    for (const auto& [topRow, col] : currentCells) {
      klotski_interfaces::msg::Cell cell;
      cell.row = toBottomLeftRow(topRow);
      cell.col = col;
      rosMove.piece.cells.push_back(cell);
    }

    // Set destination cell (use first cell of piece after move as
    // representative)
    if (!nextCells.empty()) {
      const auto& [topRow, col] = nextCells.front();
      rosMove.to_cell.row = toBottomLeftRow(topRow);
      rosMove.to_cell.col = col;
    }

    moveList.moves.push_back(rosMove);

    // Update current representation for next iteration
    currentRep = nextRep;
  }

  return moveList;
}

std::string KlotskiSolver::boardToString(const klotski_interfaces::msg::Board& board) {
  // Initialize empty board with dots
  std::string rep(N, '.');
  
  // Place pieces on the board
  for (const auto& piece : board.pieces) {
    if (piece.id.empty()) continue;
    
    char pieceChar = piece.id[0];  // Use first character of piece ID
    
    for (const auto& cell : piece.cells) {
      // Convert from bottom-left origin (ROS) to top-origin (internal)
      int topRow = toBottomLeftRow(cell.row);
      int col = cell.col;
      
      // Validate bounds
      if (inBounds(topRow, col)) {
        int idx = idxOf(topRow, col);
        rep[idx] = pieceChar;
      }
    }
  }
  
  return rep;
}

}  // namespace klotski
