#include <array>
#include <cctype>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "pkg_plan/klotski_solver.hpp"

using klotski::KlotskiSolver;

static std::string stripNonDigitsDot(const std::string& s) {
  std::string out;
  out.reserve(s.size());
  for (char ch : s) {
    if (ch >= '0' && ch <= '9')
      out.push_back(ch);
    else if (ch == '.')
      out.push_back('0');
  }
  return out;
}

static std::array<int, 5> histogram(const KlotskiSolver::Grid& g) {
  std::array<int, 5> h{};
  h.fill(0);
  for (auto v : g) {
    if (v < 0 || v > 4) continue;
    h[v] += 1;
  }
  return h;
}

static bool sameInventory(const KlotskiSolver::Grid& a,
                          const KlotskiSolver::Grid& b,
                          std::string* why = nullptr) {
  auto ha = histogram(a);
  auto hb = histogram(b);
  if (ha != hb) {
    if (why) {
      *why =
          "piece inventory mismatch: "
          "type1=" +
          std::to_string(ha[1]) + " vs " + std::to_string(hb[1]) +
          ", "
          "type2=" +
          std::to_string(ha[2]) + " vs " + std::to_string(hb[2]) +
          ", "
          "type3=" +
          std::to_string(ha[3]) + " vs " + std::to_string(hb[3]) +
          ", "
          "type4=" +
          std::to_string(ha[4]) + " vs " + std::to_string(hb[4]);
    }
    return false;
  }
  return true;
}

static std::optional<KlotskiSolver::Grid> parseGrid20(
    const std::string& token) {
  // token must be 20 chars of '.' or digits (0..9). We use 0..4 only.
  if (token.size() != KlotskiSolver::W * KlotskiSolver::H) return std::nullopt;
  KlotskiSolver::Grid g{};
  g.fill(0);
  for (size_t i = 0; i < token.size(); ++i) {
    char ch = token[i];
    if (ch == '.') {
      g[i] = 0;
      continue;
    }
    if (!std::isdigit(static_cast<unsigned char>(ch))) return std::nullopt;
    int v = ch - '0';
    if (v < 0 || v > 9) return std::nullopt;
    g[i] = static_cast<uint8_t>(v);
  }
  return g;
}

static std::optional<KlotskiSolver::Grid> parseRows(
    const std::string& rowsSpec) {
  // format: "r0/r1/r2/r3/r4" each r? has length 4 of '.' or digits
  // OR: 20-char string that gets split into 4-char rows

  std::array<std::string, KlotskiSolver::H> rows{};

  if (rowsSpec.find('/') == std::string::npos) {
    // No '/' found, split every 4 characters
    if (rowsSpec.size() != KlotskiSolver::W * KlotskiSolver::H) {
      return std::nullopt;
    }
    for (size_t r = 0; r < rows.size(); ++r) {
      rows[r] = rowsSpec.substr(r * KlotskiSolver::W, KlotskiSolver::W);
    }
  } else {
    // Parse with '/' delimiter
    size_t pos = 0, next = 0, r = 0;
    while (r < rows.size() &&
           (next = rowsSpec.find('/', pos)) != std::string::npos) {
      rows[r++] = rowsSpec.substr(pos, next - pos);
      pos = next + 1;
    }
    if (r < rows.size()) {
      rows[r++] = rowsSpec.substr(pos);
    }
    if (r != rows.size()) return std::nullopt;
  }

  std::string flat;
  flat.reserve(KlotskiSolver::W * KlotskiSolver::H);
  for (const auto& row : rows) {
    if (row.size() != KlotskiSolver::W) return std::nullopt;
    flat += row;
  }
  return parseGrid20(flat);
}

static void printUsage(const char* exe) {
  std::cerr << "Usage:\n"
               "  "
            << exe
            << " START20 GOAL20\n"
               "  "
            << exe
            << " --rows START_ROWS --goal-rows GOAL_ROWS\n"
               "\n"
               "Where START20/GOAL20 are 20-char strings of '.' and digits "
               "(0..4) in row-major, top row first.\n"
               "Example (4x5): 31133113.22.34433443\n"
               "\n"
               "Or with rows:\n"
               "  --rows      \"3113/3113/.22./3443/3443\"\n"
               "  --goal-rows \"3113/3113/.22./3443/3443\"\n";
}

int main(int argc, char** argv) {
  KlotskiSolver solver;

  KlotskiSolver::Grid start, goal;

  // Parse command line
  if (argc == 3) {
    // Two 20-char tokens (allow extra non-digits like spaces we strip out)
    auto s = stripNonDigitsDot(argv[1]);
    auto g = stripNonDigitsDot(argv[2]);
    auto startOpt = parseGrid20(s);
    auto goalOpt = parseGrid20(g);
    if (!startOpt || !goalOpt) {
      printUsage(argv[0]);
      return 1;
    }
    start = *startOpt;
    goal = *goalOpt;
  } else {
    // Look for --rows / --goal-rows
    std::string startRows, goalRows;
    for (int i = 1; i < argc; ++i) {
      std::string a = argv[i];
      if (a == "--rows" && i + 1 < argc) {
        startRows = argv[++i];
      } else if (a == "--goal-rows" && i + 1 < argc) {
        goalRows = argv[++i];
      } else {
        printUsage(argv[0]);
        return 1;
      }
    }
    if (!startRows.empty() && !goalRows.empty()) {
      auto startOpt = parseRows(startRows);
      auto goalOpt = parseRows(goalRows);
      if (!startOpt || !goalOpt) {
        printUsage(argv[0]);
        return 1;
      }
      start = *startOpt;
      goal = *goalOpt;
    } else {
      // Default tiny example (start == goal → 0 steps)
      const std::string flat =
          "3113"
          "3113"
          ".22."
          "3443"
          "3443";
      auto startOpt = parseGrid20(flat);
      auto goalOpt = parseGrid20(flat);
      start = *startOpt;
      goal = *goalOpt;
    }
  }

  std::string why;
  if (!sameInventory(start, goal, &why)) {
    std::cout << "Start grid:\n" << KlotskiSolver::debugString(start);
    std::cout << "Goal  grid:\n" << KlotskiSolver::debugString(goal);
    std::cout << "No solution: " << why << "\n";
    return 0;
  }

  // Solve
  auto unitMoves = solver.solve(start, goal);

  // Output
  std::cout << "Start grid:\n" << KlotskiSolver::debugString(start);
  std::cout << "Goal  grid:\n" << KlotskiSolver::debugString(goal);
  std::cout << "Shortest path length (unit steps): " << unitMoves.size()
            << "\n";

  // Pretty print each unit move:
  // We print the moved cells (bottom-left origin) and direction.
  auto toBottomRow = [](int top) { return KlotskiSolver::H - 1 - top; };
  for (size_t i = 0; i < unitMoves.size(); ++i) {
    const auto& m = unitMoves[i];
    std::cout << i + 1 << ". move cells {";
    for (size_t k = 0; k < m.from_cells.size(); ++k) {
      auto [r, c] = m.from_cells[k];
      std::cout << "(" << c << "," << toBottomRow(r) << ")";
      if (k + 1 < m.from_cells.size()) std::cout << ",";
    }
    std::cout << "} → " << KlotskiSolver::dirName(m.dir) << "\n";
  }

  return 0;
}
