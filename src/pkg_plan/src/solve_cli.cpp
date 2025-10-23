#include <iostream>
#include <set>

#include "pkg_plan/klotski_solver.hpp"

int main(int argc, char** argv) {
  using namespace klotski;
  KlotskiSolver solver;

  std::string start =
      "abbc"
      "abbc"
      ".dd."
      "efgh"
      "eijh";
  std::string goal =
      "bbgh"
      "bbch"
      "aec."
      "aef."
      "iddj";

  if (argc == 3) {
    start = argv[1];
    goal = argv[2];
  } else if (argc != 1) {
    std::cerr << "Usage: " << argv[0] << " [START_REP GOAL_REP]\n";
    return 1;
  }

  auto unit = solver.solve(start, goal);
  auto fused = KlotskiSolver::coalesce(unit);

  int total = 0;
  for (auto& m : fused) total += m.steps;
  std::cout << "Shortest path length (unit steps): " << total << "\n";
  for (size_t i = 0; i < fused.size(); ++i) {
    const auto& m = fused[i];
    std::cout << (i + 1) << ". piece '" << m.piece << "' → "
              << KlotskiSolver::dirName(m.dir) << " × " << m.steps << "\n";
  }

  return 0;
}
