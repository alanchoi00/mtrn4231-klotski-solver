#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional, Literal

Direction = Literal["left", "right", "up", "down"]

@dataclass(frozen=True)
class Move:
    """One unit slide of a single piece in grid cells."""
    piece: str
    direction: Direction
    steps: int

@dataclass
class NodeInfo:
    """
    One node in the precomputed Klotski graph.

    representation: 20-char row-major board string ('.' = empty).
    neighbors: list of node keys (strings) adjacent by a legal 1-cell slide.
    solution_dist: BFS distance to a goal (0 means solved), if provided.
    dist: distance-from-start used by the viewer (optional; not required for planning).
    """
    representation: str
    neighbors: List[str]
    solution_dist: Optional[int] = None
    dist: Optional[int] = None


def load_table(path: str) -> Tuple[Dict[str, NodeInfo], Dict[str, str]]:
    """
    Load a cleaned data.json and return:

      nodes_by_key: node_key -> NodeInfo
      key_by_rep  : representation -> node_key

    Accepts either:
      { "<key>": {representation, neighbors, ...}, ... }
    or under "nodes" / "nodes_to_use".
    """
    with open(path, "r") as f:
        raw = json.load(f)

    # Accept top-level dict of nodes, or nested under known keys
    if isinstance(raw, dict) and "representation" in next(iter(raw.values()), {}):
        raw_nodes = raw  # already key -> node
    else:
        raw_nodes = raw.get("nodes") or raw.get("nodes_to_use")
        if not isinstance(raw_nodes, dict):
            raise ValueError("data.json must be a dict of nodes, or contain 'nodes'/'nodes_to_use' dict")

    nodes_by_key: Dict[str, NodeInfo] = {}
    key_by_rep: Dict[str, str] = {}

    for key, node in raw_nodes.items():
        rep = node.get("representation")
        if not isinstance(rep, str):
            raise ValueError(f"node {key!r} missing 'representation' string")

        # Normalize neighbors to a list of keys (some datasets store objects with {"to": "key"})
        nbrs = node.get("neighbors", []) or []
        if len(nbrs) > 0 and isinstance(nbrs[0], dict):
            nbrs = [e.get("to") for e in nbrs if isinstance(e, dict) and "to" in e]

        # Optional ints
        sd = node.get("solution_dist")
        d  = node.get("dist")

        info = NodeInfo(
            representation=rep,
            neighbors=list(map(str, nbrs)),
            solution_dist=int(sd) if sd is not None else None,
            dist=int(d) if d is not None else None,
        )
        nodes_by_key[str(key)] = info
        key_by_rep[rep] = str(key)

    return nodes_by_key, key_by_rep

def greedy_toward_goal(nodes: Dict[str, NodeInfo], start_key: str, goal_key: Optional[str]) -> List[str]:
    """
    If goal_key is None: repeatedly step to any neighbor with *strictly smaller* solution_dist until 0.
    Else: BFS from start_key to goal_key over neighbors.
    Returns node keys [start, ..., goal]. Empty list means no path.
    """
    def sd(k: str) -> int:
        n = nodes[k]
        return n.solution_dist if n.solution_dist is not None else 10**9

    if goal_key is None:
        # Greedy descent on precomputed solution distance (optimal if solution_dist is BFS layer)
        path = [start_key]
        cur = start_key
        safety = 100_000
        while sd(cur) != 0 and len(path) < safety:
            cur_sd = sd(cur)
            nbrs = nodes[cur].neighbors
            best_key: Optional[str] = None
            best_sd = cur_sd
            for nk in nbrs:
                if nk not in nodes:
                    continue
                sdn = sd(nk)
                if sdn < best_sd:
                    best_sd = sdn
                    best_key = nk
            if best_key is None:
                return []  # stuck
            path.append(best_key)
            cur = best_key
        return path
    else:
        # Standard BFS to a specific goal node
        q = deque([start_key])
        parent: Dict[str, Optional[str]] = {start_key: None}
        while q:
            s = q.popleft()
            if s == goal_key:
                seq = [s]
                while parent[seq[-1]] is not None:
                    seq.append(parent[seq[-1]])
                return list(reversed(seq))
            for t in nodes[s].neighbors:
                if t in parent:
                    continue
                parent[t] = s
                q.append(t)
        return []

def diff_move(rep_a: str, rep_b: str) -> Move:
    """
    Compute which piece letter moved, in which direction, and how many *grid* steps.
    Assumes a 4x5 grid encoded row-major; row increases downward.
    """
    if len(rep_a) != 20 or len(rep_b) != 20:
        raise ValueError("Representations must be 20 characters long (4x5).")

    letters = sorted(set(rep_a + rep_b) - set('.'))
    moved: Optional[str] = None
    a_cells: List[int] = []
    b_cells: List[int] = []

    for ch in letters:
        a = [i for i, c in enumerate(rep_a) if c == ch]
        b = [i for i, c in enumerate(rep_b) if c == ch]
        if set(a) != set(b):
            moved = ch
            a_cells, b_cells = a, b
            break

    if moved is None:
        return Move(piece="?", direction="up", steps=0)

    def centroid(ids: List[int]) -> Tuple[float, float]:
        rs = [i // 4 for i in ids]
        cs = [i % 4 for i in ids]
        return (sum(rs) / len(rs), sum(cs) / len(cs))

    ra, ca = centroid(a_cells)
    rb, cb = centroid(b_cells)
    dr = rb - ra  # +down
    dc = cb - ca  # +right

    if abs(dc) > abs(dr):
        direction: Direction = "right" if dc > 0 else "left"
        steps = int(round(abs(dc)))
    else:
        direction = "down" if dr > 0 else "up"
        steps = int(round(abs(dr)))

    return Move(piece=moved, direction=direction, steps=steps)


# -----------------------------
# CLI
# -----------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Test Klotski planning using precomputed data.json")
    parser.add_argument("--data", required=True, help="Path to pkg_plan/data.json")
    parser.add_argument("--start-rep", help="20-char start representation (e.g. '..aa....bb...')")
    parser.add_argument("--goal-rep", help="20-char goal representation (optional). If omitted, planner drives to solution_dist==0")
    parser.add_argument("--max-steps", type=int, default=10000, help="Safety cutoff (unused for greedy)")
    args = parser.parse_args()

    nodes, key_by_rep = load_table(args.data)

    def rep_to_key(rep: str) -> str:
        k = key_by_rep.get(rep)
        if k is None:
            raise SystemExit(f"[ERROR] representation not found in data.json:\n{rep}")
        return k

    # Pick a default start if none provided: smallest 'dist' if available, else any node.
    start_rep = args.start_rep
    if start_rep is None:
        best_k = None
        best_d = 1_000_000_000
        for k, n in nodes.items():
            if n.dist is not None and n.dist < best_d:
                best_d, best_k = n.dist, k
        if best_k is None:
            best_k = next(iter(nodes.keys()))
        start_rep = nodes[best_k].representation

    start_key = rep_to_key(start_rep)
    goal_key = rep_to_key(args.goal_rep) if args.goal_rep else None

    # Plan
    path_keys = greedy_toward_goal(nodes, start_key, goal_key)
    if not path_keys or len(path_keys) < 2:
        print("No path found or already at goal.")
        return

    # Build move list by diffing consecutive representations
    path_reps = [nodes[k].representation for k in path_keys]
    moves = [diff_move(a, b) for a, b in zip(path_reps, path_reps[1:])]

    print(f"Plan length: {len(moves)} step(s)")
    for i, mv in enumerate(moves, 1):
        print(f"{i:3d}. piece '{mv.piece}' → {mv.direction} × {mv.steps}")

    # Optional sanity: confirm strictly decreasing solution_dist if goal not explicit
    if goal_key is None:
        dec_ok = True
        for a, b in zip(path_keys, path_keys[1:]):
            sa = nodes[a].solution_dist if nodes[a].solution_dist is not None else 10**9
            sb = nodes[b].solution_dist if nodes[b].solution_dist is not None else 10**9
            if sb >= sa:
                dec_ok = False
                break
        print(f"solution_dist strictly decreasing: {dec_ok}")
    else:
        print(f"Reached goal key: {path_keys[-1] == goal_key}")

if __name__ == "__main__":
    main()