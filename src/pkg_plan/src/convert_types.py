#!/usr/bin/env python3
"""
Convert Klotski node 'representation' (a..j) into type-layout strings (0..4)
and list all unique combinations present in data.json.

Mapping (from your picture):
  a,c,e,h  -> vertical 1x2  -> type 3
  b        -> 2x2           -> type 1
  d        -> horizontal 2x1-> type 2
  f,g,i,j  -> single 1x1    -> type 4
  '.'      -> empty         -> '0'
"""

import argparse
import json
import sys
from typing import Dict, Iterable, Set, Tuple

# 4x5 board, row-major, top row first
W, H = 4, 5

# Letter -> type-id (digits as chars for easy string building)
LETTER_TO_TYPE: Dict[str, str] = {
    # fixed pieces (see figure)
    "a": "3", "c": "3", "e": "3", "h": "3",          # 2x1 vertical
    "b": "1",                                        # 2x2
    "d": "2",                                        # 1x2 horizontal
    "f": "4", "g": "4", "i": "4", "j": "4",          # 1x1
    ".": "0",
}

def load_nodes(path: str) -> Dict[str, dict]:
    """Accepts a flat dict of nodes, or {nodes}/{nodes_to_use}."""
    with open(path, "r") as f:
        raw = json.load(f)
    if isinstance(raw, dict) and raw and "representation" in next(iter(raw.values())):
        return raw
    for k in ("nodes", "nodes_to_use"):
        if isinstance(raw, dict) and isinstance(raw.get(k), dict):
            return raw[k]
    raise SystemExit("data.json does not look like a node dictionary")

def rep_to_types(rep: str) -> str:
    """Convert 'abbc...' (20 chars) to '3113...' (20 digits)."""
    if len(rep) != W*H:
        raise ValueError(f"representation must be {W*H} chars, got {len(rep)}")
    out = []
    for ch in rep:
        try:
            out.append(LETTER_TO_TYPE[ch])
        except KeyError:
            raise ValueError(f"unexpected letter '{ch}' in representation")
    return "".join(out)

def main():
    ap = argparse.ArgumentParser(description="Extract unique type combinations from data.json")
    ap.add_argument("--data", required=True, help="Path to data.json")
    ap.add_argument("--write-json", metavar="OUT.json",
                    help="Optional: write unique type-strings as a JSON list")
    ap.add_argument("--show-samples", action="store_true",
                    help="Print first few example mappings")
    args = ap.parse_args()

    nodes = load_nodes(args.data)

    unique_types: Set[str] = set()
    # (optional) keep one example key per type layout
    exemplar: Dict[str, Tuple[str, str]] = {}  # types -> (node_key, rep)

    for key, node in nodes.items():
        rep = node.get("representation")
        if not isinstance(rep, str):
            continue
        types = rep_to_types(rep)
        if types not in unique_types:
            exemplar[types] = (str(key), rep)
        unique_types.add(types)

    uniq_list = sorted(unique_types)

    print(f"Total nodes: {len(nodes)}")
    print(f"Unique type combinations: {len(uniq_list)}")
    print()

    # Show a few examples
    if args.show_samples:
        print("Examples:")
        shown = 0
        for t in uniq_list[:10]:
            k, rep = exemplar[t]
            print(f"  key={k}  rep={rep}  ->  types={t}")
            shown += 1
        if len(uniq_list) > shown:
            print(f"  ... (+{len(uniq_list)-shown} more)")
        print()

    # Optional: write to JSON
    if args.write_json:
        with open(args.write_json, "w") as f:
            json.dump(uniq_list, f, indent=2)
        print(f"Wrote {len(uniq_list)} unique combinations to {args.write_json}")

if __name__ == "__main__":
    main()
