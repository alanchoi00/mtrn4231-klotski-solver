#!/usr/bin/env python3
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from klotski_interfaces.msg import Board, BoardSpec, BoardState, Cell, Piece
from klotski_interfaces.srv import CaptureBoard

W, H = 4, 5

TYPE_2x2 = 1
TYPE_1x2 = 2  # horizontal
TYPE_2x1 = 3  # vertical
TYPE_1x1 = 4

COLOR_NONE = 0
COLOR_RED = 1
COLOR_BLUE = 2
COLOR_GREEN = 3
COLOR_YELLOW = 4

def rows_to_board(rows: List[str]) -> Board:
    """rows are top-origin strings, e.g. ['3113','3113','.22.','3443','3443']"""
    b = Board()
    b.spec = BoardSpec(cols=W, rows=H, cell_size_m=0.03, board_thickness_m=0.02)

    # flatten digits to grid of ints; '.' or '0' -> 0
    grid = []
    for r in rows:
        grid.append([0 if ch in ('.', '0') else int(ch) for ch in r])

    # build connected components (4-neighborhood) per digit (type id)
    seen = [[False]*W for _ in range(H)]
    def inb(rr, cc): return 0 <= rr < H and 0 <= cc < W

    # map type-id -> (w,h,color)
    # 1: 2x2 (red), 2: 2x1 (green), 3: 1x2 (blue), 4: 1x1 (yellow)
    meta = {
        1: (2, 2, COLOR_RED,  TYPE_2x2),
        2: (2, 1, COLOR_GREEN, TYPE_1x2),
        3: (1, 2, COLOR_BLUE, TYPE_2x1),
        4: (1, 1, COLOR_YELLOW, TYPE_1x1),
    }

    for r0 in range(H):
        for c0 in range(W):
            if seen[r0][c0] or grid[r0][c0] == 0: 
                continue
            tid = grid[r0][c0]
            # flood
            stack = [(r0,c0)]
            comp = []
            seen[r0][c0] = True
            while stack:
                r,c = stack.pop()
                comp.append((r,c))
                for dr,dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                    nr, nc = r+dr, c+dc
                    if inb(nr,nc) and not seen[nr][nc] and grid[nr][nc] == tid:
                        seen[nr][nc] = True
                        stack.append((nr,nc))

            # sanity (approx): comp size should match meta area
            if tid not in meta: 
                continue
            exp_w, exp_h, col, typ = meta[tid]
            if len(comp) != exp_w * exp_h:
                # still emit but warn (useful to catch bad patterns)
                pass

            # convert top-origin comp to bottom-left origin cells
            piece = Piece()
            piece.type = typ
            piece.color = col
            for (rt, ct) in comp:
                rb = H - 1 - rt
                piece.cells.append(Cell(col=ct, row=rb))

            b.pieces.append(piece)

    return b

class MockSense(Node):
    def __init__(self):
        super().__init__('mock_sense')
        self.ui_pub = self.create_publisher(String, '/ui/events', 10)

        # Parameters: pick how we generate a board
        self.declare_parameter('mode', 'rows')  # rows|echo_goal|static
        self.declare_parameter('rows', "3113/3113/.22./3443/3443")  # default
        self.declare_parameter('frame_id', 'map')

        # Optional: publish last-captured state on a topic for convenience
        self.state_pub = self.create_publisher(BoardState, '/board_state', 10)

        self.srv = self.create_service(CaptureBoard, '/sense/capture_board', self.on_capture)
        self.get_logger().info("mock_sense ready: /sense/capture_board")

    def on_capture(self, req, res):
        mode = self.get_parameter('mode').get_parameter_value().string_value
        rows_param = self.get_parameter('rows').get_parameter_value().string_value
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self._ui(f"SENSE: capture request (mode={mode})")

        if mode == 'rows':
            rows = rows_param.split('/')
            if len(rows) != H or any(len(r)!=W for r in rows):
                res.ok = False
                res.note = "Bad rows parameter"
                return res
            board = rows_to_board(rows)

        elif mode == 'echo_goal':
            # If you want: read current goal off a latched topic or param.
            # For now, just mirror the default rows.
            rows = rows_param.split('/')
            board = rows_to_board(rows)

        else:  # 'static'
            rows = "3113/3113/.22./3443/3443".split('/')
            board = rows_to_board(rows)

        state = BoardState()
        state.board = board
        state.board_pose_map.header.frame_id = frame_id
        # (You can fill a fixed pose if useful; leaving zeros is fine for a mock)

        res.state = state
        res.ok = True
        res.note = "mock capture OK"

        # also publish on /board_state to help your existing brain flow
        self.state_pub.publish(state)
        self._ui("SENSE: published /board_state (mock)")
        return res

    def _ui(self, text: str):
        self.ui_pub.publish(String(data=text))
        self.get_logger().info(text)

def main():
    rclpy.init()
    rclpy.spin(MockSense())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
