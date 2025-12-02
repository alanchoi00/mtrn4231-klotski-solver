
# ros2 run pkg_sense mock_baselink \
#   --ros-args \
#     -p mode:=rows \
#     -p rows:="3113/3113/.22./3443/3443" \
#     -p board_x:=0.70 \
#     -p board_y:=0.08

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

    # '.' 或 '0' → 0，其他数字转 int
    grid = []
    for r in rows:
        grid.append([0 if ch in ('.', '0') else int(ch) for ch in r])

    seen = [[False] * W for _ in range(H)]

    def inb(rr, cc):
        return 0 <= rr < H and 0 <= cc < W

    # type-id -> (color, piece_type)
    # 1: 2x2 (red), 2: 1x2 (green), 3: 2x1 (blue), 4: 1x1 (yellow)
    meta = {
        1: (COLOR_RED, TYPE_2x2),
        2: (COLOR_GREEN, TYPE_1x2),
        3: (COLOR_BLUE, TYPE_2x1),
        4: (COLOR_YELLOW, TYPE_1x1),
    }

    for r0 in range(H):
        for c0 in range(W):
            if seen[r0][c0] or grid[r0][c0] == 0:
                continue
            tid = grid[r0][c0]
            stack = [(r0, c0)]
            comp = []
            seen[r0][c0] = True
            while stack:
                r, c = stack.pop()
                comp.append((r, c))
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr, nc = r + dr, c + dc
                    if inb(nr, nc) and (not seen[nr][nc]) and grid[nr][nc] == tid:
                        seen[nr][nc] = True
                        stack.append((nr, nc))

            if tid not in meta:
                continue
            color, ptype = meta[tid]

            piece = Piece()
            piece.type = ptype
            piece.color = color
            for (rt, ct) in comp:
                # 你的 Cell 是 bottom-origin 的：row=0 在最底下一行
                rb = H - 1 - rt  # top-origin -> bottom-origin
                cell = Cell()
                cell.col = ct
                cell.row = rb
                piece.cells.append(cell)

            b.pieces.append(piece)

    return b


class MockSense(Node):
    def __init__(self):
        super().__init__("mock_sense")

        self.ui_pub = self.create_publisher(String, "/ui/events", 10)
        self.state_pub = self.create_publisher(BoardState, "/board_state", 10)

        # 保持原来的参数接口
        self.declare_parameter("mode", "rows")  # rows | echo_goal | static
        self.declare_parameter("rows", "3113/3113/.22./3443/3443")

        # 现在 frame_id 默认就是 base_link
        self.declare_parameter("frame_id", "base_link")

        # 棋盘在 base_link 下的位置（随便给个合理的）
        self.declare_parameter("board_x", 0.60)
        self.declare_parameter("board_y", 0.00)
        self.declare_parameter("board_z", 0.00)

        self.srv = self.create_service(
            CaptureBoard, "/sense/capture_board", self.on_capture
        )

        self.get_logger().info(
            "mock_sense ready (BoardState.board_pose in base_link)"
        )

    def on_capture(self, req, res):
        mode = self.get_parameter("mode").get_parameter_value().string_value
        rows_param = self.get_parameter("rows").get_parameter_value().string_value
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        bx = self.get_parameter("board_x").get_parameter_value().double_value
        by = self.get_parameter("board_y").get_parameter_value().double_value
        bz = self.get_parameter("board_z").get_parameter_value().double_value

        self._ui(f"SENSE: capture request (mode={mode})")

        # 1. 按 mode 生成 Board
        if mode == "rows":
            rows = rows_param.split("/")
            if len(rows) != H or any(len(r) != W for r in rows):
                res.ok = False
                res.note = "Bad rows parameter"
                return res
            board = rows_to_board(rows)

        elif mode == "echo_goal":
            rows = rows_param.split("/")
            board = rows_to_board(rows)

        else:  # static
            rows = "3113/3113/.22./3443/3443".split("/")
            board = rows_to_board(rows)

        # 2. 填 BoardState（匹配你现在的 msg）
        state = BoardState()
        state.board = board

        now = self.get_clock().now().to_msg()
        state.stamp = now

        # --- 关键：只有一个 board_pose，用 base_link ---
        state.board_pose.header.stamp = now
        state.board_pose.header.frame_id = frame_id  # 默认 base_link

        state.board_pose.pose.position.x = bx
        state.board_pose.pose.position.y = by
        state.board_pose.pose.position.z = bz
        state.board_pose.pose.orientation.x = 0.0
        state.board_pose.pose.orientation.y = 0.0
        state.board_pose.pose.orientation.z = 0.0
        state.board_pose.pose.orientation.w = 1.0  # 无旋转

        res.state = state
        res.ok = True
        res.note = "mock capture OK (board_pose in base_link)"

        # 顺便发到 topic 上，方便你用 arm_manipulator
        self.state_pub.publish(state)
        self._ui("SENSE: published /board_state (mock, base_link)")
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
