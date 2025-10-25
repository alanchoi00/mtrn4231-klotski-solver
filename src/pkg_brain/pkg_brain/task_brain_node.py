from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from klotski_interfaces.action import MovePiece
from klotski_interfaces.msg import Board, BoardState, Move, MoveList, UICommand
from klotski_interfaces.srv import SolveBoard


class TaskBrain(Node):
    """
    Orchestrator (no hardware required):
    - Subscribes: /ui/cmd (UICommand), /board_state (BoardState)
    - Publishes: /ui/events (String)
    - Clients: /plan/solve (SolveBoard.srv), /move_piece (MovePiece.action)
    Behaviour:
      On /board_state:
        - Call /plan/solve (if service available)
        - Send first /move_piece goal (if action server available)
        - Emit progress via /ui/events
    Robust to missing solver/action (logs and waits).
    """
    def __init__(self):
        super().__init__('task_brain')

        # Params
        self.declare_parameter('auto_continue', True)
        self.declare_parameter('relocalise_between_moves', True)
        self.auto_continue = self.get_parameter('auto_continue').get_parameter_value().bool_value

        # IO
        self.create_subscription(UICommand, '/ui/cmd', self.on_ui_cmd, 10)
        self.create_subscription(BoardState, '/board_state', self.on_board_state, 10)
        self.ui_pub = self.create_publisher(String, '/ui/events', 10)
        self.create_subscription(Board, '/ui/goal', self.on_goal, 10)
        self.goal: Optional[Board] = None

        # Clients
        self.solve_cli = self.create_client(SolveBoard, '/plan/solve')
        self.move_client = ActionClient(self, MovePiece, '/move_piece')

        self.plan_queue = []
        self.busy = False
        self._say("task_brain up: waiting for /board_state")

    # ----- subscribers -----
    def on_ui_cmd(self, cmd: UICommand):
        if self.goal is None:
            self._say("Please set a goal pattern first (/ui/goal). Ignoring command.")
            return
        self._say(f"UI cmd: mode={cmd.mode}")

    def on_board_state(self, state: BoardState):
        if self.goal is None:
            self._say("Waiting for goal pattern before solving...")
            return
        # Decide whether to solve
        if not self.solve_cli.service_is_ready():
            self._say("Waiting for /plan/solve service...")
            return

        self._say(f"BoardState received with {len(state.pieces)} piece(s); requesting plan...")
        req = SolveBoard.Request()
        req.state = state
        req.goal = self.goal
        future = self.solve_cli.call_async(req)
        future.add_done_callback(self._after_solve)

    def on_goal(self, msg: Board):
        self.goal = msg
        pattern = self._board_to_pattern(msg)
        self._say(f"Goal received: {len(msg.pieces)} pieces for {msg.spec.cols}x{msg.spec.rows}, pattern: {pattern}")

    def _board_to_pattern(self, board: Board) -> str:
        """Convert a Board message to a pattern string for display"""
        W = board.spec.cols
        H = board.spec.rows

        # Create top-origin grid of digits (0 = empty)
        grid = [[0 for _ in range(W)] for _ in range(H)]

        # Fill grid with piece types at their cell positions
        for piece in board.pieces:
            for cell in piece.cells:
                col, row = cell.col, cell.row

                # Convert bottom-left origin (ROS) to top-origin (pattern display)
                top_row = H - 1 - row
                left_col = col

                if 0 <= top_row < H and 0 <= left_col < W:
                    grid[top_row][left_col] = piece.type

        # Flatten to string (row-major, top row first)
        result = ""
        for r in range(H):
            for c in range(W):
                result += str(grid[r][c])

        return result

    # ----- solver callback -----
    def _after_solve(self, fut):
        try:
            res = fut.result()
        except Exception as e:
            self._say(f"/plan/solve failed: {e}")
            return

        moves: MoveList = res.plan
        self.plan_queue = list(moves.moves) if moves else []
        self._say(f"Plan: {len(self.plan_queue)} move(s). Solved={res.solved} note={res.note}")

        if self.plan_queue and not self.busy:
            self._send_next_move()

    # ----- action send/feedback/result -----
    def _send_next_move(self):
        if not self.plan_queue:
            self._say("No moves pending.")
            return
        if not self.move_client.wait_for_server(timeout_sec=0.5):
            self._say("Waiting for /move_piece action server...")
            return

        move: Move = self.plan_queue.pop(0)
        goal = MovePiece.Goal()
        goal.move = move
        goal.grasp_frame = f"grasp_{move.piece_id}"

        self.busy = True
        self._say(f"Sending move: {move.piece_id} {move.direction} dx={move.dx:.3f} dy={move.dy:.3f}")

        send_future = self.move_client.send_goal_async(goal, feedback_callback=self._on_fb)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, goal_future):
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self._say("MovePiece goal rejected")
            self.busy = False
            return
        self._say("MovePiece goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_fb(self, fb):
        phase = fb.feedback.phase
        prog = fb.feedback.progress
        self._say(f"move feedback: {phase} {prog:.2f}")

    def _on_result(self, res_future):
        result = res_future.result().result
        self._say(f"move result: success={result.success} note={result.note}")
        self.busy = False

        if self.plan_queue and self.auto_continue:
            self._send_next_move()
        else:
            self._say("Plan complete or auto_continue disabled.")

    # ----- utils -----
    def _say(self, text: str):
        self.ui_pub.publish(String(data=text))
        self.get_logger().info(text)

def main():
    rclpy.init()
    node = TaskBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
