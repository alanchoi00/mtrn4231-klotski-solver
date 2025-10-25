from __future__ import annotations

from typing import List

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String

from klotski_interfaces.action import MovePiece
from klotski_interfaces.msg import Board, BoardState, Move, MoveList, UICommand
from klotski_interfaces.srv import CaptureBoard, SolveBoard

from .context import BrainContext
from .handlers import BaseHandler, ExecuteHandler, PlanHandler, SenseHandler
from .ui_modes import UIMode


class TaskBrain(Node):
    """
    Brain node with a Chain-of-Responsibility pipeline:
      Sense -> Plan -> Execute
    Modes from UICommand: MODE_AUTO, MODE_STEP, MODE_PAUSE, MODE_RESET
    """

    def __init__(self):
        super().__init__("task_brain")

        self.declare_parameter("relocalise_between_moves", True)

        self.ui_pub = self.create_publisher(String, "/ui/events", 10)
        self.create_subscription(UICommand, "/ui/cmd", self.on_ui_cmd, 10)
        self.create_subscription(Board, "/ui/goal", self.on_goal, 10)
        self.create_subscription(BoardState, "/board_state", self.on_board_state, 10)

        # planner service
        self.solve_cli = self.create_client(SolveBoard, "/plan/solve")

        # sense service
        self.sense_cli = self.create_client(CaptureBoard, "/sense/capture_board")

        # manip action (may not exist yet)
        self.move_client: ActionClient = ActionClient(self, MovePiece, "/move_piece")

        self.ctx = BrainContext()
        self.pipeline: List[BaseHandler] = [SenseHandler(), PlanHandler(), ExecuteHandler()]

        self.ui("TaskBrain up. Modes: auto | step | pause | reset")

    def ui(self, msg: str) -> None:
        self.ui_pub.publish(String(data=msg))
        self.get_logger().info(msg)

    def debug(self, msg: str) -> None:
        self.get_logger().debug(msg)

    def warn(self, msg: str) -> None:
        self.get_logger().warn(msg)

    def on_ui_cmd(self, cmd: UICommand) -> None:
        mode = (cmd.mode or UIMode.IDLE)
        if UIMode.is_valid(mode):
            if mode == UIMode.RESET:
                self.ui("UI: reset -> pause + clear memory")
                self.ctx.reset()
            elif mode == UIMode.PAUSE:
                self.ui("UI: pause")
                self.ctx.mode = UIMode.PAUSE
                # If an action were active, you could cancel here.
            else:
                mode_name = UIMode.to_string(mode)
                self.ui(f"UI: mode={mode_name}")
                self.ctx.mode = mode
        else:
            self.ui(f"UI: unknown mode '{cmd.mode}' (use auto|step|pause|reset)")

        self.tick("ui_cmd")

    def on_goal(self, msg: Board) -> None:
        self.ctx.goal = msg
        self.ui(f"Goal pattern received: {self._board_to_pattern(msg)}")
        # Usually we'll want to replan on new goal
        self.ctx.replan_requested = True
        self.ctx.plan_received = False  # invalidate old plan
        self.tick("goal")

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

    def on_board_state(self, state: BoardState) -> None:
        self.ctx.sensed = state
        self.ui(f"BoardState received: {len(state.board.pieces)} pieces")
        # sensing updated -> replan
        self.ctx.replan_requested = True
        self.ctx.plan_received = False  # invalidate old plan
        self.tick("board_state")

    # ------------- Pipeline tick -------------
    def tick(self, source: str) -> None:
        self.ctx.tick_source = source
        mode_name = UIMode.to_string(self.ctx.mode)
        self.debug(f"[tick] source={source} mode={mode_name} busy={self.ctx.busy} plan_idx={self.ctx.plan_index}/{len(self.ctx.plan)}")
        for h in self.pipeline:
            result, reason = h.handle(self.ctx, self)
            self.debug(f"  -> handler={h.name} result={result} reason={reason}")
            if result == "pending" or result == "done":
                break
            # else "next" -> continue

    # ------------- Stage: SENSE -------------
    def start_sense(self) -> bool:
        if not self.sense_cli.service_is_ready():
            self.debug("[sense] /sense/capture_board service not ready")
            return False

        req = CaptureBoard.Request()
        fut = self.sense_cli.call_async(req)
        fut.add_done_callback(self._on_sense_result)
        self.ui("[sense] Requesting board capture from /sense/capture_board")
        return True

    def _on_sense_result(self, fut: Future) -> None:
        try:
            res: CaptureBoard.Response = fut.result()
        except Exception as e:
            self.ctx.last_error = f"Sense failed: {e}"
            self.warn(self.ctx.last_error)
            self.ui("[sense] Failed (see logs)")
            return

        if res.ok:
            self.ctx.sensed = res.state
            self.ui(f"[sense] Captured board: {len(res.state.board.pieces)} pieces")
            # New sensing may require replanning
            self.ctx.replan_requested = True
            self.tick("sense_done")
        else:
            self.ui(f"[sense] Capture failed: {res.note}")

    # ------------- Stage: PLAN -------------
    def start_plan(self) -> bool:
        if not self.solve_cli.service_is_ready():
            return False

        req = SolveBoard.Request()
        req.state = self.ctx.sensed
        req.goal = self.ctx.goal

        fut = self.solve_cli.call_async(req)
        fut.add_done_callback(self._on_plan_result)
        self.ui("[plan] Requested plan from /plan/solve")
        return True

    def _on_plan_result(self, fut: Future) -> None:
        try:
            res: SolveBoard.Response = fut.result()
        except Exception as e:
            self.ctx.last_error = f"Plan failed: {e}"
            self.warn(self.ctx.last_error)
            self.ui("[plan] Failed (see logs)")
            # Leave ctx.plan as-is; stay paused
            return

        move_list: MoveList = res.plan
        self.ctx.plan = list(move_list.moves)
        self.ctx.plan_index = 0
        self.ctx.plan_received = True
        self.ctx.replan_requested = False

        if len(self.ctx.plan) == 0:
            if res.solved:
                self.ui(f"[plan] Goal already achieved! (0 moves needed)")
            else:
                self.ui(f"[plan] No solution found (0 moves, not solved)")
        else:
            self.ui(f"[plan] Received plan: {len(self.ctx.plan)} move(s), solved={res.solved}")

        self.tick("plan_done")

    # ------------- Stage: EXECUTE -------------
    def start_execute_next_move(self) -> bool:
        if self.ctx.plan_index >= len(self.ctx.plan):
            return False

        move: Move = self.ctx.plan[self.ctx.plan_index]

        # Try to use the MovePiece action if present; otherwise say not implemented.
        if not self.move_client.wait_for_server(timeout_sec=0.2):
            self.ui("[exec] /move_piece action server not available (not implemented yet)")
            return False

        goal = MovePiece.Goal()
        goal.move = move
        # Optional grasp_frame if your action supports it:
        if hasattr(goal, "grasp_frame"):
            if move.piece.cells:
                c0 = move.piece.cells[0]
                goal.grasp_frame = f"grasp_c{c0.col}_r{c0.row}"
            else:
                goal.grasp_frame = "grasp_unknown"

        self.ctx.busy = True
        self.ui(f"[exec] Send MovePiece: type={move.piece.type} -> to=({move.to_cell.col},{move.to_cell.row})")
        send_fut = self.move_client.send_goal_async(goal, feedback_callback=self._on_exec_feedback)
        send_fut.add_done_callback(self._on_exec_goal_response)
        return True

    def _on_exec_goal_response(self, goal_fut: Future) -> None:
        goal_handle = goal_fut.result()
        if not goal_handle.accepted:
            self.ui("[exec] MovePiece goal rejected")
            self.ctx.busy = False
            # Pause on rejection
            self.ctx.mode = UIMode.PAUSE
            return
        self.debug("[exec] MovePiece accepted")
        result_fut = goal_handle.get_result_async()
        result_fut.add_done_callback(self._on_exec_result)

    def _on_exec_feedback(self, fb: MovePiece.Feedback) -> None:
        phase = getattr(fb, "phase", "")
        prog = getattr(fb, "progress", 0.0)
        self.ui(f"[exec] feedback: {phase} {prog:.2f}")

    def _on_exec_result(self, res_fut: Future) -> None:
        try:
            result = res_fut.result().result
            ok = bool(getattr(result, "success", False))
            note = getattr(result, "note", "")
        except Exception as e:
            ok = False
            note = f"exception: {e}"

        self.ctx.busy = False
        if ok:
            self.ui(f"[exec] move OK ({note})")
            self.ctx.plan_index += 1
            # Continue in AUTO; pause in STEP
            if self.ctx.mode == UIMode.AUTO:
                self.tick("exec_next_auto")
            elif self.ctx.mode == UIMode.STEP:
                self.ctx.mode = UIMode.PAUSE
                self.tick("exec_step_done")
        else:
            self.ui(f"[exec] move FAILED ({note}) -> pause")
            self.ctx.mode = UIMode.PAUSE
            self.tick("exec_failed")

def main() -> None:
    rclpy.init()
    node = TaskBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
