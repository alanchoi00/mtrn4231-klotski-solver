#!/usr/bin/env python3
"""
Test Brain Node - 发送测试MovePiece action
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from klotski_interfaces.action import MovePiece
from klotski_interfaces.msg import Move, Piece, Cell
import time


class TestBrain(Node):
    def __init__(self):
        super().__init__('test_brain')
        
        # 创建action客户端
        self._action_client = ActionClient(
            self,
            MovePiece,
            '/arm_manipulation/move_piece'
        )
        
        self.get_logger().info('Test Brain Node started')
        self.get_logger().info('Waiting for arm_manipulation action server...')
    
    def send_move_action(self, from_cells, to_cell, piece_type=4, color=0):
        """
        发送移动指令
        
        Args:
            from_cells: list of (row, col) tuples - 当前占据的格子
            to_cell: (row, col) tuple - 目标位置
            piece_type: 棋子类型 (1=2x2, 2=1x2, 3=2x1, 4=1x1)
            color: 颜色 (0=NONE, 1=RED, etc)
        """
        # 等待action server
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return None
        
        # 创建goal
        goal_msg = MovePiece.Goal()
        
        # 设置piece信息
        goal_msg.move.piece.type = piece_type
        goal_msg.move.piece.color = color
        
        for row, col in from_cells:
            cell = Cell()
            cell.row = row
            cell.col = col
            goal_msg.move.piece.cells.append(cell)
        
        # 设置目标位置
        goal_msg.move.to_cell.row = to_cell[0]
        goal_msg.move.to_cell.col = to_cell[1]
        
        # 设置phase
        goal_msg.phase = 0  # PHASE_IDLE
        
        # 发送goal
        self.get_logger().info(f'Sending move: {from_cells} → {to_cell}')
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
        
        return future
    
    def goal_response_callback(self, future):
        """Goal响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        """Result回调"""
        result = future.result().result
        if result.success:
            self.get_logger().info('✓ Move completed successfully!')
        else:
            self.get_logger().error('✗ Move failed!')
    
    def feedback_callback(self, feedback_msg):
        """Feedback回调"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Progress: {feedback.progress:.1%}')


def main(args=None):
    rclpy.init(args=args)
    node = TestBrain()
    
    # 等待一下让系统初始化
    time.sleep(2)
    
    # 测试案例1: 移动1x1小块从(0,0)到(1,0)
    node.get_logger().info('=== Test 1: Move 1x1 piece from (0,0) to (1,0) ===')
    future1 = node.send_move_action(
        from_cells=[(0, 0)],
        to_cell=(1, 0),
        piece_type=4,  # TYPE_1_1
        color=0        # COLOR_NONE
    )
    
    # 等待第一个动作完成
    rclpy.spin_until_future_complete(node, future1, timeout_sec=30.0)
    time.sleep(2)
    
    # 测试案例2: 移动2x1横块从(0,1)到(2,1)
    node.get_logger().info('=== Test 2: Move 2x1 piece from (0,1-2) to (2,1) ===')
    future2 = node.send_move_action(
        from_cells=[(0, 1), (0, 2)],
        to_cell=(2, 1),
        piece_type=3,  # TYPE_2_1
        color=0
    )
    
    rclpy.spin_until_future_complete(node, future2, timeout_sec=30.0)
    
    node.get_logger().info('All tests completed!')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
