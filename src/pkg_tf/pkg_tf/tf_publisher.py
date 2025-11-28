#!/usr/bin/env python3
"""
Board TF Publisher - 发布棋盘坐标系TF
从相机坐标系的marker数据计算棋盘位姿，并发布到base_link
"""

from klotski_interfaces.msg._board_state import BoardState
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import Float32MultiArray
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R


class BoardTFPublisher(Node):
    def __init__(self):
        super().__init__('board_tf_publisher')
        
        # 声明参数
        self.declare_parameter('board_width', 0.20)
        self.declare_parameter('board_length', 0.25)
        self.declare_parameter('cell_size', 0.05)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('board_frame', 'board_frame')
        
        # 获取参数
        self.board_width = self.get_parameter('board_width').value
        self.board_length = self.get_parameter('board_length').value
        self.cell_size = self.get_parameter('cell_size').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.board_frame = self.get_parameter('board_frame').value
        
        # TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 发布棋盘位姿
        # self.board_pose_pub = self.create_publisher(
        #     PoseStamped,
        #     '/board_pose',
        #     10
        # )
        self.board_pose_pub = self.create_publisher(
            PoseStamped,
            '/board_pose',
            10
        )
        
        # 订阅marker数据
        self.marker_sub = self.create_subscription(
            BoardState,
            '/board_state',
            self.marker_callback,
            10
        )
        
        self.get_logger().info(f'Board TF Publisher initialized')
        self.get_logger().info(f'Board size: {self.board_width}x{self.board_length}m')
        self.get_logger().info(f'Waiting for marker data on /board_markers_camera...')
    
    # def marker_callback(self, msg: BoardState):
    #     """处理marker检测数据"""
    #     if len(msg.marker_pose) != 4:  # 4 markers
    #         self.get_logger().error(
    #             f'Expected 15 values (4 markers), got {len(msg.data)}'
    #         )
    #         return
        
    #     try:
    #         # 解析marker数据
    #         markers_camera = np.array(msg.data[:12]).reshape(4, 3)
    #         center_camera = np.array(msg.data[12:15])
            
    #         # 计算棋盘位姿
    #         board_pose_camera = self.estimate_board_pose(markers_camera, center_camera)
            
    #         # 转换到base_link
    #         board_pose_base = self.transform_to_base_link(board_pose_camera)
            
    #         if board_pose_base is not None:
    #             # 发布TF
    #             self.publish_board_tf(board_pose_base)
                
    #             # 发布位姿消息
    #             self.publish_board_pose(board_pose_base)
                
    #             # 打印信息
    #             self.log_board_info(board_pose_base)
                
    #     except Exception as e:
    #         self.get_logger().error(f'Error processing markers: {e}')
    def marker_callback(self, msg: BoardState):
        """处理BoardState消息"""
        if len(msg.marker_pose) != 4:
            self.get_logger().error(
                f'Expected 4 markers, got {len(msg.marker_pose)}'
            )
            return
        
        try:
            # 提取4个marker的3D位置
            markers_camera = np.zeros((4, 3))
            camera_frame = msg.marker_pose[0].header.frame_id
            
            for i, marker_stamped in enumerate(msg.marker_pose):
                markers_camera[i, 0] = marker_stamped.pose.position.x
                markers_camera[i, 1] = marker_stamped.pose.position.y
                markers_camera[i, 2] = marker_stamped.pose.position.z
            
            # 使用4个marker的质心作为棋盘中心
            center_camera = markers_camera.mean(axis=0)
            
            # 更新camera_frame
            if camera_frame:
                self.camera_frame = camera_frame
            
            # 更新棋盘规格
            if msg.board.spec.cell_size_m > 0:
                self.cell_size = msg.board.spec.cell_size_m
                self.board_width = msg.board.spec.cols * self.cell_size
                self.board_length = msg.board.spec.rows * self.cell_size
            
            # 计算棋盘位姿
            board_pose_camera = self.estimate_board_pose(markers_camera, center_camera)
            
            # 转换到base_link
            board_pose_base = self.transform_to_base_link(board_pose_camera)
            
            if board_pose_base is not None:
                self.publish_board_tf(board_pose_base)
                self.publish_board_pose(board_pose_base)
                self.log_board_info(board_pose_base)
                
        except Exception as e:
            self.get_logger().error(f'Error processing markers: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        
    def estimate_board_pose(self, markers_camera, center_camera):
        """从marker估计棋盘位姿（使用Kabsch算法）"""
        
        markers_board = np.array([
            [-self.board_width/2,  self.board_length/2, 0],  # 0: top-left
            [ self.board_width/2,  self.board_length/2, 0],  # 1: top-right
            [-self.board_width/2, -self.board_length/2, 0],  # 2: bottom-left  ← 修正
            [ self.board_width/2, -self.board_length/2, 0]   # 3: bottom-right ← 修正
        ])

        
        # 计算质心
        centroid_camera = markers_camera.mean(axis=0)
        centroid_board = markers_board.mean(axis=0)
        
        # 去中心化
        markers_camera_centered = markers_camera - centroid_camera
        markers_board_centered = markers_board - centroid_board
        
        # SVD求解旋转矩阵（Kabsch算法）
        H = markers_board_centered.T @ markers_camera_centered
        U, S, Vt = np.linalg.svd(H)
        rotation_matrix = Vt.T @ U.T
        
        # 确保是右手坐标系
        if np.linalg.det(rotation_matrix) < 0:
            Vt[-1, :] *= -1
            rotation_matrix = Vt.T @ U.T
        
        # # 使用检测到的中心点作为位置
        # translation = center_camera
        translation = centroid_camera - rotation_matrix @ centroid_board
        # # 转换为四元数
        # rotation = R.from_matrix(rotation_matrix)
        # quaternion = rotation.as_quat()  # [x, y, z, w]
        
        # return {
        #     'position': translation,
        #     'quaternion': quaternion
        # }
        self.get_logger().info(
            f'Board pose in camera:\n'
            f'  translation: {translation}\n'
            f'  centroid_camera: {centroid_camera}\n'
            f'  center_camera: {center_camera}'
        )
        rotation = R.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()
        return {'position': translation, 'quaternion': quaternion}
    

    def transform_to_base_link(self, board_pose_camera):
        """将相机坐标系下的位姿转换到base_link"""
        try:
            # 检查TF是否可用
            if not self.tf_buffer.can_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            ):
                self.get_logger().warn('TF from camera to base_link not available yet')
                return None
            
            # 创建PoseStamped
            pose_camera = PoseStamped()
            pose_camera.header.frame_id = self.camera_frame
            pose_camera.header.stamp = self.get_clock().now().to_msg()
            
            pose_camera.pose.position.x = float(board_pose_camera['position'][0])
            pose_camera.pose.position.y = float(board_pose_camera['position'][1])
            pose_camera.pose.position.z = float(board_pose_camera['position'][2])
            
            pose_camera.pose.orientation.x = float(board_pose_camera['quaternion'][0])
            pose_camera.pose.orientation.y = float(board_pose_camera['quaternion'][1])
            pose_camera.pose.orientation.z = float(board_pose_camera['quaternion'][2])
            pose_camera.pose.orientation.w = float(board_pose_camera['quaternion'][3])
            
            # 获取变换
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time()
            )
            
            # 执行变换
            pose_base = self.do_transform_pose(pose_camera, transform)

            # 添加调试
            self.get_logger().info(
                f'Before transform (camera): ({pose_camera.pose.position.x:.3f}, '
                f'{pose_camera.pose.position.y:.3f}, {pose_camera.pose.position.z:.3f})'
            )
            self.get_logger().info(
                f'After transform (base): ({pose_base.pose.position.x:.3f}, '
                f'{pose_base.pose.position.y:.3f}, {pose_base.pose.position.z:.3f})'
            )
            
            return pose_base
            
        except Exception as e:
            self.get_logger().error(f'Transform error: {e}')
            return None
    
    def do_transform_pose(self, pose_stamped, transform):
        """手动执行位姿变换"""
        # 提取变换参数
        t = transform.transform.translation
        q = transform.transform.rotation
        
        # 构建变换矩阵
        trans_matrix = np.array([t.x, t.y, t.z])
        rot_matrix = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        
        # 原始位姿
        orig_pos = np.array([
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z
        ])
        orig_quat = [
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w
        ]
        orig_rot = R.from_quat(orig_quat).as_matrix()
        
        # 变换位置和旋转
        new_pos = rot_matrix @ orig_pos + trans_matrix
        new_rot = rot_matrix @ orig_rot
        new_quat = R.from_matrix(new_rot).as_quat()
        
        # 创建新位姿
        new_pose = PoseStamped()
        new_pose.header.frame_id = self.base_frame
        new_pose.header.stamp = self.get_clock().now().to_msg()
        new_pose.pose.position.x = float(new_pos[0])
        new_pose.pose.position.y = float(new_pos[1])
        new_pose.pose.position.z = float(new_pos[2])
        new_pose.pose.orientation.x = float(new_quat[0])
        new_pose.pose.orientation.y = float(new_quat[1])
        new_pose.pose.orientation.z = float(new_quat[2])
        new_pose.pose.orientation.w = float(new_quat[3])
        
        return new_pose
    
    def publish_board_tf(self, board_pose_base):
        """发布board_frame的TF"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.board_frame
        
        t.transform.translation.x = board_pose_base.pose.position.x
        t.transform.translation.y = board_pose_base.pose.position.y
        t.transform.translation.z = board_pose_base.pose.position.z
        
        t.transform.rotation.x = board_pose_base.pose.orientation.x
        t.transform.rotation.y = board_pose_base.pose.orientation.y
        t.transform.rotation.z = board_pose_base.pose.orientation.z
        t.transform.rotation.w = board_pose_base.pose.orientation.w
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_board_pose(self, board_pose_base):
        """发布棋盘位姿消息"""
        self.board_pose_pub.publish(board_pose_base)
    
    def log_board_info(self, board_pose_base):
        """打印棋盘位姿信息"""
        pos = board_pose_base.pose.position
        quat = board_pose_base.pose.orientation
        
        # 计算欧拉角
        r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        euler = r.as_euler('xyz', degrees=True)
        
        self.get_logger().info(
            f'Board in {self.base_frame}:\n'
            f'  Pos: x={pos.x:.4f}, y={pos.y:.4f}, z={pos.z:.4f}\n'
            f'  Rot: roll={euler[0]:.2f}°, pitch={euler[1]:.2f}°, yaw={euler[2]:.2f}°',
            throttle_duration_sec=1.0  # 每秒最多打印一次
        )


def main(args=None):
    rclpy.init(args=args)
    node = BoardTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
