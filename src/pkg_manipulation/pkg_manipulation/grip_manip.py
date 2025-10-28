import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import serial

from klotski_interfaces.action import GripPiece


class GripperActionServer(Node):

    def __init__(self):
        super().__init__('gripper_action_server')
        self._action_server = ActionServer(
            self,
            GripPiece,
            'gripper',
            self.execute_callback)
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)  # Update the baud rate as required

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        angle = 0	

        if (goal_handle.request.grip_action == goal_handle.request.GRIP_OPEN):
            angle = 30
        elif (goal_handle.request.grip_action == goal_handle.request.GRIP_CLOSE):
            angle = 1
        
        self.serial_port.write(f"{angle}\n".encode('utf-8'))

        goal_handle.succeed()
        result = GripPiece.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    gripper_action_server = GripperActionServer()

    rclpy.spin(gripper_action_server)


if __name__ == '__main__':
    main()
