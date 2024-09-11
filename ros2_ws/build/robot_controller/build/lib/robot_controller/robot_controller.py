import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from math import radians, sin

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(JointState, 'joint_commands', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.t = 0.0

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        # 使用正弦函数生成周期性的关节角度命令
        msg.position = [
            radians(45 * sin(self.t)),
            radians(30 * sin(self.t * 0.5)),
            radians(15 * sin(self.t * 0.25)),
            radians(10 * sin(self.t * 0.1)),
            radians(5 * sin(self.t * 0.05)),
            radians(2 * sin(self.t * 0.025))
        ]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.name[0]} = {msg.position[0]} rad')
        self.t += 0.1

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
