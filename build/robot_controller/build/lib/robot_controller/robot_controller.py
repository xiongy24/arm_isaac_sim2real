import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from math import radians

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angle_1 = 0.0
        self.angle_2 = 0.0  # 新增 joint_2 的角度

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2']  # 添加 joint_2
        msg.position = [radians(self.angle_1), radians(self.angle_2)]  # 两个关节的位置
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.name[0]} = {msg.position[0]} rad, {msg.name[1]} = {msg.position[1]} rad')
        
        # 更新两个关节的角度
        self.angle_1 += 5.0
        self.angle_2 += 5.0
        if self.angle_1 > 90.0:
            self.angle_1 = 0.0
        if self.angle_2 > 90.0:
            self.angle_2 = 0.0

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
