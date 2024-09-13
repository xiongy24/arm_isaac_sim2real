import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from math import radians, sin
import serial
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(JointState, 'joint_command', 10)
        self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)  # 等待 Arduino 重置和初始化
        self.wait_for_arduino_init()
        
        self.create_service(Empty, 'start_robot', self.start_robot_callback)
        
        self.is_running = False
        timer_period = 0.05  # 20 Hz update rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.t = 0.0

    def wait_for_arduino_init(self):
        while True:
            if self.arduino.in_waiting:
                message = self.arduino.readline().decode().strip()
                if message == "Arduino initialized and ready.":
                    self.get_logger().info('Arduino initialized and ready.')
                    break
            time.sleep(0.1)

    def start_robot_callback(self, request, response):
        self.is_running = True
        self.get_logger().info('Robot started and ready to send commands')
        return response

    def timer_callback(self):
        if not self.is_running:
            return  # 如果未启动，不发送命令
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        # 所有6个关节执行简单的往复运动
        msg.position = [
            radians(45 * sin(self.t)),        # 关节1：-45度到45度
            radians(30 * sin(self.t * 0.8)),  # 关节2：-30度到30度
            radians(20 * sin(self.t * 0.6)),  # 关节3：-20度到20度
            radians(15 * sin(self.t * 0.4)),  # 关节4：-15度到15度
            radians(10 * sin(self.t * 0.2)),  # 关节5：-10度到10度
            radians(5 * sin(self.t * 0.1))    # 关节6：-5度到5度
        ]
        
        self.publisher_.publish(msg)
        self.send_to_arduino(msg.position)
        self.t += 0.05

    def send_to_arduino(self, positions):
        try:
            for angle in positions:
                angle_deg = int(min(max(angle * 180 / 3.14159 + 90, 0), 180))
                self.arduino.write(angle_deg.to_bytes(2, byteorder='big'))
            self.arduino.flush()
        except serial.SerialException as e:
            self.get_logger().error(f'Error sending data to Arduino: {e}')

    def __del__(self):
        self.arduino.close()

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
