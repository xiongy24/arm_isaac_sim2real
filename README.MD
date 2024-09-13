# 使用Isaac Sim 实现机械臂的Sim2Real

这个项目分为两个部分：

  第一部分完成机械臂的运动规划，并在Isaac Sim中进行验证

  第二部分实现真实机械臂与Isaac Sim中机械臂的运动同步

## 一、使用ROS2 控制 Isaac Sim 中的机械臂

本节展示了如何在 Ubuntu 环境下，通过 ROS2 控制 Isaac Sim 中的机械臂。我们使用 ROS2 发布关节角度命令，并通过 Isaac Sim 的 ROS2 桥接功能来控制机械臂的运动。

## 先决条件

- Ubuntu 20.04 或更高版本
- ROS2 Humble
- NVIDIA Isaac Sim 2022.1 或更高版本
- Python 3.8 或更高版本

## 安装步骤

1. **安装 ROS2 Humble**
   按照 [ROS2 安装指南](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) 进行安装。
2. **安装 NVIDIA Isaac Sim**
   从 [NVIDIA Isaac Sim 下载页面](https://developer.nvidia.com/isaac-sim) 下载并安装。
3. **创建 ROS2 工作空间：**

   ```bash
   mkdir -p ros2_ws/src
   cd ros2_ws
   colcon build
   source install/setup.bash
   ```
4. **创建 ROS2 包：**

   ```bash
   cd src
   ros2 pkg create --build-type ament_python robot_controller
   cd robot_controller/robot_controller
   ```
5. **创建 `robot_controller.py` 文件并添加以下内容：**

   ```python
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import JointState
   from math import radians, sin

   class RobotController(Node):
       def __init__(self):
           super().__init__('robot_controller')
           self.publisher_ = self.create_publisher(JointState, 'joint_commands', 10)
           timer_period = 0.1 # seconds
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
   ```
6. **更新 `setup.py` 文件：**

   ```python
   from setuptools import setup

   package_name = 'robot_controller'

   setup(
       name=package_name,
       version='0.0.0',
       packages=[package_name],
       data_files=[
           ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='your_name',
       maintainer_email='your_email@example.com',
       description='ROS2 package for controlling a robot arm in Isaac Sim',
       license='Apache License 2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'robot_controller = robot_controller.robot_controller:main'
           ],
       },
   )
   ```
7. **构建包：**

   ```bash
   cd ../../..
   colcon build --packages-select robot_controller
   source install/setup.bash
   ```

## Isaac Sim 配置

1. 打开 Isaac Sim 并创建一个新场景。
2. 导入您的机械臂模型（确保模型有正确的关节配置）。
3. 在 Isaac Sim 中启用 ROS2 桥接扩展：
   - 菜单栏中选择 `Window -> Extensions`
   - 在扩展管理器中搜索并启用 `omni.isaac.ros2_bridge`
4. 使用 Isaac Sim 的 Action Graph 创建以下节点：
   - ROS2 订阅者节点，订阅 `/joint_commands` 话题
   - ROS2 发布者节点，发布 `/joint_states` 话题
   - ArticulationController 节点，用于控制机械臂的每个关节
5. 连接这些节点，使得：
   - 从 `/joint_commands` 接收的命令被应用到 ArticulationController
   - ArticulationController 的状态被发布到 `/joint_states`

## 运行项目

1. 启动 ROS2 节点：

   ```bash
   ros2 run robot_controller robot_controller
   ```

## 调试

- 使用 `ros2 topic echo /joint_states` 监视关节状态
- 使用 `ros2 topic echo /joint_commands` 确认命令正在被发布

## 注意事项

- 确保机械臂模型的关节名称与代码中的名称匹配
- 调整代码中的运动参数以适应您的特定机械臂模型
- 注意设置适当的运动限制以避免碰撞或超出关节限制

## 故障排除

如果遇到问题：

1. 确保所有依赖项都已正确安装
2. 检查 ROS2 和 Isaac Sim 的版本兼容性
3. 验证 Isaac Sim 中的 ROS2 桥接配置是否正确
4. 检查机械臂模型的关节配置


## 二、Arduino 舵机控制集成

本节介绍如何将 ROS2 控制指令同时发送给 Isaac Sim 中的机械臂和通过 Arduino 控制的实际舵机。

### 先决条件

- Arduino 板（如 Arduino Uno）
- 6 个舵机
- USB 线缆连接 Arduino 和电脑
- VSCode 和 PlatformIO 扩展

### Arduino 程序

1. 在 VSCode 中安装 PlatformIO 扩展。
2. 创建一个新的 PlatformIO 项目，选择你的 Arduino 板型。
3. 在项目的 `src` 目录下创建 `main.cpp` 文件，添加以下代码：

   ```cpp
   #include <Arduino.h>
   #include <Servo.h>

   Servo servos[6];
   const int servoPins[6] = {2, 3, 4, 5, 6, 7};
   const int initialPositions[6] = {90, 90, 90, 90, 90, 90};  // 初始位置，可以根据需要调整

   void setup() {
     Serial.begin(115200);

     for (int i = 0; i < 6; i++) {
       servos[i].attach(servoPins[i]);
       servos[i].write(initialPositions[i]);  // 将舵机移动到初始位置
       delay(500);  // 给予足够的时间移动到初始位置
     }

     Serial.println("Arduino initialized and ready.");
   }

   void loop() {
     if (Serial.available() >= 12) {  // 6 joints * 2 bytes per angle
       for (int i = 0; i < 6; i++) {
         int angle = Serial.read() << 8 | Serial.read();  // Read 2 bytes and combine them
         angle = constrain(angle, 0, 180);  // Ensure angle is within 0-180 range
         servos[i].write(angle);
       }
     }
   }
   ```
4. 使用 PlatformIO 编译并上传代码到 Arduino。

### 修改 ROS2 节点

1. 更新 `robot_controller.py` 文件，添加 Arduino 通信功能：

   ```python
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
               radians(45 * sin(self.t)),  # 关节1：-45度到45度
               radians(30 * sin(self.t * 0.8)),  # 关节2：-30度到30度
               radians(20 * sin(self.t * 0.6)),  # 关节3：-20度到20度
               radians(15 * sin(self.t * 0.4)),  # 关节4：-15度到15度
               radians(10 * sin(self.t * 0.2)),  # 关节5：-10度到10度
               radians(5 * sin(self.t * 0.1))  # 关节6：-5度到5度
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
   ```
2. 更新 `setup.py` 文件，添加 `pyserial` 依赖：

   ```python
   ...  # 其他代码保持不变
   install_requires=['setuptools', 'pyserial'],
   ...  # 其他代码保持不变
   ```
3. 重新构建 ROS2 包：

   ```bash
   cd ros2_ws
   colcon build --packages-select robot_controller
   source install/setup.bash
   ```

### 运行集成系统

1. 确保 Arduino 已连接到电脑，并且舵机正确连接到 Arduino 的引脚 2-7。
2. 运行 ROS2 节点：

   ```bash
   ros2 run robot_controller robot_controller
   ```
3. 在另一个终端中，启动机器人：

   ```bash
   ros2 service call /start_robot std_srvs/srv/Empty
   ```

现在，当你运行 ROS2 节点时，它会同时向 Isaac Sim 中的机械臂和通过 Arduino 控制的实际舵机发送控制命令。

**注意：** 确保 Arduino 的串口设备名称正确（代码中使用的是 `/dev/ttyACM0`）。如果不同，请相应地更改代码中的端口名称。

下一步

- 实现更复杂的运动规划算法
- 添加碰撞检测和避障功能
- 集成其他传感器（如相机或力传感器）
- 实现基于任务的控制逻辑
