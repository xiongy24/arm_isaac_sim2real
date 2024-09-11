# ROS2 控制 Isaac Sim 中的机械臂运动

## 项目介绍

本项目展示了如何在 Ubuntu 环境下，通过 ROS2 控制 Isaac Sim 中的机械臂。我们使用 ROS2 发布关节角度命令，并通过 Isaac Sim 的 ROS2 桥接功能来控制机械臂的运动。

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

## 下一步

- 实现更复杂的运动规划算法
- 添加碰撞检测和避障功能
- 集成其他传感器（如相机或力传感器）
- 实现基于任务的控制逻辑

## 故障排除

如果遇到问题：

1. 确保所有依赖项都已正确安装
2. 检查 ROS2 和 Isaac Sim 的版本兼容性
3. 验证 Isaac Sim 中的 ROS2 桥接配置是否正确
4. 检查机械臂模型的关节配置

## 贡献

欢迎提交问题报告和拉取请求。

## 许可证

本项目采用 Apache License 2.0 许可证。

---
