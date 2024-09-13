import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/xy/arm_isaac_sim2real/ros2_ws/install/robot_controller'
