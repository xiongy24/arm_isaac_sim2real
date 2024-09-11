import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/xy/ros2_isaac_communicate/ros2_ws/install/robot_controller'
