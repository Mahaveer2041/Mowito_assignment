import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mahaveer/Mowito_assignment/ros2_ws/install/camera_publisher'
