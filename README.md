# ROS and BT Task

# 1. Image Conversion ROS 2 Package

Src contains ROS 2 packages for image processing and camera publishing.
I have made the packages using ubuntu 22.04 and ROS Humble. Kindly check for dependency issues.
---

## 📁 How to Install
```bash
# Clone the repository
git clone https://github.com/Mahaveer2041/Mowito_assignment.git

# Move to the repository directory
cd Mowito_assignment

# Build selected packages
colcon build --packages-select image_conversion camera_publisher

# Source the workspace
source install/setup.bash

# Basic launch (default parameters)
ros2 launch image_conversion image_processing.launch.py

# Launch with custom topics
ros2 launch image_conversion image_processing.launch.py input_topic:=/camera/image output_topic:=/processed_image

# Service call to switch to black & white mode
ros2 service call /set_conversion_mode std_srvs/srv/SetBool "{data: true}"

# Service call to switch to color mode
ros2 service call /set_conversion_mode std_srvs/srv/SetBool "{data: false}"

# Run individual image conversion node
ros2 run image_conversion image_conversion_node
