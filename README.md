# ROS and BT Task

# 1. Image Conversion ROS 2 Package

Src contains ROS 2 packages for image processing and camera publishing.
I have made the packages using ubuntu 22.04 and ROS Humble. Kindly check for dependency issues.
---

## 📁 How to Install

Initialise your ROS(source install/setup.bash or if you have in bashrc no need)
```bash
git clone https://github.com/Mahaveer2041/Mowito_assignment.git
cd mowito_assignmet
colcon build --packages-select image_conversion camera_publisher
source install/setup.bash

Basic Launch
```bash
ros2 launch image_conversion image_processing.launch.py
Launch with custom topics:
```bash
ros2 launch image_conversion image_processing.launch.py input_topic:=/camera/image output_topic:=/processed_image

Service call
```bash
ros2 service call /set_conversion_mode std_srvs/srv/SetBool "{data: true}"    /for blackwhite
ros2 service call /set_conversion_mode std_srvs/srv/SetBool "{data: false}"   //for color

Individual node running
ros2 run image_conversion image_conversion_node

## Quick Start
1. Clone repo: [`git clone https://github.com/your/repo.git`](https://github.com/Mahaveer2041/Mowito_assignment.git)
2. Build: ```colcon build --packages-select image_conversion```
3. Run: ```ros2 launch image_conversion image_processing.launch.py```

