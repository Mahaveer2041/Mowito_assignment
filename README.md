# ROS and BT Task

# 1. Image Conversion ROS 2 Package

1. **camera_publisher**:  
   Takes frames from your webcam via OpenCV and publishes on the `/image_raw` topic.  
   On Ubuntu, the camera sometimes doesn’t work due to permissions. Try running:  
   ```bash
   sudo usermod -aG video $USER
2. **Image conversion**:  
   Does the task of conversion. Service call have to be made to switch to greyscale.
   
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

##You can check the streams in rviz2 visulation windows.
rivz2

# Run individual image conversion node
ros2 run image_conversion image_conversion_node
