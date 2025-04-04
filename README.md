# 🧠 Agibot – Scout Mini + YDLIDAR + Hector SLAM

![Agibot Overview](imgs/Agibot_overview.png)
![Mapping Demo](imgs/mapping_demo.png)

---

## 📌 Introduction
This project combines:
- **Agilex Scout Mini** with CAN communication
- **YDLIDAR** for 2D LIDAR scanning
- **Hector SLAM** using LIDAR + odometry from Scout Mini
- Teleoperation using keyboard or joystick
- Remote control via web interface
- SLAM-based map saving and autonomous navigation

---

## ⚙️ Installation & Dependencies

### 1. ROS & Required Packages
```bash
sudo apt update
sudo apt install -y build-essential ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-teleop-twist-joy \
  ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-imu-filter-madgwick \
  ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-map-server ros-$ROS_DISTRO-amcl \
  ros-$ROS_DISTRO-navigation ros-$ROS_DISTRO-dwa-local-planner \
  ros-$ROS_DISTRO-robot-localization ros-$ROS_DISTRO-pointcloud-to-laserscan \
  ros-$ROS_DISTRO-tf2-tools
```

### 2. YDLIDAR SDK
```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build && cd build
cmake ..
make
sudo make install
```

---

## 🚗 Scout Mini Setup

### Configure & Start CAN:
```bash
rosrun scout_bringup setup_can2usb.bash 1  # First time
./src/Agibot_ros/start_can.sh 1           # Start CAN bus
roslaunch scout_bringup lidar.launch can:=0
```

---

## 📡 YDLIDAR Launch

Enable LIDAR:
```bash
export SCOUT_YDLIDAR_ENABLED=1
roslaunch scout_bringup lidar.launch can:=1 ydlidar_enabled:=true
```

---

## 🧭 Hector SLAM

### Launch with proper topics:
Edit config to match:
- `/scan`
- `/odom`

Then launch:
```bash
roslaunch hector_slam_launch tutorial.launch
```

---

## 🎮 Robot Control

### Joystick:
```bash
roslaunch scout_control teleop_joystick.launch
```

### Keyboard:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _repeat_rate:=50.0
```

---

## 🎥 Data Recording

Record important topics:
```bash
rosbag record -O mapping.bag /scan /odom /tf /cmd_vel
```

---

## 🗺️ Map Saving
```bash
rosrun map_server map_saver -f my_map
```

---

## 🤖 Autonomous Navigation

Launch AMCL with saved map:
```bash
roslaunch scout_navigation amcl_navigation.launch map_file:=/path/to/my_map.yaml
```

Use `RViz` to set the initial pose & navigation goal.

---

## 🙏 Acknowledgements
- Based on: `scout_ros`, `hector_slam`

---

## 📷 Placeholders for Additional Images
- ![Robot Setup](imgs/robot_setup.png)
- ![RViz Mapping](imgs/rviz_mapping.png)
- ![SLAM](imgs/slam.png)
