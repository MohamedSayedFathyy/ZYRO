# ZYRO
Autonomous Drone System with Digital Twin Integration for Real-Time Inventory and Warehouse Management (Graduation Project, Ain Shams University 2025)
# Smart Warehouse 4.0: Autonomous UAV System for Real-Time Inventory

### 🎯 Overview
An AI-powered drone system integrating RTAB-Map SLAM, LiDAR, RGB-D vision, and Oracle Cloud Digital Twin for automated warehouse management.

### 🧠 Features
- Real-time 3D mapping and localization (RTAB-Map)
- Object detection using YOLOv8 on Jetson Orin Nano
- LiDAR-based obstacle avoidance
- PX4 autonomous flight via MAVROS
- Oracle IoT Cloud data streaming and live dashboard

### 🧩 Components
- Jetson Orin Nano Super Dev Kit  
- Intel RealSense D435i  
- RPLiDAR A2M12  
- Pixhawk 4 Flight Controller

### 🧭 System Architecture
![diagram](docs/methodology_block_diagram.png)

### ⚙️ ROS2 Packages
- `slam_pkg` — RTAB-Map integration  
- `nav_pkg` — Path planning via Nav2  
- `perception_pkg` — YOLOv8 vision node  
- `iot_pkg` — Oracle IoT integration  

### 🧰 Installation
```bash
colcon build
source install/setup.bash
ros2 launch slam_pkg mapping.launch.py
