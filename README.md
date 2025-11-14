# ROS2_3D_Mapping_RTAB_Map
The repository provides a complete simulation and mapping for performing 3D SLAM (Simultaneous Localization and Mapping) using RTAB-Map within the ROS 2 ecosystem. This is designed for researchers, developers, and students who aim to explore autonomous 3D environment perception, mapping, and navigation in both simulated and real-world scenarios.

---
<img width="1080" height="1080" alt="photo-collage png" src="https://github.com/user-attachments/assets/e7df7877-d045-4dba-ba04-ae8f25b11c05" />

---

## ⚙️ Features

- Complete Mobile Robot Simulation with URDF/Xacro model integrated with realistic sensor plugins 
- Sensor Suite Support: RGB-D camera, 2D/3D LiDAR, IMU, and wheel odometry
- Real-time 3D SLAM using RTAB-Map with loop closure and graph optimization
- Gazebo Simulation Environment for physics-based testing and sensor data generation
- RViz Visualization for 3D map, point cloud, TF, odometry, and sensor topics
- RTAB-Map Visualization (rtabmap_viz) to view 3D mapping through point cloud data generated from the 3D LiDAR
- Launch Files for Fast Setup to start simulation and RTAB-Map mapping with a single command
- Map Saving & Loading (.db 3D map database generation for reuse in localization)
- Extensible for research on 3D mapping, perception, autonomous navigation, and robotics prototyping

---

## 🧠 Learning Objectives

By using this package, you will learn how to:

1. **Understand Robot Description in ROS 2**  
   - Explore how URDF/XACRO files define the robot’s physical structure, links, joints, and sensors.  
   - Visualize your robot model and sensor frames in RViz.  

2. **Perform Real-Time 3D Mapping Using RTAB-Map**  
   - Launch RTAB-Map in real-time 3D SLAM mode using sensor data from 3D LiDAR and ICP Odometry. 
   - Move the robot through teleoperation or autonomous exploration to collect spatial information. 
   - Observe 3D map reconstruction and loop-closure optimization in real time in rtabmap_viz.

3. **Visualize 3D Point Cloud Mapping and SLAM Graph**  
   - Use rtabmap_viz to view 3D mapping generated from point cloud data, keyframes, and global pose graph.
   - Display point clouds, camera depth data, TF frames, and odometry in RViz for analysis and debugging.

4. **Save and Reuse 3D Map Data**  
   - Save the environment as a .db (map database) file containing point clouds, graph structure, and metadata.

5. **Configure SLAM Parameters and Sensor Fusion**  
   - Modify RTAB-Map configuration to improve performance, accuracy, and loop-closure reliability.
   - Understand the impact of sensor fusion from IMU + Odometry + LiDAR/RGB-D data.

---

## Installation

### Make Workspace
```bash
mkdir rtab_ws/
```

### Change Workspace
```bash
cd rtab_ws
```

### Make src
```bash
mkdir src/
```

### Change Workspace
```bash
cd src
```

### Clone This Repository
```bash
git clone https://github.com/yashbhaskar/ROS2_3D_Mapping_RTAB_Map.git
```

### Install rtabmap And rtabmap-ros
```bash
sudo apt install ros-humble-rtabmap ros-humble-rtabmap-ros
```

### Change Workspace
```bash
cd ..
```

### Build the Package
```bash
colcon build --packages-select my_bot rtab_3d_mapping
source install/setup.bash
```

---

## 🚀 How to Run

### 1st Terminal : Launch robot
```bash
ros2 launch my_bot gazebo.launch.py
```

### 2nd Terminal : Start The Rtab 3D Mapping
```bash
ros2 launch rtab_3d_mapping rtab.launch.py lidar_topic:=/lidar/points     expected_update_rate:=30.0     voxel_size:=0.05     use_sim_time:=false
```
- change /lidar/points with your 3D lidar topic if you changed

### 3rd Terminal : Move Using The Keyboard
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- **Open Rtabmap_viz:** open rtabmap_viz to view 3D mapping through point cloud data generated from the 3D LiDAR
- **Open Rviz:** open Rviz to view 3D map point cloud data generated from the 3D LiDAR using octamap cloud point plugin

<img width="741" height="460" alt="screenshot-1761829460" src="https://github.com/user-attachments/assets/3e45b184-06dd-4476-8009-30f66f42c500" />
<img width="1270" height="553" alt="r2" src="https://github.com/user-attachments/assets/8d117db9-0679-4b17-9bd9-72f94bdb2750" />


### Save Map

- **Pause Mapping Process:** Go to the Detection menu in rtabmap_viz. Click on Pause (or press Space key). This stops incoming data and freezes the current state for saving.

<img width="1915" height="1048" alt="pause" src="https://github.com/user-attachments/assets/6c05ce52-e3c2-40ab-a158-6fdb08f4a3b4" />


- **Save the 3D Point Cloud Map:** Open the File menu. Select **Export 3D clouds (*.ply, .pcd, .obj) Choose the desired format (.ply recommended for visualization tools like MeshLab/ CloudCompare). Select the location and click Save.

<img width="1915" height="1048" alt="save" src="https://github.com/user-attachments/assets/ed60f118-2641-4b2a-a55c-de0035280f74" />


- **Save RTAB-Map Database File (.db):** From the File menu, choose Save config or Ctrl+S. This saves the full map database including point clouds, poses, graph, and loop closures. You can later load this .db file for localization-only mode.
- **(Optional) Export 2D Occupancy Map:** Go to File → Export 2D grid map (.png, .pgm, .bmp). 

---
## Photos

### Rtabmap_viz :

<img width="741" height="460" alt="rt4" src="https://github.com/user-attachments/assets/445e6016-c8e4-4ddf-a6c1-8d00f454c467" />
<img width="1915" height="1048" alt="rt1" src="https://github.com/user-attachments/assets/f197a5dc-fff6-4914-95eb-e48835b06dfc" />
<img width="1915" height="1048" alt="rt2" src="https://github.com/user-attachments/assets/daf649da-dc8e-45a6-9490-c1eb5975ead5" />
<img width="1851" height="1054" alt="rt3" src="https://github.com/user-attachments/assets/11c53609-46c9-47e1-a197-44ba72154967" />

### Rviz :

<img width="1270" height="553" alt="r1" src="https://github.com/user-attachments/assets/d8dab9b5-519d-4cff-acc6-55a6a6beac36" />
<img width="1294" height="766" alt="r3" src="https://github.com/user-attachments/assets/ed99a870-8f17-462a-85aa-3ab3b81ff461" />
<img width="1270" height="553" alt="r2" src="https://github.com/user-attachments/assets/c51f5ea6-641e-4071-bbec-017c7a2e927d" />
<img width="1294" height="766" alt="r4" src="https://github.com/user-attachments/assets/564c5cd1-1a58-4100-b9bf-0e239ebb9ebc" />

---

## ✉️ Contact

📧 Yash Bhaskar – ybbhaskar19@gmail.com

📌 GitHub: https://github.com/yashbhaskar

