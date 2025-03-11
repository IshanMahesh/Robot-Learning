# Robot-Learning
ENPM692


## **Installation Steps**  

### **Prerequisites:**  
- This project is developed using **ROS2 Humble** and **TurtleBot3**.   

### **Setup Instructions:**  

1. Install **ROS2 Humble**:  
   ```bash
   sudo apt install ros-humble-desktop
   ```  
2. Load the ROS environment by sourcing the setup script:  
   ```bash
   source /opt/ros/humble/setup.bash
   ```  
3. Install the TurtleBot3 packages:  
   ```bash
   sudo apt install ros-humble-turtlebot3*
   ```  
4. Resolve dependencies using **rosdep**:  
   ```bash
   rosdep install --from-paths src --ignore-src -y
   ```  
5. Build the workspace:  
   ```bash
   colcon build --symlink-install
   ```  

---

## **Part 1: Keyboard Teleoperation**  

### **Terminal 1: Launching the Simulation**  
1. Navigate to your **TurtleBot3 workspace** and source the local setup:  
   ```bash
   cd ~/turtlebot3_ws
   source install/setup.bash
   ```  
2. Set the **ROS domain ID**:  
   ```bash
   export ROS_DOMAIN_ID=1
   ```  
3. Source the ROS2 setup script:  
   ```bash
   source /opt/ros/humble/setup.bash
   ```  
4. Define the TurtleBot3 model:  
   ```bash
   export TURTLEBOT3_MODEL=burger
   ```  
5. Launch the **Gazebo simulation environment**:  
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```  


### **Terminal 2: Controlling the Robot**  
1. Go to the TurtleBot3 workspace and source the local setup again:  
   ```bash
   cd ~/turtlebot3_ws
   source install/setup.bash
   ```  
2. Set the **ROS domain ID**:  
   ```bash
   export ROS_DOMAIN_ID=1
   ```  
3. Source the ROS2 setup script:  
   ```bash
   source /opt/ros/humble/setup.bash
   ```  
4. Specify the TurtleBot3 model:  
   ```bash
   export TURTLEBOT3_MODEL=burger
   ```  
5. Launch the **keyboard teleoperation node** to control the robot:  
   ```bash
   ros2 run turtlebot3_teleop teleop_keyboard
   ```  


---

## **Part 2: BUG0-Based Obstacle Avoidance**  

### **Sensor Used:**  
- **LaserScan** (LIDAR-based obstacle detection)  

### **Terminal 1: Launching the Simulation**  
1. Go to the TurtleBot3 workspace and source the local setup:  
   ```bash
   cd ~/turtlebot3_ws
   source install/setup.bash
   ```  
2. Set the **ROS domain ID**:  
   ```bash
   export ROS_DOMAIN_ID=1
   ```  
3. Source the ROS2 setup script:  
   ```bash
   source /opt/ros/humble/setup.bash
   ```  
4. Define the TurtleBot3 model:  
   ```bash
   export TURTLEBOT3_MODEL=burger
   ```  
5. Start the **Gazebo world** for simulation:  
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```  
### **Terminal 2: Running BUG0 Navigation**  
1. Navigate to the TurtleBot3 workspace and source the local setup:  
   ```bash
   cd ~/turtlebot3_ws
   source install/setup.bash
   ```  
2. Set the **ROS domain ID**:  
   ```bash
   export ROS_DOMAIN_ID=1
   ```  
3. Source the ROS2 setup script:  
   ```bash
   source /opt/ros/humble/setup.bash
   ```  
4. Define the TurtleBot3 model:  
   ```bash
   export TURTLEBOT3_MODEL=burger
   ```  
5. Build the **custom navigation package**:  
   ```bash
   colcon build --packages-select my_navigation_package
   ```  
6. Source the installed package:  
   ```bash
   source install/local_setup.bash
   ```  
7. Run the **BUG0 obstacle avoidance node**:  
   ```bash
   ros2 run my_navigation_package bug0_node
   ```    

---

## **Additional Files Included**  
- **MP4 Demonstrations:**  
  - **Part 1:** Keyboard-based teleoperation video  
  - **Part 2:** BUG0 obstacle avoidance demo  

- **Project Files:**  
  - `my_navigation_package` (custom package for obstacle avoidance)  
  - `turtlebot3` (TurtleBot3-related dependencies)  
