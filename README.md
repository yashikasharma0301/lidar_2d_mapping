# Simultaneous Mapping And Localization using 2D LiDAR in ROS2 

This project demonstrates 2D mapping and robot localization using LiDAR with slam_toolbox in ROS 2. The package enables autonomous mapping of environments and subsequent robot localization within those maps using a differential drive robot in Gazebo simulation. Check out the 2d mapping and localization [here](https://youtu.be/uUfAPKa34kQ?si=_G-T13XOkXAiKzrS).

## System Requirements

- **Ubuntu**: 24.04 LTS
- **ROS 2**: Jazzy Jalapa
- **Gazebo**: Harmonic

## Usage

### Prerequisites

Install required ROS 2 packages:
```bash
sudo apt install ros-jazzy-robot-state-publisher \
                 ros-jazzy-joint-state-publisher \
                 ros-jazzy-xacro \
                 ros-jazzy-teleop-twist-keyboard \
                 ros-jazzy-ros-gz-sim \
                 ros-jazzy-ros-gz-bridge \
                 ros-jazzy-slam-toolbox 
```

### Installation & Setup

1. **Create Workspace and Clone Repository**
```bash
mkdir -p my_ws/src && cd my_ws/src
git clone https://github.com/yashikasharma0301/lidar_2d_mapping.git
```

2. **Build the Workspace**
```bash
cd ..
colcon build
```

3. **Source the Workspace**
```bash
source install/setup.bash
```

## Running the Simulation

### Phase 1: 2D Mapping

1. **Launch Robot with LiDAR in Gazebo**
```bash
ros2 launch lidar_2d_mapping gazebo_spawn.launch.py
```

2. **Start SLAM Mapping**
- In a new terminal:
```bash
cd my_ws
source install/setup.bash
ros2 launch lidar_2d_mapping slam_mapping.launch.py
```
- Wait for a minute and let it load

3. **Create the Map**
- In Gazebo, open the teleop plugin to control the robot
  
  <img width="1028" height="911" alt="image" src="https://github.com/user-attachments/assets/32c04c82-2415-4e2c-94da-1db616f70490" />

- Move the robot around the environment to generate a comprehensive map
- Monitor the mapping progress in RViz
  
  <img width="1841" height="871" alt="image" src="https://github.com/user-attachments/assets/0aa46aac-d0ec-497a-88d4-8c5df3e25e14" />


4. **Save the Map**
- In RViz's "SlamToolboxPlugin" panel, enter the location where you want to save the map infront of the *Serialize Map* button :
  ```
  /home/YOUR_USERNAME/my_ws/src/lidar_2d_mapping/maps/map_serialize
  ```
  Replace `YOUR_USERNAME` with your actual username
- Click on *Serialize Map* to save the generated map
  
  <img width="841" height="97" alt="image" src="https://github.com/user-attachments/assets/2964449b-7a10-46b8-ac0e-118f9e691d9c" />
- Saving the map in *Save Map* allows you to use the map for external environments also whearas to use the map with slam_toolbox, you need to save the serialized map
- Close the mapping terminal once mapping is complete

### Phase 2: Robot Localization

1. **Configure Localization Parameters**
```bash
gedit ~/my_ws/src/lidar_2d_mapping/params/mapper_params_localization.yaml
```
<img width="902" height="131" alt="image" src="https://github.com/user-attachments/assets/13b75bc8-65f0-47f9-8b28-1a52ca160897" />

- Update the `map_file_name` parameter with your saved map location
- Optionally, set your preferred initial pose using the `map_start_pose` parameter
- Save and close the file
- Close all the terminals

2. **Launch Localization**

- In a new terminal rebuild the workspace and launch the gazebo world again:
```bash
cd my_ws
colcon build
source install/setup.bash
ros2 launch lidar_2d_mapping gazebo_spawn.launch.py
```
- In another terminal:
```bash
cd my_ws
source install/setup.bash
ros2 launch lidar_2d_mapping slam_localization.launch.py
```

3. **Test Localization**
- Use the Gazebo teleop plugin to move the robot
- Observe how the robot localizes itself within the previously created map
   
  <img width="1854" height="1168" alt="image" src="https://github.com/user-attachments/assets/45aa7e77-bb9f-4c48-87fa-17c609b15e9f" />

## Credits & References

**Robot Model**: This project uses a URDF model adapted from the TortoiseBot example in the [OSRF ROS Book](https://github.com/osrf/rosbook/blob/master/code/tortoisebot/tortoisebot.urdf). The original model has been modified for ROS 2 Jazzy and Gazebo Harmonic integration with LiDAR sensor capabilities.

**Original Authors**: Open Source Robotics Foundation (OSRF)
