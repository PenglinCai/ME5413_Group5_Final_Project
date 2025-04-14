# ME5413_Final_Project

NUS ME5413 Autonomous Mobile Robotics Final Project - Group5

![Ubuntu 20.04](https://img.shields.io/badge/OS-Ubuntu_20.04-informational?style=flat&logo=ubuntu&logoColor=white&color=2bbc8a)
![ROS Noetic](https://img.shields.io/badge/Tools-ROS_Noetic-informational?style=flat&logo=ROS&logoColor=white&color=2bbc8a)
![C++](https://img.shields.io/badge/Code-C++-informational?style=flat&logo=c%2B%2B&logoColor=white&color=2bbc8a)
![Python](https://img.shields.io/badge/Code-Python-informational?style=flat&logo=Python&logoColor=white&color=2bbc8a)

![cover_image](src/me5413_world/media/gz_world.png)
## Tasks Description

### 1. Map the environment

* You may use any SLAM algorithm you like, any type:
  * 2D LiDAR
  * 3D LiDAR
  * Vision
  * Multi-sensor
* Verify your SLAM accuracy by comparing your odometry with the published `/gazebo/ground_truth/state` topic (`nav_msgs::Odometry`), which contains the gournd truth odometry of the robot.
* You may want to use tools like [EVO](https://github.com/MichaelGrupp/evo) to quantitatively evaluate the performance of your SLAM algorithm.

### 2. Using your own map, navigate your robot

* We have provided you a GUI in RVIZ that allows you to click and generate/clear the random objects in the gazebo world:
  
  ![rviz_panel_image](src/me5413_world/media/control_panel.png)

* From the starting point, move to one of the four given destination boxes at the end of the map:
  * Count the number of occurance of each type of box (e.g. box 1, 2, 3, 4, the box numbers are randomly generated)
  * Cross the bridge (the location of the bridge is randomly generated)
  * Unlock the blockade on the bridge by publishing a `true` message (`std_msgs/Bool`) to the `/cmd_open_bridge` topic
  * Dock at the destination box with the least number of occurance
## Dependencies

* System Requirements:
  * Ubuntu 20.04 (18.04 not yet tested)
  * ROS Noetic (Melodic not yet tested)
  * C++11 and above
  * CMake: 3.0.2 and above
* This repo depends on the following standard ROS pkgs:
  * `roscpp`
  * `rospy`
  * `rviz`
  * `std_msgs`
  * `nav_msgs`
  * `geometry_msgs`
  * `visualization_msgs`
  * `tf2`
  * `tf2_ros`
  * `tf2_geometry_msgs`
  * `pluginlib`
  * `map_server`
  * `gazebo_ros`
  * `jsk_rviz_plugins`
  * `jackal_gazebo`
  * `jackal_navigation`
  * `velodyne_simulator`
  * `teleop_twist_keyboard`
* And this [gazebo_model](https://github.com/osrf/gazebo_models) repositiory

## Installation

This repo is a ros workspace, containing three rospkgs:

* `interactive_tools` are customized tools to interact with gazebo and your robot
* `jackal_description` contains the modified jackal robot model descriptions
* `me5413_world` the main pkg containing the gazebo world, and the launch files

You can fork this repo to work on yourself:

```bash
# Clone your own fork of this repo (assuming home here `~/`)
cd
git clone https://github.com/<YOUR_GITHUB_USERNAME>/ME5413_Group5_Final_Project.git
cd ME5413_Group5_Final_Project

# Install all dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
catkin_make
# Source 
source devel/setup.bash
```

To properly load the gazebo world, you will need to have the necessary model files in the `~/.gazebo/models/` directory.

There are two sources of models needed:

* [Gazebo official models](https://github.com/osrf/gazebo_models)
  
  ```bash
  # Create the destination directory
  cd
  mkdir -p .gazebo/models

  # Clone the official gazebo models repo (assuming home here `~/`)
  git clone https://github.com/osrf/gazebo_models.git

  # Copy the models into the `~/.gazebo/models` directory
  cp -r ~/gazebo_models/* ~/.gazebo/models
  ```

* [Our customized models](https://github.com/NUS-Advanced-Robotics-Centre/ME5413_Final_Project/tree/main/src/me5413_world/models)

  ```bash
  # Copy the customized models into the `~/.gazebo/models` directory
  cp -r ~/ME5413_Group5_Final_Project/src/me5413_world/models/* ~/.gazebo/models
  ```

## How to complete the tasks

# 0. Initializing Gazebo World

This command will launch the gazebo with the project world

```bash
# Launch Gazebo World together with our robot
roslaunch me5413_world world.launch
```

# 1. Manual Control

If you wish to explore the gazebo world a bit, we provide you a way to manually control the robot around:

```bash
# Only launch the robot keyboard teleop control
roslaunch me5413_world manual.launch
```

**Note:** This robot keyboard teleop control is also included in all other launch files, so you don't need to launch this when you do mapping or navigation.

![rviz_manual_image](src/me5413_world/media/rviz_manual.png)

# 2. Mapping

## ME5413 Final SLAM
The ROS package '`final_slam`' contains SLAM algorithms utilized in the ME5413 Final Project by Group 5.

## Cartographer 
Please follow official documentation to install Cartographer:
https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html

## How to Use This Package:

1. Launch ME5413 world and Cartographer SLAM algorithm.
    ```shell
    roslaunch me5413_world world.launch
    roslaunch final_slam mapping_carto_2d.launch
    ```

2. (For evaluation, optional) Record the trajectory data:
    ```shell
    roscd final_slam/scripts
    chmod +x record_slam_data_ros1.sh
    ./record_slam_data_ros1.sh
    ```
    
3. Navigate the robot manually to build map. After this, run:
    ```shell
    roscd me5413_world/maps/
    rosrun map_server map_saver -f test_map map:=/map
    ```
    This saves the map in me5413_world/maps.
    
4. (Optional) Evaluate the trajectory.
    ```shell
    roscd final_slam/eval
    python3 merge_bags.py merged.bag ground_truth.bag slam_traj.bag
    evo_ape bag merged.bag /gazebo/ground_truth/state /tracked_pose --align --plot
    ```

This package contains:
### config/

- `ME5413_final_2d.lua`: 
  This is the main Cartographer 2D SLAM configuration file. It sets parameters such as map resolution, scan matching, pose graph optimization frequency, and sensor input topics. 
  Used in: 
  ```shell
  roslaunch final_slam mapping_carto_2d.launch
  ```

- `velodyne.yaml`: 
  Contains calibration and driver parameters for the Velodyne LiDAR used in SLAM. It includes data rate, field of view, scan topic, and range filtering settings. 
  Must be correctly configured before launching Cartographer or GMapping.

---

### launch/

- `mapping_carto_2d.launch`: 
  Launches Cartographer 2D with Velodyne input. Sets up the SLAM node, tf tree, and relevant topic remappings. 
  Run with: 
  ```shell
  roslaunch final_slam mapping_carto_2d.launch
  ```

- `cartographer.launch`: 
  A general-purpose launcher template for Cartographer SLAM (can switch to 2D or 3D by changing the config path inside). 
  For testing custom configs.

- `gmapping.launch`: 
  Launches GMapping-based SLAM using laser scan and odometry. Useful for baseline comparison. 
  Run with: 
  ```shell
  roslaunch final_slam gmapping.launch
  ```

- `amcl.launch`: 
  For localization after mapping. Loads a prebuilt map and performs probabilistic localization (AMCL) in the same environment.

- `ME5413_final_mapping_2d.launch`: 
  Final all-in-one launcher used in the demo, integrating sensor drivers, Cartographer, and RViz. 
  Run for quick 2D SLAM launch: 
  ```shell
  roslaunch final_slam ME5413_final_mapping_2d.launch
  ```

---

### maps/

- `my_map.pgm` and `final_map.pgm`: 
  Generated 2D occupancy grid maps from Cartographer. Saved using `map_saver` or automatically through SLAM node. 
  Corresponding `.yaml` files define map resolution, origin, and format.

---

### rviz/

- `gmapping.rviz`: 
  Custom RViz configuration to visualize laser scans, maps, robot poses, and trajectories. 

---

### scripts/

- `record_slam_data_ros1.sh`: 
  Bash script for recording ROS topics during SLAM sessions.

# 3. Navigation
All the scripts are filed in "All_scripts" folder. Before running the scripts，please first start the gazebo world and navigation package with：
```bash
roslaunch me5413_world world.launch
roslaunch navigation_pkg navigation.launch
```

To localize the boxes generated in the area and navigate the robot with a snake-like path, running: 
```bash
#Running algorithm to localize the boxes
python3 findcube.py
#Running algorithm to execute the snake-like path to explore the area
python3 snake_path.py
```
After finishing the snake-like path, subscribing to the locations, planning the shortest path by solving the Travelling Sales Man problem to visit each box one by one and recognizing the numbers on the boxes using the perception function:
```bash
#Running the perception node
python3 digit_recognition_service_node.py
#Runing algorithm to visit the boxes and store the numbers in an array
python3 subscribe_box_pos.py
```
Now, the numbers of randomly generated boxes are recognized, running the algorithms to get the bridge centre location and cross the bridge:
```bash
#Running the bridge location dection node
python3 findbridge.py
#Navigating the roobot to bridge, publish msg to remove obstacle and cross the bridge
python3 cross_bridge.py
```
if you wish to run all the above scripts all in once to complete the tasks, using:
```bash
#Running all the scripts sequentially
python3 start_robot.py 
```





## License

The [ME5413_Final_Project](https://github.com/NUS-Advanced-Robotics-Centre/ME5413_Final_Project) is released under the [MIT License](https://github.com/NUS-Advanced-Robotics-Centre/ME5413_Final_Project/blob/main/LICENSE)
