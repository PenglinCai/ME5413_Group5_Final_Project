
---

# ME5413 Final SLAM
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

---


---
