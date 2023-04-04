# ME5413_Final_SLAM_Group10

## About initial task and code
see: https://github.com/NUS-Advanced-Robotics-Centre/ME5413_Final_Project

## Usage

### 0. gazebo world

This command will launch the gazebo with the project world

```
roslaunch me5413_world world.launch
```

1. ### Mapping
After launching Step 0, in the second terminal to mapping  
choose one algorithm to build maps

```
#  GMapping
roslaunch me5413_world mapping.launch

# Cartographer2D
roslaunch jackal_cartographer_navigation final_mapping.launch

# Cartographer3D
roslaunch jackal_cartographer_navigation final_mapping_3d.launch

# fast_lio
roslaunch fast_lio mapping_velodyne.launch
```
To save 2D map, run the following command in the third terminal to save the map:
```
# Save the map as `my_map` in the `maps/` folder
roscd me5413_world/maps/
rosrun map_server map_saver -f my_map map:=/map
```
To process 3D pcd , run the following command:
```
roslaunch read_pcd read_pcd.launch
```
To convert 3d pcd processed map into 2d grid map (need to install octomap), run the following command in the third terminal to save the map:
```
# do converting
roslaunch read_pcd mapping_convert.launch

# save 2d map
roscd me5413_world/maps/
rosrun map_server map_saver -f my_map /map:=/projected_map
```
## 2. Navigation
Once completed Step 2 mapping and saved your map, quit the mapping process.  
Then, in the second terminal, choose one method to navigate:
```
# Load a map and launch AMCL localizer
roslaunch me5413_world navigation.launch

# Load a map and launch Cartographer localizer
roslaunch me5413_world navigation_cartographer.launch
```
## 3. Evaluation
To evaluate the result of mapping, use evo by comparing your odometry with the published /gazebo/ground_truth/state topic (nav_msgs::Odometry), which contains the gournd truth odometry of the robot.

```
# for Cartographer:
evo_ape bag XX.bag /gazebo/ground_truth/state /tf:map.base_link -a -p --plot_mode=xz

# for fast_lio:
evo_ape bag XX.bag /gazebo/ground_truth/state /Odometry -a -p --plot_mode=xz
```

## Appendix

### SLAM algorithm we have used

#### Cartographer for 2D and 3D mapping and 2D localization

github:https://github.com/cartographer-project/cartographer_ros  
We store configuration file in "/jackal_cartographer_navigation"  
#### Fast_lio for 3D mapping

github: https://github.com/hku-mars/FAST_LIO  
advanced(add loop closure):https://github.com/yanliang-wang/FAST_LIO_LC (still no effective improvement)

### Tools we use in mapping
#### octomap
github:https://github.com/OctoMap/octomap  
We use this ROS-Package to convert 3D point cloud into 2D grid map 
#### read_pcd
We use this folder to process 3D original mapping result, including pass-through filter,voxcel grid downsampling and RANSAC and so on

### Tools we use in navigation
#### costmap_prohibition_layer
github: https://github.com/rst-tu-dortmund/costmap_prohibition_layer  
We use this ROS-Package to set static prohibition area 
#### RestrictAreaPlugin
github: https://github.com/honorde/RestrictAreaPlugin  
We use this ROS-Package to set dynamic prohibition area 

### Tools we use in evaluation
#### evo
github: https://github.com/MichaelGrupp/evo  
We use this Python package to evaluate our map result