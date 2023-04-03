# ME5413_Final_SLAM

## about initial task and code
see: https://github.com/NUS-Advanced-Robotics-Centre/ME5413_Final_Project

## SLAM algorithm we have used
### Cartographer for 2D and 3D mapping and 2D localization
github:https://github.com/cartographer-project/cartographer_ros  
We store configuration file in "/jackal_cartographer_navigation"  
### Fast_lio for 3D mapping
github: https://github.com/hku-mars/FAST_LIO  
advanced(add loop closure):https://github.com/yanliang-wang/FAST_LIO_LC (still no effective improvement)

## Tools we use in mapping
### octomap
github:https://github.com/OctoMap/octomap  
We use this ROS-Package to convert 3D point cloud into 2D grid map 
### read_pcd
We use this folder to process 3D original mapping result, including pass-through filter,voxcel grid downsampling and RANSAC and so on

## Tools we use in navigation
### costmap_prohibition_layer
github: https://github.com/rst-tu-dortmund/costmap_prohibition_layer  
We use this ROS-Package to set static prohibition area 
### RestrictAreaPlugin
github: https://github.com/honorde/RestrictAreaPlugin  
We use this ROS-Package to set dynamic prohibition area 

## Tools we use in evaluation
### evo
github: https://github.com/MichaelGrupp/evo  
We use this Python package to evaluate our map result