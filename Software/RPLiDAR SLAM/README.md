# Hector SLAM on ROS Using RPLiDAR A1 
* Uses ROS Kinetic with hector_slam and rplidar packages to generate a 2D map of the environment using data from RPLiDAR A1 only. 
* Assumes ROS Kinetic is installed and there is a valid catkin workspace
1. Copy contents of this directory into catkin worspace folder (/src)
2. Run Hector SLAM using RPLiDAR data with the following command:
       'roslaunch hector_slam_launch ROMAP_SLAM.launch'
3. RVIZ viewer will open with a live 2D map generation.
