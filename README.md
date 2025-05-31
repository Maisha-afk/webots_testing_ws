# webots_testing_ws
***sections***
SLAM: 
**Code Blocks**
('ros2 launch husarion_web slam_toolbox.launch.py')
***sections***
ONLINE SLAM:
**Code Blocks**
ros2 launch slam_toolbox online_sync_launch.py
***sections***
IF BY using rosbag:
**Code Blocks**
ros2 bag record /scan /odometry -o husarion_map_data
***sections***
Saving the map:
**Code Blocks**
ros2 run nav2_map_server map_saver_cli -t map -f husarion_map

