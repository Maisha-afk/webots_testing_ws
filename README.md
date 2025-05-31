# webots_testing_ws

#SLAM: 
ros2 launch husarion_web slam_toolbox.launch.py

#ONLINE SLAM:
ros2 launch slam_toolbox online_sync_launch.py

#IF BY using rosbag:
ros2 bag record /scan /odometry -o husarion_map_data

#Saving the map:
ros2 run nav2_map_server map_saver_cli -t map -f husarion_map

