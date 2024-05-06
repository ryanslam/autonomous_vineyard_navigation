# autonomous_vineyard_navigation

ros2 launch quanergy_client_ros client.launch.py host:=<hostname_or_ip>
# host = 192.168.1.3 for the lidar we use

ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args --remap cloud_in:=/quanergy/points

to run:
ros2 run ag_laserscan_package alter_laser_scan
ros2 run ag_laserscan_package ag_laser_dist_orien_calc 

ros2 launch quanergy_client_ros client.launch.py host:=<hostname_or_ip>

ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args --remap cloud_in:=/quanergy/points

ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args --remap cloud_in:=/quanergy/points --ros-args -p range_max:=4.5 -p range_min:=1.0 -p max_height:=0.0 -p min_height:=-0.25
