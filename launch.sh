#!/bin/bash

ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image[gz.msgs.Image &
ros2 run ros_gz_bridge parameter_bridge /depth_camera@sensor_msgs/msg/Image[gz.msgs.Image &
ros2 run ros_gz_bridge parameter_bridge /depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked &

MicroXRCEAgent udp4 -p 8888
