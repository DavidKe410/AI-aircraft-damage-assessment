#!/bin/bash


gnome-terminal -- bash -c "source ~/air_dmg_assesment_ws/devel/setup.bash && roslaunch flir_boson_usb flir_boson.launch dev:=/dev/video1; exec bash"

sleep 3

gnome-terminal -- bash -c "source ~/air_dmg_assesment_ws/devel/setup.bash && rosparam load ~/ardu_tof_bridge_ws/src/ros1_bridge/bridge.yaml && roslaunch realsense2_camera rs_camera.launch enable_gyro:=true enable_accel:=true unite_imu_method:=linear_interpolation align_depth:=true; exec bash"

sleep 5

gnome-terminal -- bash -c "source ardu_tof_bridge_ws/install/setup.bash && ros2 run arducam tof_pointcloud; exec bash"

sleep 5

gnome-terminal -- bash -c "source ardu_tof_bridge_ws/install/setup.bash && ros2 run ros1_bridge parameter_bridge; exec bash"

sleep 5

gnome-terminal -- bash -c "source /opt/ros/noetic/setup.bash && rosrun tf static_transform_publisher 0 0 0.075 0.5 0.5 0.5 -0.5 sensor_frame camera_link 100; exec bash"
