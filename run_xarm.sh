#!/bin/bash
roslaunch dual_xarm6_moveit_config L_xarm_dual_realMove_exec.launch show_rviz:=false &
sleep 6 ; roslaunch ydlidar_ros_driver G2.launch &
sleep 2 ; roslaunch ./xarm_pipe/laser_assembler.launch &
sleep 2 ; ./xarm_pipe/laser_assembler_service_caller

#rosrun robot_upstart install --master http://192.168.0.10:11311 dual_xarm6_moveit_config/launch/L_xarm_dual_realMove_exec.launch --job xarm_ros