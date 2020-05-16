#!/bin/sh
#if [ ! -z "$1" ]; then
#	echo "Keeping all windows open!"
#	cd catkin_ws
#	catkin_make && (
#	gnome-terminal -e 'bash -c "roslaunch stage_ros stage.launch; bash"'
#	sleep 1
#	gnome-terminal -e 'bash -c "roslaunch g2o_based_mapping g2o_based_mapping.launch; bash"'
#	sleep 1
#	gnome-terminal -e 'bash -c "rosrun g2o_based_mapping g2o_based_mapping _config:=~/catkin_ws/src/advanced_robotics/g2o_based_mapping/launch/icp.yaml; bash"'
#	echo FINISHED )
#else
#	cd catkin_ws
#	catkin_make && (
#	gnome-terminal -e "roslaunch stage_ros stage.launch"
#	sleep 1
#	gnome-terminal -e "roslaunch g2o_based_mapping g2o_based_mapping.launch"
#	sleep 1
#	gnome-terminal -e "rosrun g2o_based_mapping g2o_based_mapping _config:=~/catkin_ws/src/advanced_robotics/g2o_based_mapping/launch/icp.yaml"
#	echo FINISHED )
#fi

if [ ! -z "$1" ]; then
	echo "Keeping all windows open!"
	cd beetle_rover
	(
	gnome-terminal -e 'roslaunch beetle_sim_description move.launch '
	sleep 1
	gnome-terminal -e 'roslaunch beetle_localzation_node start.launch '
	sleep 1
	gnome-terminal -e 'rosrun beetle_kinematic_model beetle_kinematic_model.py'
	sleep 1
	gnome-terminal -e 'rosrun beetle_control beetle_control.py'
	sleep 1
	gnome-terminal -e 'rosrun beetle_wheel_odometry beetle_wheel_odometry.py'
	sleep 1
	gnome-terminal -e 'roslaunch zed_wrapper zed.launch'
	sleep 1
	gnome-terminal -e 'rosrun beetle_imu ros_serial_publisher.py ' 
	sleep 1
	gnome-terminal -e 'rosrun joy joy_node '
	sleep 1
	gnome-terminal -e 'rosrun teleop_twist_joy teleop_node '
	echo FINISHED )
else
	cd beetle_rover
	(
	gnome-terminal -e 'roslaunch beetle_sim_description move.launch '
	sleep 1
	gnome-terminal -e 'roslaunch beetle_localzation_node start.launch '
	sleep 1
	gnome-terminal -e 'rosrun beetle_kinematic_model beetle_kinematic_model.py'
	sleep 1
	gnome-terminal -e 'rosrun beetle_control beetle_control.py'
	sleep 1
	gnome-terminal -e 'rosrun beetle_wheel_odometry beetle_wheel_odometry.py'
	sleep 1
	gnome-terminal -e 'roslaunch zed_wrapper zed.launch'
	sleep 1
	gnome-terminal -e 'rosrun beetle_imu ros_serial_publisher.py '
	sleep 1
	gnome-terminal -e 'rosrun joy joy_node '
	sleep 1
	gnome-terminal -e 'rosrun teleop_twist_joy teleop_node ' 
	echo FINISHED )
fi
