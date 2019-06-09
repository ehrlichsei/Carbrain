#!/bin/bash

export CAR_NAME=mrs_furious

roslaunch res/launch/test_navigation.launch&

BAGFILE=~/navigation_refactoring_fast.bag
if [ -f $BAGFILE ]
then 
	echo "$BAGFILE exists"
else
	BAGFILE=~/carolo-storage/Rosbags/navigation/navigation_refactoring_fast.bag
fi

sleep 4s
rviz&
rqt&
sleep 3s

echo "activating path_preprocessing and path_planning"
rosservice call /navigation/path_preprocessing/activate_module "moduleActive: true"
rosservice call /navigation/path_planning/activate_module "moduleActive: true"

rosbag play $@ $BAGFILE --clock

killall roscore
killall roslaunch

