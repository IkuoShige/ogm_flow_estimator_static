#!/bin/bash

dir=~
[ "$1" != "" ] && dir="$1"   #引数があったら、そちらをホームに変える。

source $dir/.bashrc
cd $dir/ros2_ws
colcon build
source $dir/.bashrc

cd $dir/ros2_ws/src/ogm_flow_estimator_static

( sleep 5 && ros2 bag play ./bag/rosbag2_2025_01_22-13_29_26 ) &
timeout 30 ros2 launch ogm_flow_estimator_static ogm_flow_estimator_static.launch.py use_rviz:=false |& tee log.txt

grep -q END log.txt
