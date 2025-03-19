#!/bin/bash

dir=~
[ "$1" != "" ] && dir="$1"   #引数があったら、そちらをホームに変える。

source $dir/.bashrc
cd $dir/ros2_ws/src/ros2_rust
git pull origin main
git checkout becc938e855587d75237cf82b9c312ebc49136ae
cd $dir/ros2_ws
rm -rf $dir/ros2_ws/src/ros2 $dir/ros2_ws/src/ros2-rust
vcs import src < src/ros2_rust/ros2_rust_humble.repos
colcon build
source $dir/.bashrc

cd $dir/ros2_ws/src/ogm_flow_estimator_static

( sleep 5 && ros2 bag play ./bag/rosbag2_2025_01_22-13_29_26 ) &
timeout 30 ros2 launch ogm_flow_estimator_static ogm_flow_estimator_static.launch.py use_rviz:=false |& tee log.txt

grep -q END log.txt
