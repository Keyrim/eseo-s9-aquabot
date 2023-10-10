#!/bin/bash


sudo apt install python3-sdformat13 ros-humble-ros-gzgarden ros-humble-xacro


mkdir -p ~/vrx_ws/src
cd ~/vrx_ws/src
git clone https://github.com/osrf/vrx.git

cd ~/vrx_ws
colcon build --merge-install