Robucar's Repository 
====================

##1. Introduction:

This repository contains ROS packages for CDTA's robucar, enabling interfacing with ROS, control, teleoperation and autonomous navigation using ROS navigation stack.  

##1. Set up:

In order to use these packages one needs to create a catkin workspace first than clone the git repository inside a folder named "src"

'''

mkdir robucar_ws
cd robucar_ws
git clone https://github.com/Ily4s/robucar.git src
cd src
catkin_init_workspace
sudo chmod +x robucar_driver/scripts/*.py
sudo chmod +x robucar_controller/scripts/*.py
sudo chmod +x robucar_tele/scripts/*.py
cd ..
catkin_make

'''

If this repo is used frequently consider adding a line to source the workspace in .bashrc

'''

source path_to_ws/devel/setup.bash

''' 

-------------

##2. Usage:




-------------

