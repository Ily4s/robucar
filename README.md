Robucar's Repository 
====================

##1. Introduction:

This repository contains ROS packages for CDTA's robucar, enabling interfacing with ROS, control, teleoperation and autonomous navigation using ROS navigation stack.  

##1. Set up:

In order to use these packages one needs to create a catkin workspace first than clone the git repository inside a folder named "src"

```
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
```

If this repo is used frequently consider adding a line to source the workspace in .bashrc

```
source path_to_ws/devel/setup.bash
``` 

-------------

##2. Usage:

To launch just the controller part use the launch files in the drivers and controller packages.
First you have to set the forcedmode parameter in "robucar_controller.launch". 
```
param name="forcedmode" type="string" value="no"
``` 
forced mode can be: no, simple or double, for the default mode forced simple steering mode and all wheels (double) steering mode respectively.

to launch all the packages with teleoperation you can use the robucar_configuration.launch (found in the 2dnav package) this file includes the other needed launch files from other packages. however the laser node should be launched first.

before launching anything be sure to set up the parameters of the ip addresses of local laptop and the teleoperator laptop. These parameters are found in "robucar_tele_remote.launch" and "robucar_tele.launch" launch files.

after launching all packages on the teleoperator you can launch the packages in the local laptop.

Here's an example of launching the system:

On the teleoperator:

```
term1$ roscore

term2$ rosrun sicktoolbox_wrapper2 sicklms5xx

term3$ roslaunch robucar_2dnav robucar_configuration.launch 

term4$ roslaunch robucar_2dnav move_base.launch
``` 

On the local system:

```
term1$ roslaunch robucar_tele robucar_tele_remote.launch 

term2$ roslaunch robucar_description robucar_rviz_map.launch
``` 


-------------

