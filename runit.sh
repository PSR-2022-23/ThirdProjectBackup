#!/bin/bash

gnome-terminal -e "bash -c 'cd ~/catkin_ws; source devel/setup.bash; catkin_make install && catkin_make; roslaunch robutler_bringup gazebo.launch; exec bash'" &
gnome-terminal -e "bash -c 'cd ~/catkin_ws/src/robutler_bringup/src;pwd;chmod +x menu.py;cd ~/catkin_ws/src/teleop/src/; pwd; chmod +x key_teleop.py; cd ~/catkin_ws; source devel/setup.bash; catkin_make install && catkin_make; roslaunch robutler_bringup bringup.launch; exec bash'" 