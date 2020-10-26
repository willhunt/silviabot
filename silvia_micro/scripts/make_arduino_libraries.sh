#!/bin/sh

# Remove existing libraries
rm -rf $HOME/Arduino/libraries/ros_lib/django_interface
rm -rf ../libraries/ros_lib
# Recreate
rosrun rosserial_arduino make_libraries.py ../libraries
cp -R ../libraries/ros_lib/django_interface $HOME/Arduino/libraries/ros_lib
rm -rf ../libraries/ros_lib