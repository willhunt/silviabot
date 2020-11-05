#!/bin/sh

# Remove existing libraries
rm -rf $HOME/Arduino/libraries/ros_lib/django_interface
rm -rf ../libraries/django_interface
# rm -rf ../libraries/ros_lib
# Recreate
rosrun rosserial_arduino make_libraries.py ../libraries
cp -R ../libraries/ros_lib/django_interface $HOME/Arduino/libraries/ros_lib
cp -R ../libraries/ros_lib/django_interface ../libraries
rm -rf ../libraries/ros_lib