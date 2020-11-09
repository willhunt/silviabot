# Notes - Temporary before transfer to docs

## Setup
```bash
$ mkdir -p catkin_ws/src
$ pip install roslibpy
```

## ROS testing
```bash
$ rostopic pub status_change django_interface/SilviaStatus "{brew: false, mode: 2}"
$ rostopic pub pump_duty std_msgs/Float64 100
$ rostopic pub status_change django_interface/SilviaStatus "{brew: true, mode: -1}"
```