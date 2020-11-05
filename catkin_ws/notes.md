# Notes - Temporary before transfer to docs

## Setup
```bash
$ mkdir -p catkin_ws/src
$ pip install roslibpy
```

## ROS testing
```bash
$ rostopic pub status_request django_interface/SilviaStatus "{brew: false, mode: 1}"
```