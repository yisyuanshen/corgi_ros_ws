# corgi_ros_ws
```
corgi_ros_ws:
|-src
| |-corgi_control_packages
| | |-csv_control
| | |-(other packages) ...
| |
| |-corgi_msgs
| | |-msg
| |
| |-corgi_panel
| |
| |-corgi_ros_bridge
| |
| |-corgi_sensors
| |
```

```
catkin build -DLOCAL_PACKAGE_PATH=$HOME/corgi_ws/install
```

```
source devel/setup.bash
roslaunch corgi_ros_bridge corgi_ros_bridge
```