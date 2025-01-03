# corgi_ros_ws
This repository contains ROS packages and related tools for both simulation and real robot control of the Corgi robot. 


## Prerequisites
Before you begin, ensure you have the following dependencies installed and accessible on your system:

- **grpc_core**: [https://github.com/kyle1548/grpc_core.git](https://github.com/kyle1548/grpc_core.git)
- **ROS** (including `catkin` tools)
- **Webots**
- **Eigen (C++)**
- **PyQt5 (Python)**
- **Jetson.GPIO (Python, required by Jetson Orin)**


## Directory Structure
A suggested directory layout is as follows:

```
${HOME}/
└─ corgi_ws/
    ├─ install/                 - Installed packages directory
    ├─ grpc_core/               - gRPC communication dependency
    └─ corgi_ros_ws/            - Main ROS workspace
        └─ src/                     - Source code for Corgi packages
            ├─ corgi_csv_control        - Trajectory control using CSV files
            ├─ corgi_data_recorder      - Data logging tools
            ├─ corgi_msgs               - ROS message definitions
            ├─ corgi_panel              - Control panel for the real robot
            ├─ corgi_ros_bridge         - ROS bridge for gRPC communication
            ├─ corgi_sim                - Simulation environment
            ├─ corgi_utils              - Utility functions
            └─ corgi_virtual_agent      - Test gRPC without a real robot
```

Note: Adjust the paths as necessary if your directory structure differs.


## Build
To build the workspace, follow these steps:

```bash
cd ~/corgi_ws/corgi_ros_ws
catkin build -DLOCAL_PACKAGE_PATH=${HOME}/corgi_ws/install
source devel/setup.bash
```


## Launching the System

### Simulation
To run the simulation in Webots:

```bash
roslaunch corgi_sim run_simulation.launch
```

Note: Please add ```export WEBOTS_HOME=/usr/local/webots``` in ```~/.bashrc``` file.

### Real Robot
To run the control panel for the real robot:

```bash
roslaunch corgi_panel corgi_control_panel.launch
```


## Control Options

### CSV Control
If you have a CSV trajectory prepared, you can run it as follows:

```bash
rosrun corgi_csv_control corgi_csv_control [your_csv_traj.csv (w/o suffix)]
```

### Real Time Control
For more advanced, real-time control strategies, run your corresponding real-time control package:

```bash
rosrun your_real_time_package your_real_time_node
```

Note: Publish ```/motor/command``` topic to control the robot.
