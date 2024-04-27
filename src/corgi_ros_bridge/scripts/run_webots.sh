#!/bin/bash
sleep .1
PACKAGE_PATH=$(rospack find corgi_ros_bridge)
webots --mode=realtime $PACKAGE_PATH/../../corgi_sim/worlds/corgi_default.wbt