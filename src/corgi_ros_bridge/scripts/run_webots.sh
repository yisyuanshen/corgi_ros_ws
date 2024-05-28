#!/bin/bash
trap "kill -9 -- -$$" INT
NodeCore &
sleep .1
PACKAGE_PATH=$(rospack find corgi_ros_bridge)
webots --mode=realtime $PACKAGE_PATH/../../corgi_sim/worlds/corgi_default.wbt &
# webots --mode=realtime $PACKAGE_PATH/../../corgi_sim/worlds/corgi_force_plate.wbt &
wait