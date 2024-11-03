#!/bin/bash
trap "kill 0" INT

(
    grpccore &
    sleep 1
    # rosrun corgi_ros_bridge corgi_ros_bridge log
    rosrun corgi_ros_bridge corgi_ros_bridge &
    rosrun corgi_virtual_agent corgi_virtual_agent
) &

wait