#!/bin/bash
trap "kill -9 -- -$$" INT
sleep 1
rosrun corgi_ros_bridge corgi_ros_bridge&
wait