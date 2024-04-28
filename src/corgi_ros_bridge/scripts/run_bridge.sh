#!/bin/bash
sleep .5
echo -e "\n\n========================================"
echo -e "       IMPORTANT INSTRUCTION"
echo -e "========================================"
echo -e "\n>> Please run Webots before proceeding.\n"
echo -e ">> Press Enter to start the bridge node...\n"
echo -e "========================================\n\n"
read
rosrun corgi_ros_bridge corgi_ros_bridge_webots