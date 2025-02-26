#include "fsm_control.hpp"
#include <ros/ros.h>
#include <corgi_msgs/FsmCmdStamped.h>
#include <iostream>

int main(int argc, char **argv) {
    ROS_INFO("FSM Command Publisher Starts");

    // Initialize the ROS node.
    ros::init(argc, argv, "fsm_cmd_pub");
    ros::NodeHandle nh;

    // Set up the publisher.
    ros::Publisher fsm_cmd_pub = nh.advertise<corgi_msgs::FsmCmdStamped>("fsm/command", 1000);
    
    // Although a loop rate is set, note that the loop will block on user input.
    ros::Rate rate(1000);

    // Prepare the FSM command message.
    corgi_msgs::FsmCmdStamped fsm_cmd;
    fsm_cmd.next_mode = IDLE_MODE;
    fsm_cmd.body_vel = 0.1;
    fsm_cmd.curvature = 0;

    int loop_count = 0;
    while (ros::ok()) {
        ros::spinOnce();

        // Reset pause and stop flags each iteration.
        fsm_cmd.pause = false;
        fsm_cmd.stop = false;

        // Wait for user input (this is a blocking call).
        char input_char;
        std::cout << "[0] IDLE; [1] CSV; [2] WHEEL; [3] WALK; [4] WLW; [5] STAIR\n"
                  << "[p] Pause; [r] Resume; [t] Stop; [q] Quit:  ";
        std::cin >> input_char;

        // Process the user input.
        switch (input_char) {
            case '0':
                fsm_cmd.next_mode = IDLE_MODE;
                break;
            case '1':
                fsm_cmd.next_mode = CSV_MODE;
                break;
            case '2':
                fsm_cmd.next_mode = WHEEL_MODE;
                break;
            case '3':
                fsm_cmd.next_mode = WALK_MODE;
                break;
            case '4':
                fsm_cmd.next_mode = WLW_MODE;
                break;
            case '5':
                fsm_cmd.next_mode = STAIR_MODE;
                break;
            case 'p':
                fsm_cmd.pause = true;
                break;
            case 'r':
                fsm_cmd.pause = false;
                break;
            case 't':
                fsm_cmd.stop = true;
                break;
            case 'q':
                return 0;
            case 'w':
                fsm_cmd.body_vel += 0.01;
                std::cout << "Body Velocity: " << fsm_cmd.body_vel << std::endl;
                break;
            case 'a':
                fsm_cmd.curvature += 0.1;
                break;
            case 's':
                fsm_cmd.body_vel -= 0.01;
                std::cout << "Body Velocity: " << fsm_cmd.body_vel << std::endl;
                break;
            case 'd':
                fsm_cmd.curvature -= 0.1;
                break;
            default:
                std::cout << "Unrecognized input, using previous settings." << std::endl;
                break;
        }
        std::cout << std::endl;

        // Update the header sequence.
        fsm_cmd.header.seq = loop_count;

        // Publish the command message.
        fsm_cmd_pub.publish(fsm_cmd);

        loop_count++;

        // Sleep to maintain loop rate if desired (note that the loop is waiting for input anyway).
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}
