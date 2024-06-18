#include <corgi_ros_bridge.hpp>

void motor_feedback_cb(motor_msg::MotorStamped msg)
{
    mtx.lock();
    motor_fb_msg = msg;
    motor_msg_updated = 1;
    mtx.unlock();
}

void force_feedback_cb(force_msg::LegForceStamped msg)
{
    mtx.lock();
    force_fb_msg = msg;
    force_msg_updated = 1;
    mtx.unlock();
}

void robot_feedback_cb(robot_msg::StateStamped msg)
{
    mtx.lock();
    robot_fb_msg = msg;
    robot_msg_updated = 1;
    mtx.unlock();
}

void ros_robot_feedback_cb(corgi_ros_bridge::RobotStamped msg)
{
    ros_robot_fb_msg = msg;
    ros_robot_msg_updated = 1;
}

void load_config()
{
    std::ifstream file("/home/biorola/corgi_ros_ws/src/corgi_ros_bridge/config/config.yaml");
    // std::ifstream file("/home/yisyuan/corgi_ros_ws/src/corgi_ros_bridge/config/config.yaml");
    YAML::Node config = YAML::Load(file);
    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
        std::string module_name = it->first.as<std::string>();
        YAML::Node module = it->second;

        KP.push_back(module["KP"].as<double>());
        KI.push_back(module["KI"].as<double>());
        KD.push_back(module["KD"].as<double>());
        M_d.push_back(module["M_d"].as<std::vector<double>>());
        K_0.push_back(module["K_0"].as<std::vector<double>>());
        D_d.push_back(module["D_d"].as<std::vector<double>>());
        adaptive_pid_x.push_back(module["Adaptive_pid_x"].as<std::vector<double>>());
        adaptive_pid_y.push_back(module["Adaptive_pid_y"].as<std::vector<double>>());
    }
    std::cout << "load config successfully" << std::endl;
}

int main(int argc, char **argv)
{
    std::cout << "corgi ros bridge started" << std::endl;
    motor_msg_updated = 0;
    force_msg_updated = 0;
    robot_msg_updated = 0;
    ros_robot_msg_updated = 0;

    ros::init(argc, argv, "corgi_ros_bridge");

    ros::NodeHandle nh;
    ros::Subscriber ros_robot_sub = nh.subscribe<corgi_ros_bridge::RobotStamped>("robot/command", 2000, ros_robot_feedback_cb);
    ros::Publisher ros_robot_pub = nh.advertise<corgi_ros_bridge::RobotStamped>("robot/state", 1000);

    core::NodeHandler nh_;
    core::Subscriber<motor_msg::MotorStamped> &motor_sub = nh_.subscribe<motor_msg::MotorStamped>("motor/state", 1000, motor_feedback_cb);
    // core::Subscriber<force_msg::LegForceStamped> &force_sub = nh_.subscribe<force_msg::LegForceStamped>("force/state", 1000, force_feedback_cb);
    core::Subscriber<force_msg::LegForceStamped> &force_sub = nh_.subscribe<force_msg::LegForceStamped>("robot/force_state", 1000, force_feedback_cb);
    core::Subscriber<robot_msg::StateStamped> &robot_state_sub = nh_.subscribe<robot_msg::StateStamped>("robot/state", 1000, robot_feedback_cb);
    core::Publisher<motor_msg::MotorStamped> &motor_pub = nh_.advertise<motor_msg::MotorStamped>("motor/command");
    // core::Publisher<force_msg::LegForceStamped> &force_pub = nh_.advertise<force_msg::LegForceStamped>("force/command");
    core::Publisher<force_msg::LegForceStamped> &force_pub = nh_.advertise<force_msg::LegForceStamped>("robot/force_command");

    double rt = 100.0;
    core::Rate rate(rt);

    load_config();

    int count = 0;
    while (ros::ok())
    {
        ROS_INFO_STREAM("Loop Count: " << count);

        ros::spinOnce();
        core::spinOnce();

        if (ros_robot_msg_updated)
        {
            if (ros_robot_fb_msg.msg_type == "force")
            {
                force_msg::LegForceStamped force_cmd_msg;
                force_msg::LegForce force_cmd;
                force_msg::Impedance imp_cmd;

                mtx.lock();

                force_cmd.set_pose_x(ros_robot_fb_msg.A_LF.force.pose_x);
                force_cmd.set_pose_y(ros_robot_fb_msg.A_LF.force.pose_y);
                force_cmd.set_force_x(ros_robot_fb_msg.A_LF.force.force_x);
                force_cmd.set_force_y(ros_robot_fb_msg.A_LF.force.force_y);
                imp_cmd.set_m_x(M_d[0][0]);
                imp_cmd.set_m_y(M_d[0][1]);
                imp_cmd.set_k0_x(K_0[0][0]);
                imp_cmd.set_k0_y(K_0[0][1]);
                imp_cmd.set_d_x(D_d[0][0]);
                imp_cmd.set_d_y(D_d[0][1]);
                imp_cmd.set_adaptive_kp_x(adaptive_pid_x[0][0]);
                imp_cmd.set_adaptive_kp_y(adaptive_pid_y[0][0]);
                imp_cmd.set_adaptive_ki_x(adaptive_pid_x[0][1]);
                imp_cmd.set_adaptive_ki_y(adaptive_pid_y[0][1]);
                imp_cmd.set_adaptive_kd_x(adaptive_pid_x[0][2]);
                imp_cmd.set_adaptive_kd_y(adaptive_pid_y[0][2]);
                force_cmd_msg.add_force()->CopyFrom(force_cmd);
                force_cmd_msg.add_impedance()->CopyFrom(imp_cmd);

                force_cmd.set_pose_x(ros_robot_fb_msg.B_RF.force.pose_x);
                force_cmd.set_pose_y(ros_robot_fb_msg.B_RF.force.pose_y);
                force_cmd.set_force_x(ros_robot_fb_msg.B_RF.force.force_x);
                force_cmd.set_force_y(ros_robot_fb_msg.B_RF.force.force_y);
                imp_cmd.set_m_x(M_d[1][0]);
                imp_cmd.set_m_y(M_d[1][1]);
                imp_cmd.set_k0_x(K_0[1][0]);
                imp_cmd.set_k0_y(K_0[1][1]);
                imp_cmd.set_d_x(D_d[1][0]);
                imp_cmd.set_d_y(D_d[1][1]);
                imp_cmd.set_adaptive_kp_x(adaptive_pid_x[1][0]);
                imp_cmd.set_adaptive_kp_y(adaptive_pid_y[1][0]);
                imp_cmd.set_adaptive_ki_x(adaptive_pid_x[1][1]);
                imp_cmd.set_adaptive_ki_y(adaptive_pid_y[1][1]);
                imp_cmd.set_adaptive_kd_x(adaptive_pid_x[1][2]);
                imp_cmd.set_adaptive_kd_y(adaptive_pid_y[1][2]);
                force_cmd_msg.add_force()->CopyFrom(force_cmd);
                force_cmd_msg.add_impedance()->CopyFrom(imp_cmd);

                force_cmd.set_pose_x(ros_robot_fb_msg.C_RH.force.pose_x);
                force_cmd.set_pose_y(ros_robot_fb_msg.C_RH.force.pose_y);
                force_cmd.set_force_x(ros_robot_fb_msg.C_RH.force.force_x);
                force_cmd.set_force_y(ros_robot_fb_msg.C_RH.force.force_y);
                imp_cmd.set_m_x(M_d[2][0]);
                imp_cmd.set_m_y(M_d[2][1]);
                imp_cmd.set_k0_x(K_0[2][0]);
                imp_cmd.set_k0_y(K_0[2][1]);
                imp_cmd.set_d_x(D_d[2][0]);
                imp_cmd.set_d_y(D_d[2][1]);
                imp_cmd.set_adaptive_kp_x(adaptive_pid_x[2][0]);
                imp_cmd.set_adaptive_kp_y(adaptive_pid_y[2][0]);
                imp_cmd.set_adaptive_ki_x(adaptive_pid_x[2][1]);
                imp_cmd.set_adaptive_ki_y(adaptive_pid_y[2][1]);
                imp_cmd.set_adaptive_kd_x(adaptive_pid_x[2][2]);
                imp_cmd.set_adaptive_kd_y(adaptive_pid_y[2][2]);
                force_cmd_msg.add_force()->CopyFrom(force_cmd);
                force_cmd_msg.add_impedance()->CopyFrom(imp_cmd);

                force_cmd.set_pose_x(ros_robot_fb_msg.D_LH.force.pose_x);
                force_cmd.set_pose_y(ros_robot_fb_msg.D_LH.force.pose_y);
                force_cmd.set_force_x(ros_robot_fb_msg.D_LH.force.force_x);
                force_cmd.set_force_y(ros_robot_fb_msg.D_LH.force.force_y);
                imp_cmd.set_m_x(M_d[3][0]);
                imp_cmd.set_m_y(M_d[3][1]);
                imp_cmd.set_k0_x(K_0[3][0]);
                imp_cmd.set_k0_y(K_0[3][1]);
                imp_cmd.set_d_x(D_d[3][0]);
                imp_cmd.set_d_y(D_d[3][1]);
                imp_cmd.set_adaptive_kp_x(adaptive_pid_x[3][0]);
                imp_cmd.set_adaptive_kp_y(adaptive_pid_y[3][0]);
                imp_cmd.set_adaptive_ki_x(adaptive_pid_x[3][1]);
                imp_cmd.set_adaptive_ki_y(adaptive_pid_y[3][1]);
                imp_cmd.set_adaptive_kd_x(adaptive_pid_x[3][2]);
                imp_cmd.set_adaptive_kd_y(adaptive_pid_y[3][2]);
                force_cmd_msg.add_force()->CopyFrom(force_cmd);
                force_cmd_msg.add_impedance()->CopyFrom(imp_cmd);

                mtx.unlock();

                force_pub.publish(force_cmd_msg);
                ROS_INFO_STREAM("Force Command Published");
            }
            else if (ros_robot_fb_msg.msg_type == "motor")
            {
                motor_msg::MotorStamped motor_cmd_msg;
                motor_msg::Motor motor_cmd;
                motor_msg::LegAngle leg_cmd;

                mtx.lock();

                motor_cmd.set_angle(ros_robot_fb_msg.A_LF.motor_r.angle);
                motor_cmd.set_torque(ros_robot_fb_msg.A_LF.motor_r.torque);
                motor_cmd.set_kp(KP[0]);
                motor_cmd.set_ki(KI[0]);
                motor_cmd.set_kd(KD[0]);
                motor_cmd_msg.add_motors()->CopyFrom(motor_cmd);

                motor_cmd.set_angle(ros_robot_fb_msg.A_LF.motor_l.angle);
                motor_cmd.set_torque(ros_robot_fb_msg.A_LF.motor_l.torque);
                motor_cmd.set_kp(KP[0]);
                motor_cmd.set_ki(KI[0]);
                motor_cmd.set_kd(KD[0]);
                motor_cmd_msg.add_motors()->CopyFrom(motor_cmd);

                leg_cmd.set_theta(ros_robot_fb_msg.A_LF.theta);
                leg_cmd.set_beta(ros_robot_fb_msg.A_LF.beta);
                motor_cmd_msg.add_legs()->CopyFrom(leg_cmd);

                motor_cmd.set_angle(ros_robot_fb_msg.B_RF.motor_r.angle);
                motor_cmd.set_torque(ros_robot_fb_msg.B_RF.motor_r.torque);
                motor_cmd.set_kp(KP[1]);
                motor_cmd.set_ki(KI[1]);
                motor_cmd.set_kd(KD[1]);
                motor_cmd_msg.add_motors()->CopyFrom(motor_cmd);

                motor_cmd.set_angle(ros_robot_fb_msg.B_RF.motor_l.angle);
                motor_cmd.set_torque(ros_robot_fb_msg.B_RF.motor_l.torque);
                motor_cmd.set_kp(KP[1]);
                motor_cmd.set_ki(KI[1]);
                motor_cmd.set_kd(KD[1]);
                motor_cmd_msg.add_motors()->CopyFrom(motor_cmd);

                leg_cmd.set_theta(ros_robot_fb_msg.B_RF.theta);
                leg_cmd.set_beta(ros_robot_fb_msg.B_RF.beta);
                motor_cmd_msg.add_legs()->CopyFrom(leg_cmd);

                motor_cmd.set_angle(ros_robot_fb_msg.C_RH.motor_r.angle);
                motor_cmd.set_torque(ros_robot_fb_msg.C_RH.motor_r.torque);
                motor_cmd.set_kp(KP[2]);
                motor_cmd.set_ki(KI[2]);
                motor_cmd.set_kd(KD[2]);
                motor_cmd_msg.add_motors()->CopyFrom(motor_cmd);

                motor_cmd.set_angle(ros_robot_fb_msg.C_RH.motor_l.angle);
                motor_cmd.set_torque(ros_robot_fb_msg.C_RH.motor_l.torque);
                motor_cmd.set_kp(KP[2]);
                motor_cmd.set_ki(KI[2]);
                motor_cmd.set_kd(KD[2]);
                motor_cmd_msg.add_motors()->CopyFrom(motor_cmd);

                leg_cmd.set_theta(ros_robot_fb_msg.C_RH.theta);
                leg_cmd.set_beta(ros_robot_fb_msg.C_RH.beta);
                motor_cmd_msg.add_legs()->CopyFrom(leg_cmd);

                motor_cmd.set_angle(ros_robot_fb_msg.D_LH.motor_r.angle);
                motor_cmd.set_torque(ros_robot_fb_msg.D_LH.motor_r.torque);
                motor_cmd.set_kp(KP[3]);
                motor_cmd.set_ki(KI[3]);
                motor_cmd.set_kd(KD[3]);
                motor_cmd_msg.add_motors()->CopyFrom(motor_cmd);

                motor_cmd.set_angle(ros_robot_fb_msg.D_LH.motor_l.angle);
                motor_cmd.set_torque(ros_robot_fb_msg.D_LH.motor_l.torque);
                motor_cmd.set_kp(KP[3]);
                motor_cmd.set_ki(KI[3]);
                motor_cmd.set_kd(KD[3]);
                motor_cmd_msg.add_motors()->CopyFrom(motor_cmd);

                leg_cmd.set_theta(ros_robot_fb_msg.D_LH.theta);
                leg_cmd.set_beta(ros_robot_fb_msg.D_LH.beta);
                motor_cmd_msg.add_legs()->CopyFrom(leg_cmd);

                mtx.unlock();

                motor_pub.publish(motor_cmd_msg);
                ROS_INFO_STREAM("Motor Command Published");
            }

            ros_robot_msg_updated = 0;
        }

        if (robot_msg_updated || force_msg_updated || motor_msg_updated)
        {
            mtx.lock();

            if (robot_msg_updated)
            {
                robot_state_msg.pose.position.x = robot_fb_msg.state().pose().position().x();
                robot_state_msg.pose.position.y = robot_fb_msg.state().pose().position().y();
                robot_state_msg.pose.position.z = robot_fb_msg.state().pose().position().z();
                robot_state_msg.pose.orientation.x = robot_fb_msg.state().pose().orientation().x();
                robot_state_msg.pose.orientation.y = robot_fb_msg.state().pose().orientation().y();
                robot_state_msg.pose.orientation.z = robot_fb_msg.state().pose().orientation().z();
                robot_state_msg.pose.orientation.w = robot_fb_msg.state().pose().orientation().w();
                robot_state_msg.twist.linear.x = robot_fb_msg.state().twist().linear().x();
                robot_state_msg.twist.linear.y = robot_fb_msg.state().twist().linear().y();
                robot_state_msg.twist.linear.z = robot_fb_msg.state().twist().linear().z();
                robot_state_msg.twist.angular.x = robot_fb_msg.state().twist().angular().x();
                robot_state_msg.twist.angular.y = robot_fb_msg.state().twist().angular().y();
                robot_state_msg.twist.angular.z = robot_fb_msg.state().twist().angular().z();

                robot_msg_updated = 0;
                ROS_INFO_STREAM("Robot State Received");
            }

            if (force_msg_updated)
            {
                if (force_fb_msg.force().size() == 4)
                {
                    robot_state_msg.A_LF.force.pose_x = force_fb_msg.force(0).pose_x();
                    robot_state_msg.A_LF.force.pose_y = force_fb_msg.force(0).pose_y();
                    robot_state_msg.A_LF.force.force_x = force_fb_msg.force(0).force_x();
                    robot_state_msg.A_LF.force.force_y = force_fb_msg.force(0).force_y();
                    robot_state_msg.B_RF.force.pose_x = force_fb_msg.force(1).pose_x();
                    robot_state_msg.B_RF.force.pose_y = force_fb_msg.force(1).pose_y();
                    robot_state_msg.B_RF.force.force_x = force_fb_msg.force(1).force_x();
                    robot_state_msg.B_RF.force.force_y = force_fb_msg.force(1).force_y();
                    robot_state_msg.C_RH.force.pose_x = force_fb_msg.force(2).pose_x();
                    robot_state_msg.C_RH.force.pose_y = force_fb_msg.force(2).pose_y();
                    robot_state_msg.C_RH.force.force_x = force_fb_msg.force(2).force_x();
                    robot_state_msg.C_RH.force.force_y = force_fb_msg.force(2).force_y();
                    robot_state_msg.D_LH.force.pose_x = force_fb_msg.force(3).pose_x();
                    robot_state_msg.D_LH.force.pose_y = force_fb_msg.force(3).pose_y();
                    robot_state_msg.D_LH.force.force_x = force_fb_msg.force(3).force_x();
                    robot_state_msg.D_LH.force.force_y = force_fb_msg.force(3).force_y();

                    ROS_INFO_STREAM("Force State Received");
                }
                force_msg_updated = 0;
            }

            if (motor_msg_updated)
            {
                if (motor_fb_msg.motors().size() == 8)
                {
                    robot_state_msg.A_LF.motor_r.angle = motor_fb_msg.motors(0).angle();
                    robot_state_msg.A_LF.motor_r.torque = motor_fb_msg.motors(0).torque();
                    robot_state_msg.A_LF.motor_r.twist = motor_fb_msg.motors(0).twist();
                    robot_state_msg.A_LF.motor_l.angle = motor_fb_msg.motors(1).angle();
                    robot_state_msg.A_LF.motor_l.torque = motor_fb_msg.motors(1).torque();
                    robot_state_msg.A_LF.motor_l.twist = motor_fb_msg.motors(1).twist();
                    // robot_state_msg.A_LF.theta = motor_fb_msg.legs(0).theta();
                    // robot_state_msg.A_LF.beta = motor_fb_msg.legs(0).beta();

                    robot_state_msg.B_RF.motor_r.angle = motor_fb_msg.motors(2).angle();
                    robot_state_msg.B_RF.motor_r.torque = motor_fb_msg.motors(2).torque();
                    robot_state_msg.B_RF.motor_r.twist = motor_fb_msg.motors(2).twist();
                    robot_state_msg.B_RF.motor_l.angle = motor_fb_msg.motors(3).angle();
                    robot_state_msg.B_RF.motor_l.torque = motor_fb_msg.motors(3).torque();
                    robot_state_msg.B_RF.motor_l.twist = motor_fb_msg.motors(3).twist();
                    // robot_state_msg.B_RF.theta = motor_fb_msg.legs(1).theta();
                    // robot_state_msg.B_RF.beta = motor_fb_msg.legs(1).beta();

                    robot_state_msg.C_RH.motor_r.angle = motor_fb_msg.motors(4).angle();
                    robot_state_msg.C_RH.motor_r.torque = motor_fb_msg.motors(4).torque();
                    robot_state_msg.C_RH.motor_r.twist = motor_fb_msg.motors(4).twist();
                    robot_state_msg.C_RH.motor_l.angle = motor_fb_msg.motors(5).angle();
                    robot_state_msg.C_RH.motor_l.torque = motor_fb_msg.motors(5).torque();
                    robot_state_msg.C_RH.motor_l.twist = motor_fb_msg.motors(5).twist();
                    // robot_state_msg.C_RH.theta = motor_fb_msg.legs(2).theta();
                    // robot_state_msg.C_RH.beta = motor_fb_msg.legs(2).beta();

                    robot_state_msg.D_LH.motor_r.angle = motor_fb_msg.motors(6).angle();
                    robot_state_msg.D_LH.motor_r.torque = motor_fb_msg.motors(6).torque();
                    robot_state_msg.D_LH.motor_r.twist = motor_fb_msg.motors(6).twist();
                    robot_state_msg.D_LH.motor_l.angle = motor_fb_msg.motors(7).angle();
                    robot_state_msg.D_LH.motor_l.torque = motor_fb_msg.motors(7).torque();
                    robot_state_msg.D_LH.motor_l.twist = motor_fb_msg.motors(7).twist();
                    // robot_state_msg.D_LH.theta = motor_fb_msg.legs(3).theta();
                    // robot_state_msg.D_LH.beta = motor_fb_msg.legs(3).beta();

                    ROS_INFO_STREAM("Motor State Received");
                }
                motor_msg_updated = 0;
            }

            mtx.unlock();

            ros_robot_pub.publish(robot_state_msg);
        }

        count += 1;

        ROS_INFO_STREAM(rate.sleep());
    }

    ROS_INFO("Shutting down the node...");

    return 0;
}
