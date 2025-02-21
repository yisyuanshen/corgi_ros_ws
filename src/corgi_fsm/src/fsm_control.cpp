#include "fsm_control.hpp"

#include "leg_model.hpp"
#include "walk_gait.hpp"
#include "wheel_to_leg.hpp"

corgi_msgs::MotorCmdStamped motor_cmd;
corgi_msgs::MotorStateStamped motor_state;
corgi_msgs::FsmCmdStamped fsm_cmd;
corgi_msgs::FsmStateStamped fsm_state;

void motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
}

void fsm_cmd_cb(const corgi_msgs::FsmCmdStamped cmd){
    fsm_cmd = cmd;
}


int main(int argc, char **argv) {

    ROS_INFO("FSM Starts\n");

    bool sim = false;

    // ros setup
    ros::init(argc, argv, "corgi_fsm");

    ros::NodeHandle nh;
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    ros::Subscriber fsm_cmd_sub = nh.subscribe<corgi_msgs::FsmCmdStamped>("fsm/command", 1000, fsm_cmd_cb);
    ros::Publisher motor_cmd_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1000);
    ros::Publisher fsm_state_pub = nh.advertise<corgi_msgs::FsmStateStamped>("fsm/state", 1000);
    ros::Rate rate(1000);

    std::vector<corgi_msgs::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    std::vector<corgi_msgs::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    // default motor command
    for (auto& cmd : motor_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0;
        if (sim) {
            cmd->kp_r = 90;
            cmd->kp_l = 90;
        }
        else {
            cmd->kp_r = 150;
            cmd->kp_l = 150;
        }
        cmd->ki_r = 0;
        cmd->ki_l = 0;
        cmd->kd_r = 1.75;
        cmd->kd_l = 1.75;
    }

    // user config
    double body_vel = 0.1;
    double stair_dist = 2.5;
    double turn_radius = 0;

    // initialize
    LegModel leg_model(sim);
    int current_mode = IDLE_MODE;
    int next_mode = IDLE_MODE;
    bool switch_mode = false;
    bool transform_finished = true;
    bool swing_finished = true;
    int step_num_to_stair = 0;
    bool stair_arrived = false;
    bool stair_csv_loaded = false;
    bool paused = false;

    std::array<std::array<double, 4>, 2> eta_list;

    double init_eta[8] = {18/180.0*M_PI, 0, 18/180.0*M_PI, 0, 18/180.0*M_PI, 0, 18/180.0*M_PI, 0};
    
    std::ifstream csv_file;
    std::string csv_line;

    // init walk class
    WalkGait walk_gait(sim, 0.0, 1000);
    walk_gait.initialize(init_eta);
    
    // init wheel to leg class
    WheelToLegTransformer wheel_to_leg_transformer(sim);
    wheel_to_leg_transformer.initialize(init_eta);


    int loop_count = 0;
    while (ros::ok()) {
        ros::spinOnce();

        body_vel = fsm_cmd.body_vel;
        turn_radius = fsm_cmd.turn_radius;

        // check if next_mode is changed
        if (fsm_cmd.next_mode != current_mode && transform_finished && swing_finished) {
            next_mode = fsm_cmd.next_mode;
            switch_mode = true;
        }

        // check if pause command is sent
        if (fsm_cmd.pause) {
            if (!paused) {
                ROS_INFO("FSM: PAUSE!\n");
                paused = true;
            }
            rate.sleep();
            continue;
        }
        else {
            if (paused) {
                ROS_INFO("FSM: RESUME!\n");
                paused = false;
            }
        }

        // check if stop command is sent
        if (fsm_cmd.stop) {
            ROS_INFO("FSM: STOP!\n");
            ros::shutdown();
            return 0;
        }

        // switch mode
        if (switch_mode){
            switch (next_mode) {
                case IDLE_MODE:
                    ROS_INFO("FSM: Entering IDLE MODE\n");
                    break;

                case CSV_MODE:
                    if (current_mode == IDLE_MODE) {
                        ROS_INFO("FSM: Entering CSV MODE\n");
                        break;
                    }
                    rate.sleep();
                    continue;

                case WHEEL_MODE:
                    if (current_mode == IDLE_MODE) {
                        ROS_INFO("FSM: Entering WHEEL MODE\n");
                        break;
                    }
                    rate.sleep();
                    continue;

                case WALK_MODE:
                    for (int i=0; i<4; i++){
                        init_eta[2*i] = motor_cmd_modules[i]->theta;
                        init_eta[2*i+1] = motor_cmd_modules[i]->beta;
                    }
                    if (current_mode == WHEEL_MODE) {
                        transform_finished = false;
                        wheel_to_leg_transformer.initialize(init_eta);
                        ROS_INFO("FSM: Transforming From WHEEL To LEG\n");
                        break;
                    }
                    else if (current_mode == IDLE_MODE) {
                        walk_gait.initialize(init_eta);

                        ROS_INFO("FSM: Entering WALK MODE\n");
                        break;
                    }
                    rate.sleep();
                    continue;

                case WLW_MODE:
                    if (current_mode == IDLE_MODE) {
                        ROS_INFO("FSM: Entering WLW MODE\n");
                        break;
                    }
                    rate.sleep();
                    continue;

                case STAIR_MODE:
                    if (current_mode == IDLE_MODE || current_mode == WHEEL_MODE) {
                        ROS_INFO("FSM: Entering STAIR MODE\n");
                        for (int i=0; i<4; i++){
                            init_eta[2*i] = motor_cmd_modules[i]->theta;
                            init_eta[2*i+1] = motor_cmd_modules[i]->beta;
                        }
                        transform_finished = false;
                        wheel_to_leg_transformer.initialize(init_eta);
                        ROS_INFO("FSM: Transforming From WHEEL To LEG\n");
                        break;
                    }
                    rate.sleep();
                    continue;

                default:
                    ROS_WARN("FSM: Unknown Command Received!\n");
                    break;
            }

            current_mode = next_mode;
            switch_mode = false;
        }
        
        // update
        switch (current_mode) {
            case IDLE_MODE:
                break;

            case CSV_MODE:
                break;

            case WHEEL_MODE:
                for (int i=0; i<4; i++){
                    motor_cmd_modules[i]->theta = 17/180.0*M_PI;
                    motor_cmd_modules[i]->beta += (i == 1 || i == 2) ? -body_vel/leg_model.radius*0.001 : body_vel/leg_model.radius*0.001;
                }
                break;

            case WALK_MODE:
                if (!transform_finished){
                    eta_list = wheel_to_leg_transformer.step();

                    if (wheel_to_leg_transformer.transform_finished) {
                        for (int i=0; i<4; i++){
                            init_eta[2*i] = motor_cmd_modules[i]->theta;
                            init_eta[2*i+1] = motor_cmd_modules[i]->beta;
                        }

                        walk_gait.initialize(init_eta);
                        transform_finished = true;
                        
                        ROS_INFO("FSM: Entering WALK MODE\n");
                    }
                }
                else{
                    walk_gait.set_velocity(body_vel);
                    // walk_gait.set_turn_radius(turn_radius);

                    eta_list = walk_gait.step();
                }

                for (int i=0; i<4; i++) {
                    if (eta_list[0][i] > M_PI*159.9/180.0) {
                        ROS_INFO("Exceed Upper Bound.\n");
                        eta_list[0][i] = M_PI*159.9/180.0;
                    }
                    if (eta_list[0][i] < M_PI*16.9/180.0) {
                        ROS_INFO("Exceed Lower Bound.\n");
                        eta_list[0][i] = M_PI*16.9/180.0;
                    }
                    motor_cmd_modules[i]->theta = eta_list[0][i];
                    motor_cmd_modules[i]->beta = (i == 1 || i == 2) ? eta_list[1][i] : -eta_list[1][i];
                }
                break;

            case WLW_MODE:
                break;

            case STAIR_MODE:
                if (!transform_finished) {
                    eta_list = wheel_to_leg_transformer.step();

                    if (wheel_to_leg_transformer.transform_finished) {
                        for (int i=0; i<4; i++){
                            init_eta[2*i] = motor_cmd_modules[i]->theta;
                            init_eta[2*i+1] = motor_cmd_modules[i]->beta;
                        }

                        walk_gait.initialize(init_eta);
                        
                        double remaining_dist = stair_dist-wheel_to_leg_transformer.total_move_dist;
                        step_num_to_stair = int((remaining_dist-0.6)/0.3) + 1;
                        double step_length = (remaining_dist-0.6) / double(step_num_to_stair);

                        std::cout << "Step Length: " << step_length << std::endl << std::endl;
                        std::cout << "Step Number: " << step_num_to_stair << std::endl << std::endl;

                        walk_gait.set_step_height(0.08);
                        walk_gait.set_step_length(step_length);

                        transform_finished = true;
                        
                        ROS_INFO("FSM: Transform Finished\n");
                    }
                }
                else if (!stair_arrived) {
                    std::array<int, 4> step_count = walk_gait.get_step_count();;

                    if (std::accumulate(step_count.begin(), step_count.end(), 0) == step_num_to_stair*4-1) {
                        walk_gait.set_step_length(0.3);
                    }
                    else if (std::accumulate(step_count.begin(), step_count.end(), 0) == step_num_to_stair*4+8) {
                        for (int i=0; i<4; i++) {
                            std::cout << "eta_" << i << ": [" << eta_list[0][i] << ", " << eta_list[1][i] << "]" << std::endl;
                        }
                        std::cout << std::endl;
                        stair_arrived = true;
                    }

                    walk_gait.set_velocity(body_vel);
                    eta_list = walk_gait.step();
                }
                else if (!stair_csv_loaded) {
                    std::string csv_file_path;
                    csv_file_path = std::getenv("HOME");
                    csv_file_path += "/corgi_ws/corgi_ros_ws/src/corgi_fsm/stair_traj_csv/";
                    
                    if (sim && (eta_list[1][0] < 0)) csv_file_path += "walk2stair_stair_l.csv";
                    else if (sim && (eta_list[1][0] > 0)) csv_file_path += "walk2stair_stair_r.csv";
                    else if (!sim && (eta_list[1][0] < 0)) csv_file_path += "exp_walk2stair_stair_l.csv";
                    else if (!sim && (eta_list[1][0] > 0)) csv_file_path += "exp_walk2stair_stair_r.csv";
                    
                    csv_file.open(csv_file_path);

                    if (!csv_file.is_open()) {
                        ROS_INFO("Failed to open the CSV file\n");
                        return 1;
                    }
                    
                    stair_csv_loaded = true;
                }
                else {
                    if (std::getline(csv_file, csv_line)) {
                        std::vector<double> columns;
                        std::stringstream ss(csv_line);
                        std::string item;
                        
                        for (int i=0; i<4; i++){
                            std::getline(ss, item, ',');
                            eta_list[0][i] =  std::stod(item);
        
                            std::getline(ss, item, ',');
                            eta_list[1][i] = std::stod(item);
                        }
                    }
                    else {
                        break;
                    }
                }

                for (int i=0; i<4; i++) {
                    if (eta_list[0][i] > M_PI*159.9/180.0) {
                        ROS_INFO("Exceed Upper Bound.\n");
                        eta_list[0][i] = M_PI*159.9/180.0;
                    }
                    if (eta_list[0][i] < M_PI*16.9/180.0) {
                        ROS_INFO("Exceed Lower Bound.\n");
                        eta_list[0][i] = M_PI*16.9/180.0;
                    }
                    motor_cmd_modules[i]->theta = eta_list[0][i];
                    if (!stair_csv_loaded) motor_cmd_modules[i]->beta = (i == 1 || i == 2) ? eta_list[1][i] : -eta_list[1][i];
                    else motor_cmd_modules[i]->beta = eta_list[1][i];
                }
                break;

            default:
                break;
        }

        motor_cmd.header.seq = loop_count;
        motor_cmd.header.stamp = ros::Time::now();

        fsm_state.header.seq = loop_count;
        fsm_state.header.stamp = ros::Time::now();

        motor_cmd_pub.publish(motor_cmd);
        fsm_state_pub.publish(fsm_state);

        loop_count++;

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}