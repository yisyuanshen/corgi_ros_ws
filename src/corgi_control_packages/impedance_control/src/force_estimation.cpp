#include "force_estimation.hpp"


void motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
}


Eigen::MatrixXd calculate_P_poly(int rim, double alpha){
    Eigen::Rotation2D<double> rotation(alpha);
    Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();

    H_l_poly << 0.02986810945369848, -0.10150955831419921, 0.00033837074403774705, 0.006765373774739306, 0.007615792562309228, -0.0024121994732995045, -6.520446163389534e-05, 5.435366353719506e-05,
                0.0984219968319075, 0.020210699222679533, -0.04976557612069304, -0.0023520295662631807, 0.0029910867610955538, 0.0009163508036696394, -0.00032806661645546487, 1.823661837557865e-05;
    U_l_poly << 0.009669527527254635, -0.03326882772877039, 0.0014183676837420903, 0.002963308891146737, 0.0008700406282839998, 0.0007517214838673226, -0.0003701278840337812, 2.5056958461185766e-05,
                -0.0006690023490031166, 0.014773023018696044, -0.04975560872784549, 0.029792034672425482, -0.019784732744835352, 0.004526695454915333, 0.0003729635468404478, -0.00016159426009107307;
    F_l_poly << -0.07922197995394527, 0.006429966727950573, 0.002120211801039433, 0.025143170481931387, -0.02091333510279192, 0.005551135130921033, -2.9108620785113164e-05, -0.00013153304571884635,
                -0.04889045741031588, -0.04099768033007372, -0.050576527465387176, 0.05336639156632884, -0.029257201539865652, 0.004423676195682945, 0.0010571619431373274, -0.00025474474248363046;
    L_l_poly << -0.006205715410243533, 0.005373735412447777, 0.06028316700203501, -0.025480735039307013, -0.00855485481636677, 0.008709592938898975, -0.0021348251734653874, 0.00015989475249695854,
                0.020478449370300727, -0.04889887701569285, -0.08046609883658265, 0.04415062837261041, -0.0077196354531666005, -0.004295629132118317, 0.0020770723033178957, -0.00021893555992257824;
    G_poly   << 0,0,0,0,0,0,0,0,
                -0.08004472811678946, -0.04301096555457295, -0.10580886132752444, 0.0888545682810313, -0.031030861225472762, -0.0011104867548842852, 0.0030345590247493667, -0.00046519990417785516;

    L_r_poly << L_l_poly * (-1, 1);
    F_r_poly << F_l_poly * (-1, 1);
    U_r_poly << U_l_poly * (-1, 1);

    if (rim == 1) P_poly = rot_alpha * (H_l_poly-U_l_poly) + U_l_poly;
    else if (rim == 2) P_poly = rot_alpha * (F_l_poly-L_l_poly) + L_l_poly;
    else if (rim == 3) P_poly = G_poly;
    else if (rim == 4) P_poly = rot_alpha * (G_poly-L_r_poly) + L_r_poly;
    else if (rim == 5) P_poly = rot_alpha * (F_r_poly-U_r_poly) + U_r_poly;
    else P_poly << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    
    return P_poly;
}


Eigen::MatrixXd calculate_jacobian(Eigen::MatrixXd P_theta, Eigen::MatrixXd P_theta_deriv, double beta){
    double cos_beta = cos(beta);
    double sin_beta = sin(beta);

    double dtheta_dphiR = -0.5;
    double dtheta_dphiL =  0.5;
    double dbeta_dphiR  =  0.5;
    double dbeta_dphiL  =  0.5;

    double dPx_dtheta = P_theta_deriv(0, 0)*cos_beta - P_theta_deriv(1, 0)*sin_beta;
    double dPy_dtheta = P_theta_deriv(0, 0)*sin_beta + P_theta_deriv(1, 0)*cos_beta;
    double dPx_dbeta  = P_theta(0, 0)*(-sin_beta) - P_theta(1, 0)*cos_beta;
    double dPy_dbeta  = P_theta(0, 0)*cos_beta + P_theta(1, 0)*(-sin_beta);

    double J11 = dPx_dtheta * dtheta_dphiR + dPx_dbeta * dbeta_dphiR;
    double J12 = dPx_dtheta * dtheta_dphiL + dPx_dbeta * dbeta_dphiL;
    double J21 = dPy_dtheta * dtheta_dphiR + dPy_dbeta * dbeta_dphiR;
    double J22 = dPy_dtheta * dtheta_dphiL + dPy_dbeta * dbeta_dphiL;

    Eigen::MatrixXd jacobian(2, 2);
    jacobian << J11, J12, J21, J22;

    return jacobian;
}


Eigen::MatrixXd estimate_force(double theta, double beta, double torque_r, double torque_l){
        LegModel legmodel(true);
        legmodel.contact_map(theta, beta);

        Eigen::MatrixXd P_poly = calculate_P_poly(legmodel.rim, legmodel.alpha);
        Eigen::MatrixXd P_poly_deriv(2, 7);

        for (int i=0; i<7; i++) P_poly_deriv.col(i) = P_poly.col(i+1)*(i+1);

        Eigen::MatrixXd P_theta = Eigen::MatrixXd::Zero(2, 1);
        Eigen::MatrixXd P_theta_deriv = Eigen::MatrixXd::Zero(2, 1);

        for (int i=0; i<8; i++) P_theta += P_poly.col(i) * pow(theta, i); 
        for (int i=0; i<7; i++) P_theta_deriv += P_poly_deriv.col(i) * pow(theta, i); 
        
        Eigen::MatrixXd jacobian(2, 2);
        jacobian = calculate_jacobian(P_theta, P_theta_deriv, beta);
        
        Eigen::MatrixXd torque(2, 1);
        torque << torque_r, torque_l;

        Eigen::MatrixXd force_est = jacobian.inverse().transpose() * torque;

        return force_est;
}


int main(int argc, char **argv) {

    ROS_INFO("Force Estimation Starts\n");

    ros::init(argc, argv, "force_estimate");

    ros::NodeHandle nh;
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    ros::Rate rate(1000);


    std::vector<corgi_msgs::MotorState*> motor_states = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };


    while (ros::ok()) {
        ros::spinOnce();

        printf("\n\nSeq = %d\n", motor_state.header.seq);
        for (auto & state : motor_states){
            Eigen::MatrixXd force_est = estimate_force(state->theta, state->beta, state->torque_r, state->torque_l);

            printf("TB = [%.4lf, %.4lf]\n", state->theta, state->beta);
            printf("Force_est = [%.4lf, %.4lf]\n", force_est(0, 0), force_est(1, 0));
        }

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}