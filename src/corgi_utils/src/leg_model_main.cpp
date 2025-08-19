#include <iostream>
#include <cmath>
#include <stdexcept>
#include <chrono>
#include <Eigen/Dense>

#include "leg_model.hpp"

int main() {
    LegModel legmodel(true);
    std::array<double, 2> new_theta_beta;
    double theta = M_PI * 130.0 / 180.0;
    double beta =  M_PI * 50.0 / 180.0;

    /* Forward kinematics */
    std::cout << "****************************************\n";
    std::cout << "****** Forward kinematics example ******\n";
    std::cout << "****************************************\n";
    auto start = std::chrono::high_resolution_clock::now();
    for (int i=0; i<1000000; i++){
        legmodel.forward(theta, beta);
    }
    auto end = std::chrono::high_resolution_clock::now();
    // 計算執行時間，並轉換成毫秒
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "time: " << duration.count() << " ms" << std::endl;
    std::cout << "Output G with single value input: (" << legmodel.G[0] << ", " << legmodel.G[1] << ")\n";

    // Note: The contact_map function and other advanced features are not fully implemented in this example.
    /* Contact map */
    legmodel.contact_map(theta, beta);
    std::cout << "Output rim with single value input: " << legmodel.rim << std::endl;
    std::cout << "Output alpha with single value input: " << legmodel.alpha << std::endl;
    std::cout << "Output contact_p with single value input: (" << legmodel.contact_p[0] << ", " << legmodel.contact_p[1] << ")\n";
    
    /* Inverse kinematics */
    std::cout << "\n";
    std::cout << "****************************************" << std::endl;
    std::cout << "****** Inverse kinematics example ******" << std::endl;
    std::cout << "****************************************" << std::endl;
    // inverse for G
    std::cout << "==========Inverse for G==========" << std::endl;
    std::array<double, 2> G_p = {0.05, -0.25};
    std::cout << "Input G: " << G_p[0] << ", " << G_p[1] << std::endl;
    new_theta_beta = legmodel.inverse(G_p, "G");
    legmodel.forward(new_theta_beta[0], new_theta_beta[1]);
    std::cout << "Output theta, beta (degree): " << new_theta_beta[0]*180.0/M_PI << ", "<< new_theta_beta[1]*180.0/M_PI << std::endl;
    std::cout << "Output G: " << legmodel.G[0] << ", " << legmodel.G[1] << std::endl;
    // inverse for left upper rim
    std::cout << "==========Inverse for U_l==========" << std::endl;
    std::array<double, 2> Ul_p = {-0.01, -0.015};
    std::cout << "Input U_l: " << Ul_p[0] << ", " << Ul_p[1] << std::endl;
    new_theta_beta = legmodel.inverse(Ul_p, "U_l");
    legmodel.forward(new_theta_beta[0], new_theta_beta[1]);
    std::cout << "Output theta, beta (degree): " << new_theta_beta[0]*180.0/M_PI << ", "<< new_theta_beta[1]*180.0/M_PI << std::endl;
    std::cout << "Output U_l: " << legmodel.U_l[0] << ", " << legmodel.U_l[1] << std::endl;
    // inverse for right lower rim
    std::cout << "==========Inverse for L_r==========" << std::endl;
    std::array<double, 2> Lr_p = {-0.01, -0.015};
    std::cout << "Input L_r: " << Lr_p[0] << ", " << Lr_p[1] << std::endl;
    new_theta_beta = legmodel.inverse(Lr_p, "L_r");
    legmodel.forward(new_theta_beta[0], new_theta_beta[1]);
    std::cout << "Output theta, beta (degree): " << new_theta_beta[0]*180.0/M_PI << ", "<< new_theta_beta[1]*180.0/M_PI << std::endl;
    std::cout << "Output L_r: " << legmodel.L_r[0] << ", " << legmodel.L_r[1] << std::endl;

    /* Move */
    std::cout << "\n";
    std::cout << "**************************" << std::endl;
    std::cout << "****** Move example ******" << std::endl;
    std::cout << "**************************" << std::endl;
    std::array<double, 2> hip = {0.1, 0};
    std::array<double, 2> desired_hip = {0.2, 0};
    auto start2 = std::chrono::high_resolution_clock::now();
    for (int i=0; i<10000; i++){
        new_theta_beta = legmodel.move(theta, beta, {desired_hip[0]-hip[0], desired_hip[1]-hip[1]});
    }
    auto end2 = std::chrono::high_resolution_clock::now();
    // 計算執行時間，並轉換成毫秒
    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2);
    std::cout << "time: " << duration2.count() << " ms" << std::endl;
    std::cout << "Use theta = " << new_theta_beta[0] << ", " << "beta = " << new_theta_beta[1] 
            << " allows the leg to roll from (" << hip[0] << ", "  << hip[1] << ") to (" << desired_hip[0] << ", "  << desired_hip[1] << ") along the ground." << std::endl;
    

    // Eigen::VectorXd a(5);
    // Eigen::VectorXd b(5);
    // Eigen::VectorXd result(5);

    // // 初始化向量
    // a << 1.0, 2.0, 3.0, 4.0, 5.0;

    // b << 5.0, 4.0, 3.0, 2.0, 1.0;
    // a << 5.0, 2.0, 3.0, 4.0, 5.0;
    // std::cout << a ;

    return 0;
}//end main
