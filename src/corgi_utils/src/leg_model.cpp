#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <chrono>
#include <Eigen/Dense>

#include "leg_model.hpp"
#include "fitted_coefficient.hpp"

LegModel::LegModel(bool sim) {
    // Constants initialization
    max_theta = M_PI * 160.0 / 180.0;
    min_theta = M_PI * 17.0 / 180.0;
    theta0 = M_PI * 17.0 / 180.0;
    beta0 = M_PI * 90.0 / 180.0;

    // Wheel radius
    R = 0.1; // 10 cm
    if (sim) {
        r = 0.0125; // No tire
    } else {
        r = 0.019;  // With tire
    }
    radius = R + r;

    // Linkage parameters
    arc_HF = M_PI * 130.0 / 180.0;
    arc_BC = M_PI * 101.0 / 180.0;
    l1 = 0.8 * R;
    l2 = R - l1;
    l3 = 2.0 * R * sin(arc_BC / 2.0);
    l4 = 0.882966335 * R;
    l5 = 0.9 * R;
    l6 = 0.4 * R;
    l7 = 2.0 * R * sin((arc_HF - arc_BC - theta0) / 2.0);
    l8 = 2.0 * R * sin((M_PI - arc_HF) / 2.0);

    // Useful parameters
    l_AE = l5 + l6;
    l_BF = 2.0 * R * sin((arc_HF - theta0) / 2.0);
    l_BH = 2.0 * R * sin(theta0 / 2.0);
    ang_UBC = (M_PI - arc_BC) / 2.0;
    ang_LFG = (M_PI - (M_PI - arc_HF)) / 2.0;

    // Initialize positions
    this->forward(theta0, 0.0);
}

void LegModel::forward(double theta_in, double beta_in, bool vector) {
    theta = theta_in;
    beta = beta_in;

    // Limit theta
    if (theta > max_theta) {
        theta = max_theta;
        std::cout << "Theta exceeds upper limit. Set to max_theta.\n";
    }
    if (theta < min_theta) {
        theta = min_theta;
        std::cout << "Theta below lower limit. Set to min_theta.\n";
    }

    // Calculate positions
    this->calculate();
    this->rotate();
    if (vector) {
        this->to_vector();
    }
}

void LegModel::calculate() {
    using namespace std::complex_literals; // For 1i
    // Forward kinematics calculations
    A_l_c = l1 * std::exp(1i * theta);
    B_l_c = R * std::exp(1i * theta);
    ang_OEA = std::asin((l1 / l_AE) * sin(theta));
    E_c = l1 * cos(theta) - l_AE * cos(ang_OEA);
    D_l_c = E_c + l6 * std::exp(1i * ang_OEA);
    l_BD = std::abs(D_l_c - B_l_c);
    ang_DBC = std::acos((l_BD * l_BD + l3 * l3 - l4 * l4) / (2.0 * l_BD * l3));
    C_l_c = B_l_c + (D_l_c - B_l_c) * std::exp(-1i * ang_DBC) * (l3 / l_BD);
    ang_BCF = std::acos((l3 * l3 + l7 * l7 - l_BF * l_BF) / (2.0 * l3 * l7));
    F_l_c = C_l_c + (B_l_c - C_l_c) * std::exp(-1i * ang_BCF) * (l7 / l3);
    ang_OGF = std::asin(std::abs(F_l_c.imag()) / l8);
    G_c = F_l_c - l8 * std::exp(1i * ang_OGF);
    U_l_c = B_l_c + (C_l_c - B_l_c) * std::exp(1i * ang_UBC) * (R / l3);
    L_l_c = F_l_c + (G_c - F_l_c) * std::exp(1i * ang_LFG) * (R / l8);
    H_l_c = U_l_c + (B_l_c - U_l_c) * std::exp(-1i * theta0);
    this->symmetry();
}

void LegModel::symmetry() {
    // Symmetric positions
    A_r_c = std::conj(A_l_c);
    B_r_c = std::conj(B_l_c);
    C_r_c = std::conj(C_l_c);
    D_r_c = std::conj(D_l_c);
    F_r_c = std::conj(F_l_c);
    H_r_c = std::conj(H_l_c);
    U_r_c = std::conj(U_l_c);
    L_r_c = std::conj(L_l_c);
}

void LegModel::rotate() {
    using namespace std::complex_literals;
    std::complex<double> rot_ang = std::exp(1i * (beta + beta0));
    // Rotate positions
    A_l_c *= rot_ang;
    A_r_c *= rot_ang;
    B_l_c *= rot_ang;
    B_r_c *= rot_ang;
    C_l_c *= rot_ang;
    C_r_c *= rot_ang;
    D_l_c *= rot_ang;
    D_r_c *= rot_ang;
    E_c   *= rot_ang;
    F_l_c *= rot_ang;
    F_r_c *= rot_ang;
    G_c   *= rot_ang;
    H_l_c *= rot_ang;
    H_r_c *= rot_ang;
    U_l_c *= rot_ang;
    U_r_c *= rot_ang;
    L_l_c *= rot_ang;
    L_r_c *= rot_ang;
}

void LegModel::to_vector() {
    A_l = {A_l_c.real(), A_l_c.imag()};
    A_r = {A_r_c.real(), A_r_c.imag()};
    B_l = {B_l_c.real(), B_l_c.imag()};
    B_r = {B_r_c.real(), B_r_c.imag()};
    C_l = {C_l_c.real(), C_l_c.imag()};
    C_r = {C_r_c.real(), C_r_c.imag()};
    D_l = {D_l_c.real(), D_l_c.imag()};
    D_r = {D_r_c.real(), D_r_c.imag()};
    E   = {E_c.real()  , E_c.imag()};
    F_l = {F_l_c.real(), F_l_c.imag()};
    F_r = {F_r_c.real(), F_r_c.imag()};
    G   = {G_c.real()  , G_c.imag()};
    H_l = {H_l_c.real(), H_l_c.imag()};
    H_r = {H_r_c.real(), H_r_c.imag()};
    U_l = {U_l_c.real(), U_l_c.imag()};
    U_r = {U_r_c.real(), U_r_c.imag()};
    L_l = {L_l_c.real(), L_l_c.imag()};
    L_r = {L_r_c.real(), L_r_c.imag()};
}

void LegModel::contact_map(double theta_in, double beta_in, double slope) {
        using namespace std::complex_literals;
        double beta_adjusted = beta_in - slope;

        this->forward(theta_in, beta_adjusted, false);

        std::complex<double> G_l_tmp = (G_c - L_l_c) / R * radius + L_l_c;
        std::complex<double> G_r_tmp = (G_c - L_r_c) / R * radius + L_r_c;
        std::complex<double> H_l_tmp = (H_l_c - U_l_c) / R * radius + U_l_c;
        std::complex<double> H_r_tmp = (H_r_c - U_r_c) / R * radius + U_r_c;
        std::complex<double> F_l_tmp = (F_l_c - U_l_c) / R * radius + U_l_c;
        std::complex<double> F_r_tmp = (F_r_c - U_r_c) / R * radius + U_r_c;

        std::array<std::array<double, 2>, 6> arc_list = {
            this->arc_min(H_l_tmp, F_l_tmp, U_l_c, "left upper"),
            this->arc_min(F_l_tmp, G_l_tmp, L_l_c, "left lower"),
            this->arc_min(G_l_tmp, G_r_tmp, G_c, "G"),
            this->arc_min(G_r_tmp, F_r_tmp, L_r_c, "right lower"),
            this->arc_min(F_r_tmp, H_r_tmp, U_r_c, "right upper"),
            {0.0, 0.0}
        };

        double min_value = arc_list[0][0];
        int min_index = 0;
        for(int i=1; i<6; i++){
            if (arc_list[i][0] < min_value) {
                min_value = arc_list[i][0];
                min_index = i;
            }//end if
        }//end for

        rim = min_index==5? 0 : min_index+1;
        alpha = arc_list[min_index][1];
        height = -arc_list[min_index][0];
}//end contact_map

std::array<double, 2> LegModel::arc_min(const std::complex<double>& p1, const std::complex<double>& p2, const std::complex<double>& O, const std::string& rim) {
        using namespace std::complex_literals;
        double lowest_point = 0.0;
        double alpha = 0.0;
        double bias_alpha = 0.0;

        if (rim == "left upper") {
            // bias_alpha = -M_PI;
        } else if (rim == "left lower") {
            // bias_alpha = -M_PI / 3.6; // -50 degrees
        } else if (rim == "G") {
            std::complex<double> direction_G = p1 + p2;
            bias_alpha = std::arg((p1 - O) / direction_G);
        } else if (rim == "right lower") {
            // bias_alpha = 0.0;
        } else if (rim == "right upper") {
            // bias_alpha = M_PI / 3.6; // 50 degrees
        }//end if else

        double cal_err = 1e-9;
        bool in_range = ((p2 - O).real() >= -cal_err) && ((p1 - O).real() <= cal_err);

        if (in_range) {
            lowest_point = O.imag() - radius;
            alpha = std::arg(-1i / (p1 - O));
        } else {
            std::complex<double> smaller = (p1.imag() < p2.imag()) ? p1 : p2;
            lowest_point = 1.0; // Set to a large value if not normal contact
            alpha = std::arg((smaller - O) / (p1 - O));
        }//end if else

        return {lowest_point, alpha + bias_alpha};
}//end arc_min

// Note: The inverse and move functions require root-finding and numerical methods that are complex to implement.
// For a complete implementation, you would need to use numerical libraries like Eigen, Ceres Solver, or write custom solvers.
std::array<double, 2> LegModel::inverse(const double pos[2], const std::string &joint) {
    using namespace std::complex_literals;
    double abs_pos = std::sqrt(pos[0]*pos[0] + pos[1]*pos[1]);
    if (joint == "G"){
        theta = inv_G_dist_poly(abs_pos);
        beta = std::atan2(pos[1], pos[0]) - std::atan2(-abs_pos, 0);    // atan2(y, x)
    } else if (joint == "Ul" || joint == "Ur"){
        theta = inv_U_dist_poly(abs_pos);
        double U_x_beta0, U_y_beta0;
        if (joint == "Ul"){
            U_x_beta0 = U_l_poly[0](theta);
            U_y_beta0 = U_l_poly[1](theta);
        } else {    // Ur
            U_x_beta0 = U_r_poly[0](theta);
            U_y_beta0 = U_r_poly[1](theta);    
        }//end if else
        beta = std::atan2(pos[1], pos[0]) - std::atan2(U_y_beta0, U_x_beta0);    // atan2(y, x)
    } else if (joint == "Ll" || joint == "Lr"){
        theta = inv_L_dist_poly(abs_pos);
        double L_x_beta0, L_y_beta0;
        if (joint == "Ll"){
            L_x_beta0 = L_l_poly[0](theta);
            L_y_beta0 = L_l_poly[1](theta);
        } else {    // Lr
            L_x_beta0 = L_r_poly[0](theta);
            L_y_beta0 = L_r_poly[1](theta);  
        }//end if else            
        beta = std::atan2(pos[1], pos[0]) - std::atan2(L_y_beta0, L_x_beta0);    // atan2(y, x)
    } else {
        throw std::runtime_error("joint needs to be 'G', 'Ul', 'Ur', 'Ll', or 'Lr'.");
    }//end if else

    return {theta, beta};
}//end inverse

std::array<double, 2> LegModel::move(double theta_in, double beta_in, const std::array<double, 2>& move_vec, bool contact_upper, double tol, size_t max_iter) {
    this->contact_map(theta_in, beta_in);
    
    // Contact point logic
    int contact_rim;
    if (contact_upper) {
        contact_rim = rim;
    } else {
        if (rim == 2 || rim == 3 || rim == 4) {
            contact_rim = rim;
        } else if (beta > 0) {
            contact_rim = 2;
        } else {
            contact_rim = 4;
        }//end if else
    }//end if else

    // Use optimization solver to find d_theta and d_beta (analogous to fsolve)
    std::array<double, 2> guess_dq = {0.0, 0.0};    // d_theta, d_beta / initial guess = (0, 0)
    for (size_t iter = 0; iter < max_iter; ++iter) {
        std::array<double, 2> cost = this->objective(guess_dq, {theta_in, beta_in}, move_vec, contact_rim);     // 计算当前函数值
        Eigen::Vector2d cost_vec(cost[0], cost[1]);

        double norm_cost = cost_vec.norm();          // 计算残差范数
        if (norm_cost < tol) {                // 判断收敛
            //std::cout << "Converged after " << iter << " iterations.\n";
            break;
        }//end if

        // computeJacobian, 数值计算雅可比矩阵
        double epsilon = 1e-6;
        Eigen::Matrix2d Jac;
        for (size_t i = 0; i < 2; ++i) {
            std::array<double, 2> dq_eps = guess_dq;
            dq_eps[i] += epsilon;  // 对第 i 个变量加一个小扰动
            std::array<double, 2> cost_eps = this->objective(dq_eps, {theta_in, beta_in}, move_vec, contact_rim);
            Eigen::Vector2d cost_eps_vec(cost_eps[0], cost_eps[1]);
            Jac.col(i) = (cost_eps_vec - cost_vec) / epsilon;  // 数值差分计算导数
        }//end for

        Eigen::Vector2d dq = Jac.partialPivLu().solve(-cost_vec);   // 解线性方程 Jac * dq = -cost_vec

        if (dq.norm() < tol) {             // 判断步长是否足够小
            //std::cout << "Converged after " << iter << " iterations.\n";
            break;
        }//end if

        // 更新解
        guess_dq[0] += dq[0];
        guess_dq[1] += dq[1];

        if (iter == max_iter-1) {
            throw std::runtime_error("Newton solver did not converge.");
        }//end if
    }//end for

    // update theta, beta
    theta += guess_dq[0];
    beta  += guess_dq[1];
    return {theta, beta};
}//end move

std::array<double, 2> LegModel::objective(const std::array<double, 2>& d_q, const std::array<double, 2>& current_q, const std::array<double, 2>& move_vec, int contact_rim) {
    using namespace std::complex_literals;
    std::array<double, 2> guessed_q = {current_q[0] + d_q[0], current_q[1] + d_q[1]};
    
    std::complex<double> current_F_exp, current_G_exp, current_U_exp, current_L_exp, 
                        guessed_F_exp, guessed_G_exp, guessed_U_exp, guessed_L_exp;
    std::array<double, 2> guessed_hip;
    if (contact_rim == 1) {
        // Left upper rim 
        current_F_exp = ( F_l_poly[0](current_q[0])+1i*F_l_poly[1](current_q[0]) ) *std::exp( std::complex<double>(0, current_q[1]) );
        current_U_exp = ( U_l_poly[0](current_q[0])+1i*U_l_poly[1](current_q[0]) ) *std::exp( std::complex<double>(0, current_q[1]) );
        guessed_F_exp = ( F_l_poly[0](guessed_q[0])+1i*F_l_poly[1](guessed_q[0]) ) *std::exp( std::complex<double>(0, guessed_q[1]) );
        guessed_U_exp = ( U_l_poly[0](guessed_q[0])+1i*U_l_poly[1](guessed_q[0]) ) *std::exp( std::complex<double>(0, guessed_q[1]) );
        double d_alpha = std::arg( -1i/(guessed_F_exp - guessed_U_exp) ) - std::arg( -1i/(current_F_exp - current_U_exp) );
        double roll_d = d_alpha * radius;
        std::array<double, 2> next_U = {current_U_exp.real() + roll_d, current_U_exp.imag()};
        guessed_hip = {next_U[0] - guessed_U_exp.real(), next_U[1] - guessed_U_exp.imag()}; // next_U - guessed_U
    } else if (contact_rim == 2) {
        // Left lower rim 
        current_G_exp = 1i*G_poly[1](current_q[0]) *std::exp( std::complex<double>(0, current_q[1]) );
        current_L_exp = ( L_l_poly[0](current_q[0])+1i*L_l_poly[1](current_q[0]) ) *std::exp( std::complex<double>(0, current_q[1]) );
        guessed_G_exp = 1i*G_poly[1](guessed_q[0]) *std::exp( std::complex<double>(0, guessed_q[1]) );
        guessed_L_exp = ( L_l_poly[0](guessed_q[0])+1i*L_l_poly[1](guessed_q[0]) ) *std::exp( std::complex<double>(0, guessed_q[1]) );
        double d_alpha = std::arg( -1i/(guessed_G_exp - guessed_L_exp) ) - std::arg( -1i/(current_G_exp - current_L_exp) );
        double roll_d = d_alpha * radius;
        std::array<double, 2> next_L = {current_L_exp.real() + roll_d, current_L_exp.imag()};
        guessed_hip = {next_L[0] - guessed_L_exp.real(), next_L[1] - guessed_L_exp.imag()}; // next_L - guessed_L
    } else if (contact_rim == 3) {
        // G
        current_G_exp = 1i*G_poly[1](current_q[0]) *std::exp( std::complex<double>(0, current_q[1]) );
        guessed_G_exp = 1i*G_poly[1](guessed_q[0]) *std::exp( std::complex<double>(0, guessed_q[1]) );
        double roll_d = -d_q[1] * r;
        std::array<double, 2> next_G = {current_G_exp.real() + roll_d, current_G_exp.imag()};
        guessed_hip = {next_G[0] - guessed_G_exp.real(), next_G[1] - guessed_G_exp.imag()}; // next_G - guessed_G
    } else if (contact_rim == 4) {
        // Right lower rim 
        current_G_exp = 1i*G_poly[1](current_q[0]) *std::exp( std::complex<double>(0, current_q[1]) );
        current_L_exp = ( L_r_poly[0](current_q[0])+1i*L_r_poly[1](current_q[0]) ) *std::exp( std::complex<double>(0, current_q[1]) );
        guessed_G_exp = 1i*G_poly[1](guessed_q[0]) *std::exp( std::complex<double>(0, guessed_q[1]) );
        guessed_L_exp = ( L_r_poly[0](guessed_q[0])+1i*L_r_poly[1](guessed_q[0]) ) *std::exp( std::complex<double>(0, guessed_q[1]) );
        double d_alpha = std::arg( -1i/(guessed_G_exp - guessed_L_exp) ) - std::arg( -1i/(current_G_exp - current_L_exp) );
        double roll_d = d_alpha * radius;
        std::array<double, 2> next_L = {current_L_exp.real() + roll_d, current_L_exp.imag()};
        guessed_hip = {next_L[0] - guessed_L_exp.real(), next_L[1] - guessed_L_exp.imag()}; // next_L - guessed_L
    } else if (contact_rim == 5) {
        // Right upper rim 
        current_F_exp = ( F_r_poly[0](current_q[0])+1i*F_r_poly[1](current_q[0]) ) *std::exp( std::complex<double>(0, current_q[1]) );
        current_U_exp = ( U_r_poly[0](current_q[0])+1i*U_r_poly[1](current_q[0]) ) *std::exp( std::complex<double>(0, current_q[1]) );
        guessed_F_exp = ( F_r_poly[0](guessed_q[0])+1i*F_r_poly[1](guessed_q[0]) ) *std::exp( std::complex<double>(0, guessed_q[1]) );
        guessed_U_exp = ( U_r_poly[0](guessed_q[0])+1i*U_r_poly[1](guessed_q[0]) ) *std::exp( std::complex<double>(0, guessed_q[1]) );
        double d_alpha = std::arg( -1i/(guessed_F_exp - guessed_U_exp) ) - std::arg( -1i/(current_F_exp - current_U_exp) );
        double roll_d = d_alpha * radius;
        std::array<double, 2> next_U = {current_U_exp.real() + roll_d, current_U_exp.imag()};
        guessed_hip = {next_U[0] - guessed_U_exp.real(), next_U[1] - guessed_U_exp.imag()}; // next_U - guessed_U
    } else {
        throw std::runtime_error("The leg doesn't contact ground.");
    }//end if else
    
    // Return the result of the objective function
    return {guessed_hip[0] - move_vec[0], guessed_hip[1] - move_vec[1]};
}//end objective



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

    /* Inverse kinematics */
    std::cout << "\n";
    std::cout << "****************************************" << std::endl;
    std::cout << "****** Inverse kinematics example ******" << std::endl;
    std::cout << "****************************************" << std::endl;
    // inverse for G
    std::cout << "==========Inverse for G==========" << std::endl;
    double G_p[2] = {0.05, -0.25};
    std::cout << "Input G: " << G_p[0] << ", " << G_p[1] << std::endl;
    new_theta_beta = legmodel.inverse(G_p, "G");
    legmodel.forward(new_theta_beta[0], new_theta_beta[1]);
    std::cout << "Output theta, beta (degree): " << new_theta_beta[0]*180.0/M_PI << ", "<< new_theta_beta[1]*180.0/M_PI << std::endl;
    std::cout << "Output G: " << legmodel.G[0] << ", " << legmodel.G[1] << std::endl;
    // inverse for left upper rim
    std::cout << "==========Inverse for U_l==========" << std::endl;
    double Ul_p[2] = {-0.01, -0.015};
    std::cout << "Input U_l: " << Ul_p[0] << ", " << Ul_p[1] << std::endl;
    new_theta_beta = legmodel.inverse(Ul_p, "Ul");
    legmodel.forward(new_theta_beta[0], new_theta_beta[1]);
    std::cout << "Output theta, beta (degree): " << new_theta_beta[0]*180.0/M_PI << ", "<< new_theta_beta[1]*180.0/M_PI << std::endl;
    std::cout << "Output U_l: " << legmodel.U_l[0] << ", " << legmodel.U_l[1] << std::endl;
    // inverse for right lower rim
    std::cout << "==========Inverse for L_r==========" << std::endl;
    double Lr_p[2] = {-0.01, -0.015};
    std::cout << "Input L_r: " << Lr_p[0] << ", " << Lr_p[1] << std::endl;
    new_theta_beta = legmodel.inverse(Lr_p, "Lr");
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
