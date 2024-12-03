#include <iostream>
#include <vector>
#include <cmath>

std::vector<std::vector<double>> calculate_inv_T(const std::vector<std::vector<double>>& matrix) {
    double determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];

    std::vector<std::vector<double>> matrix_inverse(2, std::vector<double>(2));
    matrix_inverse[0][0] =  matrix[1][1] / determinant;
    matrix_inverse[0][1] = -matrix[0][1] / determinant;
    matrix_inverse[1][0] = -matrix[1][0] / determinant;
    matrix_inverse[1][1] =  matrix[0][0] / determinant;

    std::vector<std::vector<double>> matrix_inverse_transpose(2, std::vector<double>(2));
    matrix_inverse_transpose[0][0] = matrix_inverse[0][0];
    matrix_inverse_transpose[0][1] = matrix_inverse[1][0];
    matrix_inverse_transpose[1][0] = matrix_inverse[0][1];
    matrix_inverse_transpose[1][1] = matrix_inverse[1][1];

    return matrix_inverse_transpose;
}


// calculate contact position on the rim




// calculate jacobian matrix on the contact point
std::vector<std::vector<double>> calculate_jacobian(double theta, double beta) {

    double dtheta_dphiR = -1.0 / 2.0;
    double dtheta_dphiL =  1.0 / 2.0;
    double dbeta_dphiR  =  1.0 / 2.0;
    double dbeta_dphiL  =  1.0 / 2.0;

    double cos_beta = cos(beta);
    double sin_beta = sin(beta);

    // polynomial P_theta = {};

    // double dPx_dtheta = P_theta[0].deriv()(theta)*cos_beta - P_theta[1].deriv()(theta)*sin_beta;
    // double dPy_dtheta = P_theta[0].deriv()(theta)*sin_beta + P_theta[1].deriv()(theta)*cos_beta;
    // double dPx_dbeta = P_theta[0](theta)*(-sin_beta) - P_theta[1]*cos_beta;
    // double dPy_dbeta = P_theta[0](theta)*cos_beta + P_theta[1]*(-sin_beta);

    double dPx_dtheta = 0;
    double dPy_dtheta = 1;
    double dPx_dbeta  = 1;
    double dPy_dbeta  = 0;

    double J11 = dPx_dtheta * dtheta_dphiR + dPx_dbeta * dbeta_dphiR;
    double J12 = dPx_dtheta * dtheta_dphiL + dPx_dbeta * dbeta_dphiL;
    double J21 = dPy_dtheta * dtheta_dphiR + dPy_dbeta * dbeta_dphiR;
    double J22 = dPy_dtheta * dtheta_dphiL + dPy_dbeta * dbeta_dphiL;

    return {{J11, J12}, {J21, J22}};
}

// calculate the torque caused by inertia through dynamic model
std::vector<double> compensate_torque(double theta, double beta, double theta_dot, double beta_dot, double theta_ddot, double beta_ddot){
    std::vector<double> Rm_coef = {-0.0035, 0.0110, 0.0030, 0.0500, -0.0132};
    std::vector<double> Ic_coef = {0.0001, -0.0001, -0.0013, 0.0043, 0.0041};

    double m = 0.65;
    double g = 9.81;

    double Rm = 0, Rm_dot = 0, Rm_ddot = 0;
    double Ic = 0, Ic_dot = 0, Ic_ddot = 0;

    for (int i = 0; i < 5; i++) {
        Rm += Rm_coef[i] * pow(theta, i);
        Ic += Ic_coef[i] * pow(theta, i);
    }

    for (int i = 1; i < 5; i++) {
        Rm_dot += i * Rm_coef[i] * pow(theta, i - 1);
        Ic_dot += i * Ic_coef[i] * pow(theta, i - 1);
    }

    for (int i = 2; i < 5; i++) {
        Rm_ddot += i * (i - 1) * Rm_coef[i] * pow(theta, i - 2);
        Ic_ddot += i * (i - 1) * Ic_coef[i] * pow(theta, i - 2);
    }


    std::vector<std::vector<double>> M = {
        {m, 0},
        {0, Ic + m * Rm * Rm}
    };

    std::vector<double> acceleration = {Rm_ddot, beta_ddot};

    std::vector<double> coriolis = {
        -m * Rm * beta_dot * beta_dot,
        2 * m * Rm * Rm_ddot * beta_dot + Ic * beta_dot
    };

    std::vector<double> gravity = {
        -m * g * cos(beta),
        m * g * Rm * sin(beta)
    };

    double f_Rm = M[0][0] * acceleration[0] + coriolis[0] + gravity[0];
    double tau_beta = M[1][1] * acceleration[1] + coriolis[1] + gravity[1];

    std::vector<std::vector<double>> jacobian_phi = {{-0.5, 0.5},
                                                     { 0.5, 0.5}};
    std::vector<std::vector<double>> jacobian_theta = {{Rm_dot, 0},
                                                       {0,      1}};

    std::vector<std::vector<double>> jacobian_phi_theta = {{jacobian_phi[0][0]*jacobian_theta[0][0]+jacobian_phi[0][1]*jacobian_theta[1][0],
                                                            jacobian_phi[0][0]*jacobian_theta[0][1]+jacobian_phi[0][1]*jacobian_theta[1][1]},
                                                           {jacobian_phi[1][0]*jacobian_theta[0][0]+jacobian_phi[1][1]*jacobian_theta[1][0],
                                                            jacobian_phi[1][0]*jacobian_theta[0][1]+jacobian_phi[1][1]*jacobian_theta[1][1]}};

    double tau_phiR = jacobian_phi_theta[0][0]*f_Rm+jacobian_phi_theta[1][0]*tau_beta;
    double tau_phiL = jacobian_phi_theta[0][1]*f_Rm+jacobian_phi_theta[1][1]*tau_beta;

    return {tau_phiR, tau_phiL};
}



// calculate the contact force by virtual work method
std::vector<double> estimate_force(std::vector<std::vector<double>> jacobian_inv_T, std::vector<double> tau) {
    std::vector<double> f_est = {0, 0};

    f_est[0] = jacobian_inv_T[0][0] * tau[0] + jacobian_inv_T[0][1] * tau[1];
    f_est[1] = jacobian_inv_T[1][0] * tau[0] + jacobian_inv_T[1][1] * tau[1];

    return f_est;
}




int main(int argc, char **argv) {
    std::cout << "Force Estimation Node" << std::endl;

    double theta = 17;
    double beta = 10;

    std::cout << "\ntheta = " << theta
              << "\nbeta  = " << beta
              << std::endl;


    std::vector<std::vector<double>> jacobian = calculate_jacobian(theta, beta);
    std::vector<std::vector<double>> jacobian_inv_T = calculate_inv_T(jacobian);

    std::cout << "\nJacobian Matrix:\n[[" << jacobian_inv_T[0][0] << ", " << jacobian_inv_T[0][1]
                               << "]\n [" << jacobian_inv_T[1][0] << ", " << jacobian_inv_T[1][1] 
                               << "]]" << std::endl;


    std::vector<double> tau = {1, 1};
    std::vector<double> tau_phi = compensate_torque(theta, beta, 0, 0, 0, 0);

    tau[0] -= tau_phi[0];
    tau[1] -= tau_phi[1];

    std::vector<double> f_est = estimate_force(jacobian_inv_T, tau);

    std::cout << "\nEstimated Force:\n[" << f_est[0] << ", " << f_est[0] << "]" << std::endl;

    return 0;
}