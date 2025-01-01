#ifndef LINKLEGMODEL_HPP
#define LINKLEGMODEL_HPP
#include <cmath>
#include <complex>
#include <iostream>
#include <mutex>

#define M_PI_2_F 1.57079632679489661923132169163975144f

#define M_PI_F 3.14159265358979323846 
enum RIM
{
    UPPER_RIM_L = 1, // O1_
    LOWER_RIM_L = 2, // O2_
    G_POINT = 3, // G
    LOWER_RIM_R = 4, // O2
    UPPER_RIM_R = 5, // O1
    NO_CONTACT = 0
};

inline float to_degrees(float radians) {
    return radians * (180.0 / M_PI_F);
}
inline float to_radians(float degrees) {
    return degrees * M_PI_F / 180.0;
}
class LinkLegModel
{
    public:
    LinkLegModel(float r = 0.0125, float R = 0.1) {
        this->R = R;
        this->r = r;
        this->l1 = 0.8 * this->R;
        this->l2 = 0.9 * this->R;
        this->l3 = 1.3 * this->R;
        this->l4 = 0.4 * this->R;
        this->l5 = 0.88296634 * this->R;
        this->l6 = 0.2 * this->R;
        this->l7 = 2.0 * this->R * sin(to_radians(101.0 / 2.0));
        this->l8 = 2.0 * this->R * sin(to_radians(113.0 / 2.0));
        this->l9 = 2.0 * this->R * sin(to_radians(17.0 / 2.0));
        this->l10 = 2.0 * this->R * sin(to_radians(50.0 / 2.0));
    }
    void calculate(float theta, float theta_d, float theta_dd);
    float inverse(float r, RIM rim);

    std::complex<float> A;
    std::complex<float> B;
    std::complex<float> C;
    std::complex<float> D;
    std::complex<float> F;
    std::complex<float> H;
    std::complex<float> O1;
    std::complex<float> O2;
    std::complex<float> A_;
    std::complex<float> B_;
    std::complex<float> C_;
    std::complex<float> D_;
    std::complex<float> F_;
    std::complex<float> H_;
    std::complex<float> O1_;
    std::complex<float> O2_;
    std::complex<float> E;
    std::complex<float> G;

    std::complex<float> A_d;
    std::complex<float> B_d;
    std::complex<float> C_d;
    std::complex<float> D_d;
    std::complex<float> F_d;
    std::complex<float> O1_d;
    std::complex<float> O2_d;
    std::complex<float> A_d_;
    std::complex<float> B_d_;
    std::complex<float> C_d_;
    std::complex<float> D_d_;
    std::complex<float> F_d_;
    std::complex<float> O1_d_;
    std::complex<float> O2_d_;
    std::complex<float> E_d;
    std::complex<float> G_d;

    std::complex<float> A_dd;
    std::complex<float> B_dd;
    std::complex<float> C_dd;
    std::complex<float> D_dd;
    std::complex<float> F_dd;
    std::complex<float> O1_dd;
    std::complex<float> O2_dd;
    std::complex<float> A_dd_;
    std::complex<float> B_dd_;
    std::complex<float> C_dd_;
    std::complex<float> D_dd_;
    std::complex<float> F_dd_;
    std::complex<float> O1_dd_;
    std::complex<float> O2_dd_;
    std::complex<float> E_dd;
    std::complex<float> G_dd;

    float O1_w = 0;
    float O2_w = 0;
    float O1_w_ = 0;
    float O2_w_ = 0;
    float O1_w_d = 0;
    float O2_w_d = 0;
    float O1_w_d_ = 0;
    float O2_w_d_ = 0;

    float R;
    float r;
    float l1;
    float l2;
    float l3;
    float l4;
    float l5;
    float l6;
    float l7;
    float l8;
    float l9;
    float l10;

    const float min_theta = to_radians(17.0);
    const float max_theta = to_radians(140.0);
    float max_r_G = 1.;
    std::once_flag flag;
    const float to1 = to_radians(39.5);
    const float to2 = - to_radians(65.0);
    const float tf = to_radians(6.0);
    const float th = to_radians(121.0);

    float oe = 0;
    float oe_d = 0;
    float oe_dd = 0;

    float db = 0;
    float db_d = 0;
    float db_dd = 0;
    
    float theta = to_radians(17.0);
    float theta_d = 0;
    float theta_dd = 0;

    float phi = 0;
    float phi_d = 0;
    float phi_dd = 0;
    
    float epsilon = 0;
    float epsilon_d = 0;
    float epsilon_dd = 0;
    
    float theta2 = 0;
    float theta2_d = 0;
    float theta2_dd = 0;

    float rho = 0;
    float rho_d = 0;
    float rho_dd = 0;

    float to = 0;

    void D_phi();
    void D_oe();
    void D_db();
    void D_epsilon();
    void D_theta2();
    void D_rho();
    void D_O1();
    void D_G();
    void D_O2();
    void symmetry();
};
#endif