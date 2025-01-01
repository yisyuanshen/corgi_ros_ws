/**
 * @file LinkLegModel.cpp
 * 
 * @author peichunhuang
 */
#include "kinematic/LinkLegModel.hpp"
void LinkLegModel::calculate(float theta, float theta_d, float theta_dd)
{
    this->theta = theta;
    this->theta_d = theta_d;
    this->theta_dd = theta_dd;

    this->phi = asin(this->l1 * sin(this->theta) / this->l3);
    this->D_phi();
    this->A = this->l1 * std::polar(1.f, this->theta);
    this->B = this->R * std::polar(1.f, this->theta);
    this->oe = this->l1 * cos(this->theta) - this->l3 * cos(this->phi);
    this->E = this->oe * exp(0);
    this->D_oe();
    this->D = this->E + this->l4 * std::polar(1.f, this->phi);
    float db_2 = this->l2 * this->l2 + this->l6 * this->l6 - 2 * this->l2 * this->l6 * cos(M_PI_F - this->theta + this->phi);
    this->db = sqrt(db_2);
    this->D_db();
    this->epsilon = atan2(this->l2 * sin(this->phi) + this->l6 * sin(this->theta), this->l2 * cos(this->phi) + this->l6 * cos(this->theta));
    this->D_epsilon();
    this->theta2 = acos((db_2 + this->l7 * this->l7 - this->l5 * this->l5) / (2.0 * this->db * this->l7));
    this->D_theta2();
    this->to = M_PI_F - this->theta2 + this->epsilon;
    this->C = this->B + this->l7 * std::polar(1.f, this->to);
    this->O1 = this->B + this->R * std::polar(1.f, this->to + this->to1);
    this->D_O1();
    this->F = this->B + this->l8 * std::polar(1.f, this->to + this->tf);
    this->H = this->B + this->l9 * std::polar(1.f, this->to + this->th);
    this->rho = asin((this->R * sin(this->theta) + this->l8 * sin(this->to + this->tf)) / this->l10);
    this->D_rho();
    this->G = this->F - this->l10 * std::polar(1.f, this->rho);
    this->D_G();
    this->O2 = this->G + this->R * std::polar(1.f, this->rho + this->to2);
    this->D_O2();
    this->symmetry();
}
void LinkLegModel::D_phi()
{
    float k = this->l1 / this->l3;
    float ph = k * cos(this->theta) / cos(k * sin(this->theta));
    this->phi_d = ph * this->theta_d;
    this->phi_dd = ph *  this->theta_dd + k * this->theta_d * (-sin(this->theta) * cos(k * sin(this->theta)) + k * cos(this->theta) * cos(this->theta) * (sin(k * sin(this->theta)))) 
    / cos(k * sin(this->theta)) / cos(k * sin(this->theta));
}
void LinkLegModel::D_oe()
{
    this->oe_d = -this->l1 * sin(this->theta) * this->theta_d + this->l3 * sin(this->phi) * this->phi_d;
    this->oe_dd = -this->l1 * sin(this->theta) * this->theta_dd + this->l3 * sin(this->phi) * this->phi_dd - this->l1 * cos(this->theta) * this->theta_d * this->theta_d + this->l3 * cos(this->phi) * this->phi_d * this->phi_d;
}
void LinkLegModel::D_db()
{
    float db2 = this->l2 * this->l6 * sin(M_PI_F - this->theta + this->phi);
    this->db_d = db2 * (-this->theta_d + this->phi_d) / this->db;
    this->db_dd = db2  * (-this->theta_dd + this->phi_dd) / this->db + (-this->theta_d + this->phi_d) * this->l2 * this->l6 * cos(M_PI_F - this->theta + this->phi) * (-this->theta_d + this->phi_d) / this->db - this->db_d  * (-this->theta_d + this->phi_d) * db2 / this->db / this->db;
}
void LinkLegModel::D_epsilon()
{
    this->epsilon_d = (this->l2 * this->l2 * this->phi_d + this->l6 * this->l6 * this->theta_d + this->l2 * this->l6 * cos(this->phi - this->theta) * (this->phi_d + this->theta_d)) / this->db / this->db;
    this->epsilon_dd = ((this->l2 * this->l2 * this->phi_dd + this->l6 * this->l6 * this->theta_dd + this->l2 * this->l6 * cos(this->phi - this->theta) * (this->phi_dd + this->theta_dd) - this->l2 * this->l6 * sin(this->phi - this->theta) * (this->phi - this->theta) * (this->phi_d + this->theta_d)) * this->db * this->db 
    - (this->l2 * this->l2 * this->phi_d + this->l6 * this->l6 * this->theta_d + this->l2 * this->l6 * cos(this->phi - this->theta) * (this->phi_d + this->theta_d)) * 2 * this->db * this->db_d) / pow(this->db, 4);
}
void LinkLegModel::D_theta2()
{
    float num = this->db_d * this->l7 * (4 * this->db * this->db - 2 * (this->db * this->db + this->l7 * this->l7 - this->l5 * this->l5));
    float num_part = (2 * this->db * this->l7) * (2 * this->db * this->l7);
    float den_sq = num_part * num_part + (this->db * this->db + this->l7 * this->l7 - this->l5 * this->l5) * (this->db * this->db + this->l7 * this->l7 - this->l5 * this->l5) * num_part;
    float num_dt = 4 * this->db * this->db_d * this->db_d * this->l7 + this->db_dd * this->l7 * (2 * this->db * this->db + this->l7 * this->l7 - this->l5 * this->l5);
    float den_dt = 64 * this->db * this->db * this->db * std::pow(this->l7, 4) * this->db_d + 8 * (this->db * this->db + this->l7 * this->l7 - this->l5 * this->l5) * this->db * this->db_d * num_part + 
    (this->db * this->db + this->l7 * this->l7 - this->l5 * this->l5) * (this->db * this->db + this->l7 * this->l7 - this->l5 * this->l5) * 8 * this->l7 * this->l7 * this->db * this->db_d;
    this->theta2_d = num / sqrt(den_sq);
    this->theta2_dd = (sqrt(den_sq) * num_dt - num * 0.5 * den_dt / sqrt(den_sq)) / den_sq;
}
void LinkLegModel::D_rho()
{
    float num = this->R * cos(this->theta) * this->theta_d + this->l8 * (-this->theta2_d + this->epsilon_d) * cos(this->to + this->tf);
    float den_sq = this->l10 * this->l10 - (this->R * sin(this->theta) + this->l8 * sin(this->to + this->tf)) * (this->R * sin(this->theta) + this->l8 * sin(this->to + this->tf));
    float num_dt = -this->R * sin(this->theta) * this->theta_d * this->theta_d + this->R * cos(this->theta) * this->theta_dd + this->l8 * (-this->theta2_dd + this->epsilon_dd) * cos(this->to + this->tf) - this->l8 * (-this->theta2_d + this->epsilon_d) * (-this->theta2_d + this->epsilon_d) * sin(this->to + this->tf);
    this->rho_d = num / sqrt(den_sq);
    this->rho_dd = (sqrt(den_sq) * num_dt + num * num * (this->R * sin(this->theta) + this->l8 * sin(this->to + this->tf)) / sqrt(den_sq)) / den_sq;
}
void LinkLegModel::D_O1()
{
    this->O1_d = this->R * (this->theta_d) * std::polar(1.f, (this->theta + M_PI_2_F)) + this->R * (this->epsilon_d - this->theta2_d) * std::polar(1.f, (this->to + this->to1 + M_PI_2_F));
    this->O1_dd = this->R * (this->theta_dd) * std::polar(1.f, this->theta + M_PI_2_F) + this->R * (this->epsilon_dd - this->theta2_dd) * std::polar(1.f, this->to + this->to1 + M_PI_2_F) - this->R * (this->theta_d) * (this->theta_d) * std::polar(1.f, this->theta) - this->R * (this->epsilon_d - this->theta2_d) * (this->epsilon_d - this->theta2_d) * std::polar(1.f, this->to + this->to1);
}
void LinkLegModel::D_G()
{
    this->G_d = this->R * this->theta_d * std::polar(1.f, this->theta + M_PI_2_F) + this->l8 * (this->epsilon_d - this->theta2_d) * std::polar(1.f, this->to + this->tf + M_PI_2_F) - this->l10 * this->rho_d * std::polar(1.f, this->rho + M_PI_2_F);
    this->G_dd = this->R * this->theta_dd * std::polar(1.f, this->theta + M_PI_2_F) + this->l8 * (this->epsilon_dd - this->theta2_dd) * std::polar(1.f, this->to + this->tf + M_PI_2_F) - this->l10 * this->rho_dd * std::polar(1.f, this->rho + M_PI_2_F) - this->R * this->theta_d * this->theta_d * std::polar(1.f, this->theta ) - this->l8 * (this->epsilon_d - this->theta2_d) * (this->epsilon_d - this->theta2_d) * std::polar(1.f, this->to + this->tf ) + this->l10 * this->rho_d * this->rho_d * std::polar(1.f, this->rho );
}
void LinkLegModel::D_O2()
{
    this->O2_d = this->G_d + this->R * this->rho_d * std::polar(1.f, this->rho + this->to2 + M_PI_2_F);
    this->O2_dd = this->G_dd + this->R * this->rho_dd * std::polar(1.f, this->rho + this->to2 + M_PI_2_F) - this->R * this->rho_d * this->rho_d * std::polar(1.f, this->rho + this->to2);
}
void LinkLegModel::symmetry()
{
    this->O2_ = std::conj(this->O2);
    this->O2_d_ = std::conj(this->O2_d);
    this->O2_dd_ = std::conj(this->O2_dd);
    this->O1_ = std::conj(this->O1);
    this->O1_d_ = std::conj(this->O1_d);
    this->O1_dd_ = std::conj(this->O1_dd);
    this->F_ = std::conj(this->F);
    this->H_ = std::conj(this->H);
    this->O1_w = -this->theta2_d + this->epsilon_d;
    this->O1_w_ = -this->O1_w;
    this->O2_w = this->rho_d;
    this->O2_w_ = -this->O2_w;
    this->O1_w_d = -this->theta2_dd + this->epsilon_dd;
    this->O1_w_d_ = -this->O1_w_d;
    this->O2_w_d = this->rho_dd;
    this->O2_w_d_ = -this->O2_w_d;
}

float LinkLegModel::inverse(float r, RIM rim) {
    std::call_once(flag, [this]() {
        this->calculate(this->max_theta, 0, 0);
        this->max_r_G = -this->G.real();
        return true;
    });
    float theta_optimal = to_radians(17.0); 
    switch (rim)
    {
        case UPPER_RIM_R: // right upper ring

        break;
        case UPPER_RIM_L: // left upper ring

        break;
        case LOWER_RIM_R: // right downward ring

        break;
        case LOWER_RIM_L: // left downward ring

        break;
        case G_POINT: // G 
            // inverse function was too complicated, use iteration to find inverse,
            // and wa can find that the function of G and theta almost linear in our working space (0, 145).
            // use linear function as initial guess. Since the function was simple, we can easily get its optimal direction.
            if (r < this->R) r = R;
            else if (r > max_r_G) r = max_r_G;
            theta_optimal = (r - this->R) / (max_r_G - this->R) * (max_theta - min_theta) + min_theta;
            // std::cout << theta_optimal << "\n";
            for (int i = 0; i < 10; i ++) {
                this->calculate(theta_optimal, 0, 0);
                theta_optimal += (-this->G.real() - r);
            }
            if (theta_optimal > max_theta) theta_optimal = max_theta;
            else if (theta_optimal < min_theta) theta_optimal = min_theta;
        break;
    }
    return theta_optimal;
}