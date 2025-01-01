#ifndef LEG_HPP
#define LEG_HPP
#include <Eigen/Dense>
#include "LinkLegModel.hpp"

class Leg : public LinkLegModel
{
    public:
    Leg(Eigen::Vector3f offset, float R = 0.1, float r = 0.0125)
    : LinkLegModel(r, R)
    {
        this->offset = offset;
    }
    void Calculate(float theta, float theta_d, float theta_dd, float beta, float beta_d, float beta_dd);
    Eigen::Vector3f RimCentorVelocity(Eigen::Vector3f v, Eigen::Vector3f w, RIM rim);
    Eigen::Vector3f RimCentorPosition(RIM rim);
    void PointContact(RIM rim, float alpha=0);
    void PointVelocity(Eigen::Vector3f v, Eigen::Vector3f w, RIM rim, float alpha=0, bool inbody_coord=true);
    float RimRoll(RIM rim);
    Eigen::Vector3f RollVelocity(Eigen::Vector3f w, RIM rim, float alpha=0);
    float beta = 0;
    float beta_d = 0;
    float beta_dd = 0;
    Eigen::Vector3f offset;
    Eigen::Vector3f contact_point;
    Eigen::Vector3f contact_velocity;
    float Radius() {return this->R;}
    float radius() {return this->r;}
    std::pair<float, float> Inverse(Eigen::Vector3f p, RIM rim);
    float link_w = 0;
    float link_w_d = 0;
    std::complex<float> rim_p;

};
#endif