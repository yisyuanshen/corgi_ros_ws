#ifndef CLEG_HPP
#define CLEG_HPP
#include <cmath>
#include <complex>
#include <iostream>
#include <mutex>
#include <Eigen/Dense>
#include <deque>
#include "nlopt.hpp"
#define M_PI_2_F 1.57079632679489661923132169163975144f

#define M_PI_F 3.14159265358979323846 
enum RIM
{
    CONTACT = 1,
    NO_CONTACT = 0
};

inline float to_degrees(float radians) {
    return radians * (180.0 / M_PI_F);
}
inline float to_radians(float degrees) {
    return degrees * M_PI_F / 180.0;
}

class CLeg {
    public:
    CLeg(float l_, float r_, Eigen::Vector3f offset_) : l(l_), r(r_) {
        offset = offset_;
        theta = 0;
        phi = 0;
        theta_d = 0;
        phi_d = 0;
        theta_dd = 0;
        phi_dd = 0;
        theta0 = M_PI_2_F * 3.f;
        phi0 = 0.9948;
    }
    void calculate(float theta, float phi, float theta_d, float phi_d, float tm) {
        this->theta = theta;
        this->phi = phi;
        this->theta_d = theta_d;
        this->phi_d = phi_d;
    }
    Eigen::Vector3f PointContact(RIM rim, float alpha=0) {
        if (rim) {
            std::complex<float> contact = std::polar<float>(l, theta0 + theta) + std::polar<float>(r, theta0 + theta - M_PI_F + phi0 - phi)
            + std::polar<float>(r, alpha+M_PI_F);
            return Eigen::Vector3f(contact.imag(), 0, contact.real()) + offset;
        }
        else return Eigen::Vector3f(0, 0, 0);
    }
    Eigen::Vector3f roll(RIM rim, Eigen::Vector3f w, float alpha=0, float dt=0.001) {
        if (rim) {
            float roll_distance = r * (theta_d - phi_d + w(1)) * dt;
            return Eigen::Vector3f(roll_distance * cos(alpha), 0, roll_distance * sin(alpha));
        }
        else return Eigen::Vector3f(0, 0, 0);
    }
    private:
    float l;
    float r;
    float phi;
    float theta;
    float phi0;
    float theta0;
    float phi_d;
    float theta_d;
    float phi_dd;
    float theta_dd;
    Eigen::Vector3f offset;
};

#endif