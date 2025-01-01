#ifndef CONTACT_MAP_HPP
#define CONTACT_MAP_HPP

#include "kinematic/Leg.hpp"
#include <vector>
#include <deque>
#include <tuple>

using trajectory = std::tuple<float, float, float, Eigen::Matrix3f>; // theta, beta, contact_exp, R
namespace estimation_model {
extern Eigen::Vector3f da;
extern Eigen::Vector3f dw;
extern Eigen::Vector3f dba;
extern Eigen::Vector3f dR;
extern Eigen::Vector4f dM; // theta, beta, theta_d, beta_d
extern float dL; // lidar
class ContactMap {
    public:
        ContactMap() {}
        RIM lookup(float theta, float beta) { // theta [17  160]; beta [0  360)
            RIM r = NO_CONTACT;
            rad_mod2(beta);
            theta = theta * 180.0 / M_PI;
            beta = beta * 180.0 / M_PI;
            if (theta > 108.3) {
                if (b1(theta) > beta) r = G_POINT;
                else if (b2(theta) >= beta) r = LOWER_RIM_R;
                else if (b3(theta) > beta) r = UPPER_RIM_R;
                else if ((360 - b3(theta)) > beta) r = NO_CONTACT;
                else if ((360 - b2(theta)) > beta) r = UPPER_RIM_L;
                else if ((360 - b1(theta)) > beta) r = LOWER_RIM_L;
                else r = G_POINT;
            }
            else {
                if (b1(theta) > beta) r = G_POINT;
                else if (b2(theta) >= beta) r = LOWER_RIM_R;
                else if (180.0 > beta) r = UPPER_RIM_R;
                else if ((360 - b2(theta)) > beta) r = UPPER_RIM_L;
                else if ((360 - b1(theta)) > beta) r = LOWER_RIM_L;
                else r = G_POINT;
            }
            return r;
        }
        std::pair<float, float> Boundary(float theta, RIM r) {
            switch(r) {
                case G_POINT:
                    return std::pair<float, float>(-b1(theta), b1(theta));
                break;
                case LOWER_RIM_R:
                    return std::pair<float, float>(b1(theta), b2(theta));
                break;
                case UPPER_RIM_R:
                    return theta > 108.3? std::pair<float, float>(b2(theta), b3(theta)) : std::pair<float, float>(b2(theta), 180.0);
                break;
                case NO_CONTACT:
                    return theta > 108.3? std::pair<float, float>(b3(theta), 360 - b3(theta)) : std::pair<float, float>(180.0, 180.0);
                break;
                case UPPER_RIM_L:
                    return theta > 108.3? std::pair<float, float>(360 - b3(theta), 360 - b2(theta)) : std::pair<float, float>(180.0, 360 - b2(theta));
                break;
                case LOWER_RIM_L:
                    return std::pair<float, float>(360 - b2(theta), 360 - b1(theta));
                break;
            }
        }

        float boudary_beta(float theta, float beta, float theta2, float beta2) {
            RIM rim = this->lookup(theta, beta);
            RIM rim2 = this->lookup(theta2, beta2);
            if (rim == rim2) return 0;
            std::pair<float, float> boundary = this->Boundary(theta * 180. / M_PI, rim);
            float bl = boundary.first * M_PI / 180.0; rad_mod2(bl);
            float bu = boundary.second * M_PI / 180.0; rad_mod2(bu);
            float diff1 = (bl - beta); rad_mod(diff1);
            float diff2 = (bu - beta); rad_mod(diff2);
            float diff3 = (bl - beta2); rad_mod(diff3);
            float diff4 = (bu - beta2); rad_mod(diff4);
            diff1 = abs(diff1) > abs(diff3)? diff3: diff1;
            diff2 = abs(diff2) > abs(diff4)? diff4: diff2;
            float bound_beta;
            if (abs(diff1) > abs(diff2)) bound_beta = bu;
            else bound_beta = bl;
            return bound_beta;
        }
        void rad_mod(float &rad) {
            if (rad > M_PI) {
                rad -= 2*M_PI;
                rad_mod(rad);
            }
            else if (rad <= -M_PI) {
                rad += 2*M_PI;
                rad_mod(rad);
            }
        }

        void rad_mod2(float &rad) {
            if (rad > 2. * M_PI) {
                rad -= 2*M_PI;
                rad_mod2(rad);
            }
            else if (rad <= 0) {
                rad += 2*M_PI;
                rad_mod2(rad);
            }
        }

        Eigen::Vector3f travel(std::deque<trajectory> path, Leg &leg) {
            int size = path.size() ;
            if (size == 1) return Eigen::Vector3f(0, 0, 0);
            RIM last_rim = this->lookup(std::get<0>(path[0]), std::get<2>(path[0]));
            Eigen::Vector3f distance(0, 0, 0);
            Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
            for (int i = 0; i < size - 1; i ++) {
                RIM current_rim = this->lookup(std::get<0>(path[i+1]), std::get<2>(path[i+1]));
                if (current_rim != last_rim) {
                    if (current_rim == G_POINT || last_rim == G_POINT) {
                        float bound_beta = this->boudary_beta(std::get<0>(path[i]), std::get<2>(path[i]), std::get<0>(path[i+1]), std::get<2>(path[i+1]));
                        Eigen::Vector3f travel_current = travel_contineous(last_rim, leg, std::get<2>(path[i]), bound_beta, std::get<1>(path[i]), std::get<3>(path[i]), std::get<3>(path[i+1]), cov);
                        distance += travel_current;
                        travel_current = travel_contineous(current_rim, leg, bound_beta, std::get<2>(path[i+1]), std::get<1>(path[i]), std::get<3>(path[i]), std::get<3>(path[i+1]), cov);
                        distance += travel_current;
                    }
                    else if (current_rim == NO_CONTACT || last_rim == NO_CONTACT) {
                        return Eigen::Vector3f(0, 0, 0);
                    }
                    else {
                        Eigen::Vector3f travel_current = travel_contineous(last_rim, leg, std::get<2>(path[i]), std::get<2>(path[i+1]), std::get<1>(path[i]), std::get<3>(path[i]), std::get<3>(path[i+1]), cov);
                        distance += travel_current;
                        leg.Calculate(std::get<0>(path[i]), 0, 0, std::get<1>(path[i]), 0, 0);
                        float angle_between_body_frame = std::get<2>(path[i]) - std::get<1>(path[i]);
                        if ( angle_between_body_frame >= M_PI)  angle_between_body_frame -= M_PI * 2;
                        if ( angle_between_body_frame <= -M_PI)  angle_between_body_frame += M_PI * 2;
                        leg.PointContact(last_rim, angle_between_body_frame);
                        Eigen::Vector3f point_from = leg.contact_point;
                        leg.PointContact(current_rim, angle_between_body_frame);
                        Eigen::Vector3f point_to = leg.contact_point;
                        distance += std::get<3>(path[i]) * (point_to - point_from);
                    }
                }
                else {
                    Eigen::Vector3f travel_current = travel_contineous(last_rim, leg, std::get<2>(path[i]), std::get<2>(path[i+1]), std::get<1>(path[i]), std::get<3>(path[i]), std::get<3>(path[i+1]), cov);
                    distance += travel_current;
                }
                last_rim = current_rim;
            }
            return distance;
        }

        Eigen::Vector3f travel2(std::deque<trajectory> path, Leg &leg) { // R : dR
            int size = path.size() ;
            if (size == 1) return Eigen::Vector3f(0, 0, 0);
            RIM last_rim = this->lookup(std::get<0>(path[0]), std::get<2>(path[0]));
            Eigen::Vector3f distance(0, 0, 0);
            Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
            Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
            for (int i = 0; i < size - 1; i ++) {
                Eigen::Matrix3f rot_last = rot;
                rot = rot * (std::get<3>(path[i])).transpose();
                RIM current_rim = this->lookup(std::get<0>(path[i+1]), std::get<2>(path[i+1]));
                if (current_rim != last_rim) {
                    if (current_rim == G_POINT || last_rim == G_POINT) {
                        float bound_beta = this->boudary_beta(std::get<0>(path[i]), std::get<2>(path[i]), std::get<0>(path[i+1]), std::get<2>(path[i+1]));
                        Eigen::Vector3f travel_current = travel_contineous(last_rim, leg, std::get<2>(path[i]), bound_beta, std::get<1>(path[i]), rot_last, rot, cov);
                        distance += travel_current;
                        travel_current = travel_contineous(current_rim, leg, bound_beta, std::get<2>(path[i+1]), std::get<1>(path[i]), rot_last, rot, cov);
                        distance += travel_current;
                    }
                    else if (current_rim == NO_CONTACT || last_rim == NO_CONTACT) {
                        return Eigen::Vector3f(0, 0, 0);
                    }
                    else {
                        Eigen::Vector3f travel_current = travel_contineous(last_rim, leg, std::get<2>(path[i]), std::get<2>(path[i+1]), std::get<1>(path[i]), rot_last, rot, cov);
                        distance += travel_current;
                        leg.Calculate(std::get<0>(path[i]), 0, 0, std::get<1>(path[i]), 0, 0);
                        float angle_between_body_frame = std::get<2>(path[i]) - std::get<1>(path[i]);
                        if ( angle_between_body_frame >= M_PI)  angle_between_body_frame -= M_PI * 2;
                        if ( angle_between_body_frame <= -M_PI)  angle_between_body_frame += M_PI * 2;
                        leg.PointContact(last_rim, angle_between_body_frame);
                        Eigen::Vector3f point_from = leg.contact_point;
                        leg.PointContact(current_rim, angle_between_body_frame);
                        Eigen::Vector3f point_to = leg.contact_point;
                        distance += rot_last * (point_to - point_from);
                    }
                }
                else {
                    Eigen::Vector3f travel_current = travel_contineous(last_rim, leg, std::get<2>(path[i]), std::get<2>(path[i+1]), std::get<1>(path[i]), rot_last, rot, cov);
                    distance += travel_current;
                }
                last_rim = current_rim;
            }
            return distance;
        }

        Eigen::Vector3f compensate(std::deque<trajectory> path, Leg &leg, std::deque<float> theta_d, float dt) {
            int size = path.size() ;
            if (size == 1) return Eigen::Vector3f(0, 0, 0);
            RIM last_rim = this->lookup(std::get<0>(path[0]), std::get<2>(path[0]));
            Eigen::Vector3f distance(0, 0, 0);
            for (int i = 0; i < size - 1; i ++) {
                RIM current_rim = this->lookup(std::get<0>(path[i+1]), std::get<2>(path[i+1]));
                if (current_rim == NO_CONTACT || last_rim == NO_CONTACT) {return Eigen::Vector3f(0, 0, 0);}
                else {
                    leg.Calculate(std::get<0>(path[i]), theta_d[i], 0, 0, 0, 0);
                    float omega_last = leg.RimRoll(last_rim);
                    float omega_current = leg.RimRoll(current_rim);
                    float radius_current = current_rim == G_POINT? leg.radius(): leg.Radius() + leg.radius();
                    float radius_last = last_rim == G_POINT? leg.radius(): leg.Radius() + leg.radius();
                    float rolling_distance = (radius_last * omega_last + radius_current * omega_current) * dt * 0.5;
                    float angle_between_body_frame = std::get<2>(path[i]) - std::get<1>(path[i]);
                    if ( angle_between_body_frame >= M_PI)  angle_between_body_frame -= M_PI * 2;
                    if ( angle_between_body_frame <= -M_PI)  angle_between_body_frame += M_PI * 2;
                    Eigen::Vector3f body_frame_travel(cos(angle_between_body_frame) * rolling_distance, 0, sin(-angle_between_body_frame) * rolling_distance);
                    distance += std::get<3>(path[i]) * body_frame_travel;
                }
                last_rim = current_rim;
            }
            return distance;
        }

        Eigen::Vector3f compensate2(std::deque<trajectory> path, Leg &leg, std::deque<float> theta_d, float dt) {
            int size = path.size() ;
            if (size == 1) return Eigen::Vector3f(0, 0, 0);
            RIM last_rim = this->lookup(std::get<0>(path[0]), std::get<2>(path[0]));
            Eigen::Vector3f distance(0, 0, 0);
            Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
            for (int i = 0; i < size - 1; i ++) {
                Eigen::Matrix3f rot_last = rot;
                rot = rot * (std::get<3>(path[i])).transpose();
                RIM current_rim = this->lookup(std::get<0>(path[i+1]), std::get<2>(path[i+1]));
                if (current_rim == NO_CONTACT || last_rim == NO_CONTACT) {return Eigen::Vector3f(0, 0, 0);}
                else {
                    leg.Calculate(std::get<0>(path[i]), theta_d[i], 0, 0, 0, 0);
                    float omega_last = leg.RimRoll(last_rim);
                    float omega_current = leg.RimRoll(current_rim);
                    float radius_current = current_rim == G_POINT? leg.radius(): leg.Radius() + leg.radius();
                    float radius_last = last_rim == G_POINT? leg.radius(): leg.Radius() + leg.radius();
                    float rolling_distance = (radius_last * omega_last + radius_current * omega_current) * dt * 0.5;
                    float angle_between_body_frame = std::get<2>(path[i]) - std::get<1>(path[i]);
                    if ( angle_between_body_frame >= M_PI)  angle_between_body_frame -= M_PI * 2;
                    if ( angle_between_body_frame <= -M_PI)  angle_between_body_frame += M_PI * 2;
                    Eigen::Vector3f body_frame_travel(cos(angle_between_body_frame) * rolling_distance, 0, sin(-angle_between_body_frame) * rolling_distance);
                    distance += rot_last * body_frame_travel;
                    float radius = current_rim == G_POINT? leg.radius(): current_rim == NO_CONTACT? 0: leg.radius() + leg.Radius();
                }
                last_rim = current_rim;
            }
            return distance;
        }
        Eigen::Matrix3f travel_covariance;
    private:
        inline float b1(float theta) {return -2.61019580e-09 * pow(theta, 5) + 1.24181267e-06 * pow(theta, 4) 
        - 2.24183011e-04 * pow(theta, 3) + 1.78431692e-02 * theta * theta - 1.33151836e-01 * theta - 1.78362899e+00 ;}
        inline float b2(float theta) {return -1.22581785e-09 * pow(theta, 5) + 5.02932993e-07 * pow(theta, 4) 
        -7.37114643e-05 * pow(theta, 3) + 6.47617996e-03 * theta * theta -3.31750539e-01 * theta + 5.40846840e+01 ;}
        inline float b3(float theta) {return -4.87190741e-07 * pow(theta, 5) + 3.21347467e-04 * pow(theta, 4) 
        -8.40604260e-02 * pow(theta, 3) + 1.09041600e+01 * theta * theta -7.02946587e+02 * theta + 1.82438639e+04 ;}
        Eigen::Vector3f travel_contineous(RIM current_rim, Leg &leg, float contact_beta_from, float contact_beta_to, float beta_from, Eigen::Matrix3f rot_from, Eigen::Matrix3f rot_to, Eigen::Matrix3f &cov) {
            float radius = current_rim == G_POINT? leg.radius(): current_rim == NO_CONTACT? 0: leg.radius() + leg.Radius();
            float dbeta = contact_beta_to - contact_beta_from;
            if (dbeta >= M_PI) dbeta -= M_PI * 2;
            if (dbeta <= -M_PI) dbeta += M_PI * 2;
            float angle_between_body_frame = contact_beta_from - beta_from;
            if ( angle_between_body_frame >= M_PI)  angle_between_body_frame -= M_PI * 2;
            if ( angle_between_body_frame <= -M_PI)  angle_between_body_frame += M_PI * 2;
            float d = radius * dbeta;
            Eigen::Vector3f body_frame_travel(cos(angle_between_body_frame) * d, 0, sin(-angle_between_body_frame) * d) ;
            return rot_from * body_frame_travel;
        }
};
}

#endif