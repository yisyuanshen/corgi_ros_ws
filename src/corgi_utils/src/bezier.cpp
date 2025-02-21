#include <vector>
#include <array>
#include <cmath>

#include "bezier.hpp"

/* class Bezier */
Bezier::Bezier(const std::vector<std::array<double, 2>>& control_pts) : 
    /* Initializer List */
    control_pts(control_pts) 
{
    bz_cff = bz_coeff(control_pts);
}//end Bezier

std::array<double, 2> Bezier::getBzPoint(double t, double offset_x, double offset_y) {
    auto bzt_cff = bzt_coeff(control_pts, t);
    double x = 0;
    double y = 0;
    for (size_t i=0; i<control_pts.size(); i++) {
        x += bzt_cff[i] * bz_cff[i] * control_pts[i][0];
        y += bzt_cff[i] * bz_cff[i] * control_pts[i][1];
    }//end for
    x += offset_x;
    y += offset_y;
    return {x, y};
}//end getBzPoint

int Bezier::fact(int n) {
    return (n == 0 || n == 1)? 1 : n * fact(n - 1);
}//end fact

int Bezier::comb(int n, int k) {
    return fact(n) / (fact(k) * fact(n - k));
}//end comb

std::vector<int> Bezier::bz_coeff(const std::vector<std::array<double, 2>>& cp_list) {
    int sz = cp_list.size();
    std::vector<int> bzc(sz);
    for (int i=0; i<sz; i++) {
        bzc[i] = comb(sz - 1, i);
    }//end for
    return bzc;
}//end bz_coeff

std::vector<double> Bezier::bzt_coeff(const std::vector<std::array<double, 2>>& cp_list, double t) {
    int sz = cp_list.size();
    std::vector<double> bzc(sz);
    for (int i=0; i<sz; i++) {
        double ord_t_1 = static_cast<double>((sz - 1) - i);
        double ord_t = static_cast<double>(i);
        bzc[i] = std::pow((1 - t), ord_t_1) * std::pow(t, ord_t);
    }//end for
    return bzc;
}//end bzt_coeff


/* class SwingProfile */
// p_l: position of G when lift leg
// p_t: position of G when touch ground
SwingProfile::SwingProfile(std::array<double, 2> p_l, std::array<double, 2> p_t, double step_height, int direction) :
    /* Initializer List */
    L(p_t[0] - p_l[0]), 
    h(step_height), 
    offset_x(p_l[0]), 
    offset_y(p_l[1]), 
    diff_h(p_t[1] - p_l[1]),
    dh(0), dL1(0), dL2(0), dL3(0), dL4(0),
    direction(direction)
{
    getControlPoints();
    bezier = Bezier(control_points);
}//end LegModel

std::array<double, 2> SwingProfile::getFootendPoint(double t_duty) {
    return bezier.getBzPoint(t_duty, offset_x, offset_y);
}//end getFootendPoint

int SwingProfile::getDirection() {
    return direction;
}//end getDirection

void SwingProfile::getControlPoints() {
    std::array<double, 2> c0 = {0, 0};
    std::array<double, 2> c1 = {c0[0] - dL1, c0[1]};
    std::array<double, 2> c2 = {c1[0] - dL2, c1[1] + h};
    std::array<double, 2> c3 = c2;
    std::array<double, 2> c4 = c2;
    std::array<double, 2> c5 = {c4[0] + 0.5 * L + dL1 + dL2, c4[1]};
    std::array<double, 2> c6 = c5;
    // std::array<double, 2> c7 = {c5[0], c5[1] + dh};
    // std::array<double, 2> c8 = {c7[0] + 0.5 * L + dL3 + dL4, c7[1]};
    std::array<double, 2> c7 = {c5[0] + 0.5 * L + dL3 + dL4, c5[1] + dh};
    std::array<double, 2> c8 = c7;
    std::array<double, 2> c9 = c8;
    std::array<double, 2> c10 = {c8[0] - dL4, c8[1] - h - dh + diff_h};
    std::array<double, 2> c11 = {c10[0] - dL3, c10[1]};

    control_points = {c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11};
}//end getControlPoint