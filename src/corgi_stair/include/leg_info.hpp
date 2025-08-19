#ifndef LEGINFO_HPP
#define LEGINFO_HPP

#include <array>

class LegInfo {
    public:
        LegInfo(int leg_ID, double BL=0.444);
        std::array<double, 2> get_hip_position(std::array<double, 2> CoM, double pitch);

        int ID;
        double BL;
        std::array<double, 2> foothold;
        std::array<double, 2> next_foothold;
        bool touch;
        bool next_up;
        bool one_step;
        bool contact_edge;
        double contact_alpha;
        int stair_count;    // the leg is stepping on this stair

};//end class LegInfo

#endif // LEGINFO_HPP