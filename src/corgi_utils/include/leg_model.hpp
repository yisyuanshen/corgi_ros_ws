#ifndef LEGMODEL_HPP
#define LEGMODEL_HPP

#include <array>
#include <complex>

class LegModel {
    public:
        // Constructor
        LegModel(bool sim = true);

        // Forward kinematics
        void forward(double theta, double beta, bool vector = true);

        // Inverse kinematics
        std::array<double, 2> inverse(const std::array<double, 2> &pos, const std::string &joint = "G");
        std::array<double, 2> inverse(const double pos[2], const std::string &joint = "G") {
            return inverse(std::array<double, 2>{pos[0], pos[1]}, joint);
        }
        
        // Contact map
        void contact_map(double theta_in, double beta_in, double slope=0.0, bool contact_upper=true, bool contact_lower=true);

        // Move
        std::array<double, 2> move(double theta_in, double beta_in, std::array<double, 2> move_vec, double slope=0.0, bool contact_upper=true, bool contact_lower=true, double tol = 1e-14, size_t max_iter = 100);

        // Joint Positions
        std::array<double, 2> A_l, A_r, B_l, B_r, C_l, C_r, D_l, D_r, E, F_l, F_r, G, H_l, H_r, U_l, U_r, L_l, L_r;
        
        // Joint Positions in complex
        std::complex<double> A_l_c, A_r_c, B_l_c, B_r_c, C_l_c, C_r_c, D_l_c, D_r_c, E_c, F_l_c, F_r_c, G_c, H_l_c, H_r_c, U_l_c, U_r_c, L_l_c, L_r_c;

        // Constants
        const double R; // Wheel radius
        const double r; // Tire radius
        const double radius;  // Wheel radius + Tire radius

        // Current theta and beta
        double theta;
        double beta;

        // Contact map variable
        int rim = 3;    // 1 -> 2 -> 3 -> 4 -> 5 -> 0: 
                        // U_l -> L_l -> G -> L_r -> U_r -> None
        double alpha;
        std::array<double, 2> contact_p;
    private:

        // Constants
        const double max_theta;
        const double min_theta;
        const double theta0;
        const double beta0;
        const double arc_HF;
        const double arc_BC;
        const double l1, l2, l3, l4, l5, l6, l7, l8;
        const double l_AE, l_BF, l_BH, ang_UBC, ang_LFG, ang_BCF;

        // Intermediate variables
        double l_BD;
        double ang_OEA;
        double ang_DBC;
        double ang_OGF;

        // Helper functions
        void calculate();
        void rotate();
        void symmetry();
        void to_vector();
        std::array<double, 3> arc_min(const std::complex<double>& p1, const std::complex<double>& p2, const std::complex<double>& O, const std::string& rim);
        std::array<double, 2> objective(const std::array<double, 2>& d_q, const std::array<double, 2>& current_q, const std::array<double, 2>& move_vec, int contact_rim);

};//end class LegModel

#endif // LEGMODEL_HPP
