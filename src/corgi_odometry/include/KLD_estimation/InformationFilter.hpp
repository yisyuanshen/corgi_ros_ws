#ifndef IF_HPP
#define IF_HPP
#include <Eigen/Dense>
#include "ContactMap.hpp"
#include <thread>
namespace estimation_model {

    Eigen::Matrix3f AngleAxisToRotation(Eigen::Vector3f dw) ;

    class U {
        public:
            U(int size, Eigen::Vector3f a_init, Eigen::Vector3f w_init, float dt) ;
            Eigen::MatrixXf R(float dt, Eigen::Vector3f v, Eigen::Vector3f ba) ;
            Eigen::MatrixXf A(float dt) ;
            Eigen::VectorXf dz(float dt);
            Eigen::VectorXf u(float dt) ;
            void push_data(Eigen::Vector3f a, Eigen::Vector3f w, float dt) ;
        private:
            std::deque<Eigen::Vector3f> accel;
            std::deque<Eigen::Matrix3f> rot_d;
            const int n;
            Eigen::MatrixXf A_;
            const float low_pass_ba;
            const float low_pass_v;
    };

    class Z {
        public:
        virtual Eigen::VectorXf z(float dt) {return Eigen::VectorXf();}
        virtual Eigen::MatrixXf Q(float dt) {return Eigen::MatrixXf();}
        virtual Eigen::MatrixXf C(float dt) {return Eigen::MatrixXf();}
    };

    class DP: public Z {
        public:
            DP(int size, Leg &leg_, Eigen::Vector<float, 5> encoder_init, float alpha_init, U *input) ;
            DP(int size, Leg &leg_, U *input) ;
            void init(Eigen::Vector<float, 5> encoder_init, float alpha_init) ;
            Eigen::MatrixXf Q(float dt);
            Eigen::VectorXf z(float dt);
            Eigen::MatrixXf C(float dt);
            void push_data(Eigen::Vector<float, 5> encoders, Eigen::Vector3f wk, float dt, float alpha = -100) ; // encoders: theta, beta, beta_d, omega
        private:
            std::deque<trajectory> trajectories;
            std::deque<float> theta_d;
            Leg &leg;
            const int n;
            ContactMap cm;
            U *u;
    };

    class T265: public Z {
        public:
            T265(Z* z, Eigen::Matrix3f transform, Eigen::Vector3f offset, int j, Eigen::Vector3f p, Eigen::Matrix3f R) ;
            void push_data(Eigen::Vector3f p, Eigen::Matrix3f R, Eigen::Matrix3f cov, uint32_t sec, uint32_t usec);
            Eigen::MatrixXf Q(float dt);
            Eigen::VectorXf z(float dt);
            Eigen::MatrixXf C(float dt);
            bool is_update();
        private:
            bool update;
            std::deque<Eigen::Vector3f> position;
            std::deque<Eigen::Matrix3f> rot;
            Eigen::MatrixXf Q_;
            Z *z_;
            Eigen::Vector3f offset_;
            Eigen::Matrix3f transform_;
            const int n;
            uint32_t last_sec;
            uint32_t last_usec;
    };
    
    class PKLD {
        public:
        PKLD(float t, Z* observed) ;
        void valid(Eigen::VectorXf &dy, Eigen::MatrixXf &dY) ;
        private:
        Eigen::MatrixXf Q;
        float dt;
        Z* z;
    };
    class GKLD {
        public:
        GKLD(int j, float t, U* input) ;
        void init(Eigen::VectorXf x_init) ;
        void push_pkld(PKLD* p) ;
        void predict() ;
        void valid();
        void valid(bool exclude_[4]);
        void certain_valid(PKLD* p);
        void score(Eigen::MatrixXf Yj, Eigen::VectorXf yj, int i);
        void iterative_valid() ;
        float threshold = 0;
        Eigen::VectorXf state() {return x;}
        Eigen::MatrixXf A;
        Eigen::VectorXf x;
        Eigen::VectorXf y;
        Eigen::MatrixXf Y;
        Eigen::MatrixXf Y_inv;
        Eigen::MatrixXf R;
        std::vector<bool> exclude;
        std::vector<float> scores;
        private:
        std::vector<std::thread> threads;
        std::vector<PKLD*> information_pools;
        double g_score;
        float dt;
        U* u;
        const int n;
        int pool_size = 0;
        std::vector<Eigen::VectorXf> y_set;
        std::vector<Eigen::MatrixXf> Y_set;
    };
}

#endif