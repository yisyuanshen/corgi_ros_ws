/**
 * @file InformationFilter.cpp
 * 
 * @author peichunhuang
 */
#include "KLD_estimation/InformationFilter.hpp"

namespace estimation_model {
    Eigen::Vector3f da(0.008, 0.008, 0.008);
    Eigen::Vector3f dw(0.01, 0.01, 0.01);
    Eigen::Vector3f dba(3.924e-4, 3.924e-4, 3.924e-4);
    Eigen::Vector3f dR(0.01, 0.01, 0.01);
    Eigen::Vector4f dM(0.01, 0.01, 0.3, 0.3); // theta, beta, theta_d, beta_d
    float dL = 0.01; // lidar
    Eigen::Matrix3f CovarianceFromRotation(Eigen::Matrix3f R, Eigen::Vector3f v) {
        v = R * v;
        Eigen::Matrix3f C;
        float d = dR.norm();
        float r11 = v(2) * v(2) * dR(1) * dR(1) + v(1) * v(1) * dR(2) * dR(2);
        float r12 = -v(1) * v(0) * dR(2) * dR(2);
        float r13 = -v(2) * v(0) * dR(1) * dR(1);
        float r22 = v(0) * v(0) * dR(2) * dR(2) + v(2) * v(2) * dR(0) * dR(0);
        float r23 = -v(2) * v(1) * dR(0) * dR(0);
        float r33 = v(1) * v(1) * dR(0) * dR(0) + v(0) * v(0) * dR(1) * dR(1);
        C << r11, r12, r13,
        r12, r22, r23,
        r13, r23, r33;
        C *= d * d;
        return C;
    }
    Eigen::Matrix3f AngleAxisToRotation(Eigen::Vector3f dw_) {
        double dtheta = dw_.norm();
        Eigen::Vector3f n_dw = dw_ / dtheta;
        Eigen::Matrix3f skew_dw; skew_dw << 0, -dw_(2), dw_(1), dw_(2), 0, -dw_(0), -dw_(1), dw_(0), 0;
        return Eigen::MatrixXf::Identity(3, 3) + sin(dtheta) * skew_dw + (1 - cos(dtheta)) * (dw_ * dw_.transpose() - Eigen::MatrixXf::Identity(3, 3));
    }
    U::U(int size, Eigen::Vector3f a_init, Eigen::Vector3f w_init, float dt) : n(size), low_pass_ba((2.f * M_PI * dt * 5.f) / (2.f * M_PI * dt * 5. + 1.f)), low_pass_v((2.f * M_PI * dt * 500.) / (2.f * M_PI * dt * 500. + 1.f)) {
        for (int i = 0; i < size + 2; i ++) {
            rot_d.push_back(AngleAxisToRotation(w_init * dt)) ;
            accel.push_back(a_init) ;
        }
        A_ = Eigen::MatrixXf::Zero(6 * n, 6 * n);
        for (int i = 0; i < n - 1; i ++) {
            A_.block<3, 3>(i * 3, i * 3 + 3) = Eigen::Matrix3f::Identity();
        }
        A_.block<3, 3>(n * 3 - 3, n * 3 - 3) = Eigen::Matrix3f::Identity();
        A_.block<3, 3>(n * 3 - 3, n * 6 - 3) = dt * Eigen::Matrix3f::Identity();
        for (int i = n; i < 2 * n - 1; i ++) {
            A_.block<3, 3>(i * 3, i * 3) = (1.-low_pass_ba) * Eigen::Matrix3f::Identity();
            A_.block<3, 3>(i * 3, i * 3 + 3) = low_pass_ba * Eigen::Matrix3f::Identity();
        }
        A_.block<3, 3>(n * 6 - 3, n * 6 - 6) = (1.-low_pass_ba) * Eigen::Matrix3f::Identity();
        A_.block<3, 3>(n * 6 - 3, n * 6 - 3) = low_pass_ba * Eigen::Matrix3f::Identity();
    }

    Eigen::VectorXf U::dz(float dt) {
        Eigen::Vector3f sum = Eigen::Vector3f(0, 0, 0);
        Eigen::Matrix3f r = Eigen::Matrix3f::Identity();
        for (int i = 0; i < n; i ++) {
            Eigen::Vector3f a = 0.5 * dt * dt * r * accel[i];
            r *= rot_d[i];
            sum += a;
        }
        return sum;
    }

    Eigen::MatrixXf U::R(float dt, Eigen::Vector3f v, Eigen::Vector3f ba) {
        Eigen::MatrixXf R = 4e-10 * Eigen::MatrixXf::Identity(n * 6, n * 6);
        R.block<3, 3>(n * 3 - 3, n * 3 - 3) = CovarianceFromRotation(rot_d[n - 1], v + dt * accel[n-1] - dt * ba) + dt * dt * rot_d[n - 1] * da.asDiagonal() * rot_d[n - 1].transpose();
        // R.block<3, 3>(n * 6 - 3, n * 6 - 3) = 2.5e-5 * Eigen::Matrix3f::Identity();
        R.block<3, 3>(n * 6 - 3, n * 6 - 3) = dba.asDiagonal();
        return R;
    }
    Eigen::MatrixXf U::A(float dt) {
        A_.block<3, 3>(n * 3 - 3, n * 3 - 3) = rot_d[n - 1];
        A_.block<3, 3>(n * 3 - 3, n * 6 - 3) = -dt * rot_d[n - 1];
        return A_;
    }
    Eigen::VectorXf U::u(float dt) {
        Eigen::VectorXf u = Eigen::VectorXf::Zero(n * 6);
        Eigen::Vector3f a =  dt * rot_d[n-1] * accel[n-1];
        u.segment((n-1) * 3, 3) = a;
        return u;
    }
    void U::push_data(Eigen::Vector3f a, Eigen::Vector3f w, float dt) {
        rot_d.push_back(AngleAxisToRotation(w * dt)) ;
        accel.push_back(a) ;
        rot_d.pop_front() ;
        accel.pop_front() ;
    }

    DP::DP(int size, Leg &leg_, Eigen::Vector<float, 5> encoder_init, float alpha_init, U *input) : n(size), leg(leg_), u(input) { // n = j + 1
        for (int i = 0; i < size; i ++) {
            trajectories.push_back(trajectory{encoder_init(0), encoder_init(1), encoder_init(1) + alpha_init, Eigen::Matrix3f::Identity()});
            theta_d.push_back(encoder_init(4));
        }
    }

    DP::DP(int size, Leg &leg_,U *input) : n(size), leg(leg_), u(input){}

    void DP::init(Eigen::Vector<float, 5> encoder_init, float alpha_init){
        for (int i = 0; i < n; i ++) {
            trajectories.push_back(trajectory{encoder_init(0), encoder_init(1), encoder_init(1) + alpha_init, Eigen::Matrix3f::Identity()});
            theta_d.push_back(encoder_init(4));
        }
    }

    void DP::push_data(Eigen::Vector<float, 5> encoders, Eigen::Vector3f wk, float dt, float alpha) {
        trajectory last = trajectories.back();
        float contact_beta = (encoders(2) + encoders(3)) * dt + std::get<2>(last);
        if (alpha != -100) contact_beta = encoders(1) + alpha;
        trajectories.push_back(trajectory{encoders(0), encoders(1), contact_beta, AngleAxisToRotation(wk * dt)});
        trajectories.pop_front();
        theta_d.push_back(encoders(4));
        theta_d.pop_front();
    } // encoders: theta, beta, beta_d, omega, theta_d

    Eigen::MatrixXf DP::Q(float dt) {
        return Eigen::Matrix3f::Identity() * 2.5e-7;
    }
    Eigen::VectorXf DP::z(float dt) {
        Eigen::Vector3f t = cm.travel2(trajectories, leg);
        Eigen::Vector3f c = cm.compensate2(trajectories, leg, theta_d, dt);
        trajectory last = trajectories.back();
        trajectory first = trajectories.front();
        RIM last_contact_rim = cm.lookup(std::get<0>(last), std::get<2>(last));
        RIM first_contact_rim = cm.lookup(std::get<0>(first), std::get<2>(first));
        leg.Calculate(std::get<0>(last), 0, 0, std::get<1>(last), 0, 0);
        leg.PointContact(last_contact_rim, std::get<2>(last) - std::get<1>(last));
        Eigen::Matrix3f last_rot = Eigen::Matrix3f::Identity();
        for (int i = 0; i < n - 1; i ++) {
            last_rot = last_rot * (std::get<3>(trajectories[i])).transpose();
        }
        Eigen::Vector3f last_point = last_rot * leg.contact_point;


        leg.Calculate(std::get<0>(last) + dM(0), 0, 0, std::get<1>(last) + dM(1), 0, 0);
        leg.PointContact(last_contact_rim, std::get<2>(last) - std::get<1>(last));
        Eigen::Vector3f last_point1 = last_rot * leg.contact_point;
        leg.Calculate(std::get<0>(last) - dM(0), 0, 0, std::get<1>(last) - dM(1), 0, 0);
        leg.PointContact(last_contact_rim, std::get<2>(last) - std::get<1>(last));
        Eigen::Vector3f last_point2 = last_rot * leg.contact_point;

        leg.Calculate(std::get<0>(first), 0, 0, std::get<1>(first), 0, 0);
        leg.PointContact(first_contact_rim, std::get<2>(first) - std::get<1>(first));
        Eigen::Vector3f first_point = leg.contact_point;
        Eigen::Vector3f d = t + first_point - last_point + c - u->dz(dt);
        return d;
    }
    
    Eigen::MatrixXf DP::C(float dt) {
        static Eigen::MatrixXf C_ = Eigen::MatrixXf::Constant(3, (n - 1) * 6, 0);
        Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
        for (int i = 0; i < (n - 1); i++) {
            C_.block<3, 3>(0, 3 * i) = dt * rot ;
            C_.block<3, 3>(0, 3 * (i + n - 1)) = - dt * rot * dt * 0.5 ;
            rot = rot * (std::get<3>(trajectories[i])).transpose();
        }
        return C_;
    }

    T265::T265(Z* observed, Eigen::Matrix3f transform, Eigen::Vector3f offset, int j, Eigen::Vector3f p, Eigen::Matrix3f R) : z_(observed), offset_(offset), transform_(transform), n(j) {
        for (int i = 0; i < j; i ++) {
            position.push_back(p);
            rot.push_back(R);
        }
        Q_ = Eigen::Matrix3f::Identity() * 1e-8;
        update = false;
        // transform : from t265 to imu base frame
    }
    void T265::push_data(Eigen::Vector3f p, Eigen::Matrix3f R, Eigen::Matrix3f cov, uint32_t sec, uint32_t usec) {
        position.push_back(p);
        rot.push_back(R);
        position.pop_front();
        rot.pop_front();
        Q_ = cov;
        if (sec > last_sec || usec > last_usec) update = true;
        else update = false;
        last_sec = sec;
        last_usec = usec;
    }
    Eigen::MatrixXf T265::Q(float dt) {
        return Q_;
    }
    Eigen::VectorXf T265::z(float dt) {
        return transform_ * rot.front().transpose() * (position.back() - rot.back() * offset_ - position.front() + rot.front() * offset_);
    }
    Eigen::MatrixXf T265::C(float dt) {
        return z_->C(dt);
    }
    bool T265::is_update() {
        return update;
    }

    PKLD::PKLD(float t, Z* observed) : dt(t), z(observed){
    }
    void PKLD::valid(Eigen::VectorXf &dy, Eigen::MatrixXf &dY) {
        Eigen::VectorXf observed = z->z(dt);
        Q = z->Q(dt);
        Eigen::MatrixXf Qinv = Q.inverse();
        Eigen::MatrixXf C = z->C(dt);
        Eigen::MatrixXf Ct = C.transpose();
        dy = Ct * Qinv * observed;
        dY = Ct * Qinv * C;
    }
    GKLD::GKLD(int j, float t, U* input) : dt(t), n(j), u(input) {
        A = Eigen::MatrixXf::Identity(6 * j, 6 * j);
        for (int i = j; i < 2 * j; i ++) {
            A.block<3, 3>((i - j) * 3, i * 3) = -dt * Eigen::Matrix3f::Identity();
        }
        Y = Eigen::MatrixXf::Zero(6 * j, 6 * j);
        for (int i = 0; i < 6 * j; i++) {
            if (i < 3 * j) Y(i, i) = 1e8;
            else Y(i, i) = 1e8;
        }
        R.resize(6 * j, 6 * j) ;
        x.resize(6 * j) ;
        y = Eigen::VectorXf::Constant(6 * j, 1e-10);
    }
    void GKLD::init(Eigen::VectorXf x_init) {
        x = x_init;
    }
    void GKLD::push_pkld(PKLD* p) {
        information_pools.push_back(p);
        exclude.push_back(false);
        pool_size += 1;
        scores.push_back(0);
        threads.push_back(std::thread());
    }
    void GKLD::predict() {
        A = u->A(dt);
        x = A * x + u->u(dt); // x predict
        R = u->R(dt, x.segment(3 * n - 3, 3), x.segment(6 * n - 3, 3));
        Y = (A * Y.inverse() * A.transpose() + R).inverse(); // Y predict
        Y_inv = Y.inverse();
        y = Y * x; // y predict
        for (int i = 0; i < pool_size; i++) 
            exclude[i] = false;
    }
    void GKLD::score(Eigen::MatrixXf Yj, Eigen::VectorXf yj, int i) {
        Eigen::VectorXf xj = Yj.inverse() * yj;
        if (i == -1) g_score = 0.5 * (Yj * Y_inv).trace() + 0.5 * log(Y.norm() / Yj.norm()) - 0.5 * (float)(6 * n) + 0.5 * (xj - x).transpose() * Yj * (xj - x);
        else scores[i] = 0.5 * (Yj * Y_inv).trace() + 0.5 * log(Y.norm() / Yj.norm()) - 0.5 * (float)(6 * n) + 0.5 * (xj - x).transpose() * Yj * (xj - x);
    }
    void GKLD::certain_valid(PKLD* p) {
        Eigen::VectorXf y__;
        Eigen::MatrixXf Y__;
        p->valid(y__, Y__);
        Y += Y__;
        y += y__;
    }
    void GKLD::valid() {
        static float alpha = (2.f * M_PI * dt * 1.f) / (2.f * M_PI * dt * 1.f + 1.f);
        y_set.resize(pool_size);
        Y_set.resize(pool_size);
        for (int i = 0; i < pool_size; i++) {
            Eigen::VectorXf y__;
            Eigen::MatrixXf Y__;
            information_pools[i]->valid(y__, Y__);
            y_set[i] = y__;
            Y_set[i] = Y__;
            exclude[i] = false;
            scores[i] = 0;
        }
        Eigen::VectorXf sum_y = y;
        Eigen::MatrixXf sum_Y = Y;
        threshold = threshold > 0.05? threshold: 0.05;
        for (int i = 0; i < pool_size; i++) {
            sum_y += y_set[i];
            sum_Y += Y_set[i];
        }
        score(sum_Y, sum_y, -1);
        if (g_score < (threshold * 1.02) ) {
            Y = sum_Y;
            y = sum_y;
            x = Y.inverse() * y;
            threshold *= 1 - alpha;
            threshold += alpha * (g_score);
            return;
        }
        else {
            threshold *= 1 - alpha;
            std::vector<Eigen::VectorXf> yj_set;
            std::vector<Eigen::MatrixXf> Yj_set;
            for (int i = 0; i < pool_size; i++) { 
                yj_set.push_back(y+y_set[i]);
                Yj_set.push_back(Y+Y_set[i]);
            }
            for (int i = 0; i < pool_size; i++) {
                threads[i] = std::thread(&GKLD::score, this, std::ref(Yj_set[i]), std::ref(yj_set[i]), i);
            }
            double min = 1e300;
            int min_index = 0;
            int exclude_num = 0;
            for (int i = 0; i < pool_size; i++) {
                if (threads[i].joinable())
                    threads[i].join();
                if (min > scores[i]) {
                    min = scores[i];
                    min_index =i;
                }
                if (scores[i] > (threshold * 1.02)) {
                    exclude[i] = true;
                    exclude_num += 1;
                }
                else {
                    Y += Y_set[i];
                    y += y_set[i];
                }
            }
            threshold += alpha * (scores[min_index]);
            if (exclude_num == pool_size) {
                Y += Y_set[min_index];
                y += y_set[min_index];
                exclude[min_index] = false;
            }
            x = Y.inverse() * y;
            return;
        }
    }

    void GKLD::valid(bool exclude_[4]) {
        y_set.resize(pool_size);
        Y_set.resize(pool_size);
        Eigen::VectorXf sum_y = y;
        Eigen::MatrixXf sum_Y = Y;
        for (int i = 0; i < pool_size; i++) {
            exclude[i] = exclude_[i];
            if(!exclude[i]) {
                Eigen::VectorXf y__;
                Eigen::MatrixXf Y__;
                information_pools[i]->valid(y__, Y__);
                y_set[i] = y__;
                Y_set[i] = Y__;
                sum_y += y__;
                sum_Y += Y__;
            }
        }
        Y = sum_Y;
        y = sum_y;
        x = Y.inverse() * y;
        return;
    }
    
    void GKLD::iterative_valid() {
    }
}
