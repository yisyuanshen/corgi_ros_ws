/**
 * @file IF_main.cpp
 * 
 * @author peichunhuang
 */
#include "KLD_estimation/InformationFilter.hpp"
#include "KLD_estimation/csv_reader.hpp"
#include <random>
using namespace estimation_model;

template<size_t n>
Eigen::Vector<float, n> random_vector() {
    std::random_device rd;
    std::mt19937 gen(rd());  //here you could also set a seed
    std::uniform_real_distribution<float> dis(-1, 1);
    Eigen::Vector<float, n> V = Eigen::Vector<float, n>().NullaryExpr([&](){return dis(gen);});
    return V;
}

float random_number() {
    std::random_device rd;
    std::mt19937 gen(rd());  //here you could also set a seed
    std::uniform_real_distribution<float> dis(-1, 1);
    return dis(gen);
}


int main (int argc, char* argv[]) {

    std::string filename = std::string(argv[1]);
    std::string filepath = std::getenv("HOME");
    filepath += "/corgi_ws/corgi_ros_ws/src/corgi_odometry/data/";
    DataProcessor::DataFrame df =  DataProcessor::read_csv(filepath+filename+".csv");
    int n = df.row;
    Eigen::initParallel();
    // float thres = 1e-10;
    float thres = 0.08;
    if (argc == 3) {
        thres = std::atof(argv[2]);
    }
    std::cout << thres << "\n";
    int start_index = 0;
    while (1) {
        start_index ++;
        if (df.iloc("trigger", start_index)) 
            break;
    }
    Eigen::MatrixXf estimate_state = Eigen::MatrixXf::Zero(n - start_index, 46);
    Eigen::Vector3f a(df.iloc("a.x", start_index), df.iloc("a.y", start_index), df.iloc("a.z", start_index));
    Eigen::Quaternionf q(df.iloc("q.w", start_index), df.iloc("q.x", start_index), df.iloc("q.y", start_index), df.iloc("q.z", start_index));
    Eigen::Vector<float, 5> encoder_lf(df.iloc("lf.theta", start_index), df.iloc("lf.beta", start_index), df.iloc("lf.beta_d", start_index), df.iloc("w.y", start_index), df.iloc("lf.theta_d", start_index));
    Eigen::Vector<float, 5> encoder_rf(df.iloc("rf.theta", start_index), df.iloc("rf.beta", start_index), df.iloc("rf.beta_d", start_index), df.iloc("w.y", start_index), df.iloc("rf.theta_d", start_index));
    Eigen::Vector<float, 5> encoder_rh(df.iloc("rh.theta", start_index), df.iloc("rh.beta", start_index), df.iloc("rh.beta_d", start_index), df.iloc("w.y", start_index), df.iloc("rh.theta_d", start_index));
    Eigen::Vector<float, 5> encoder_lh(df.iloc("lh.theta", start_index), df.iloc("lh.beta", start_index), df.iloc("lh.beta_d", start_index), df.iloc("w.y", start_index), df.iloc("lh.theta_d", start_index));

    Eigen::Vector3f v_init(0, 0, 0) ;
    const int j = 10; // sample time (matrix size)
    const int SAMPLE_RATE = 200; // 200 500 1000 400 Hz 
    float dt = 1 / (float)SAMPLE_RATE;
    U u(j, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0), dt) ;
    Leg lf_leg(Eigen::Vector3f(0.222, 0.193, 0), 0.1, 0.012);
    Leg rf_leg(Eigen::Vector3f(0.222, -0.193, 0), 0.1, 0.012);
    Leg rh_leg(Eigen::Vector3f(-0.222, -0.193, 0), 0.1, 0.012);
    Leg lh_leg(Eigen::Vector3f(-0.222, 0.193, 0), 0.1, 0.012);
    DP lf(j + 1, lf_leg, encoder_lf, 0, &u);
    DP rf(j + 1, rf_leg, encoder_rf, 0, &u);
    DP rh(j + 1, rh_leg, encoder_rh, 0, &u);
    DP lh(j + 1, lh_leg, encoder_lh, 0, &u);
    Eigen::Quaternionf qt265(df.iloc("t265.qw", start_index), df.iloc("t265.qx", start_index), df.iloc("t265.qy", start_index), df.iloc("t265.qz", start_index));
    Eigen::Matrix3f t265_to_imu;
    t265_to_imu << 0, 0, -1, 1, 0, 0, 0, -1, 0;
    T265 t265(&lf, t265_to_imu, Eigen::Vector3f(0, 0, 0.285), j + 1, 
    Eigen::Vector3f(df.iloc("t265.x", 0), df.iloc("t265.y", 0), df.iloc("t265.z", 0)), 
    qt265.toRotationMatrix());
    Eigen::VectorXf x = Eigen::VectorXf::Zero(6 * j);
    for (int i = 0; i < j; i++) {
        x(i * 3) = v_init(0);
        x(i * 3 + 1) = v_init(1);
        x(i * 3 + 2) = v_init(2);
        x((i + j) * 3) = 0;
        x((i + j) * 3 + 1) = 0;
        x((i + j) * 3 + 2) = 0;
    }

    PKLD lf_pkld(dt, &lf);
    PKLD rf_pkld(dt, &rf);
    PKLD rh_pkld(dt, &rh);
    PKLD lh_pkld(dt, &lh);
    PKLD t265_pkld(dt, &t265);

    GKLD filter(j, dt, &u);
    filter.threshold = thres;
    filter.init(x) ;
    filter.push_pkld(&lf_pkld) ;
    filter.push_pkld(&rf_pkld) ;
    filter.push_pkld(&rh_pkld) ;
    filter.push_pkld(&lh_pkld) ;
    int counter = 0;
    Eigen::Vector3f p(0, 0, 0);

    Eigen::Quaternionf q_init(df.iloc("q.w", start_index), df.iloc("q.x", start_index), df.iloc("q.y", start_index), df.iloc("q.z", start_index));
    Eigen::Matrix3f R_init = q_init.toRotationMatrix();
    Eigen::Matrix3f rot;
    rot << 1, 0, 0, 0, -1, 0, 0, 0, -1;
    int sec = 0, usec = 0;
    R_init = rot * R_init;
    Eigen::Vector3f t265_pose = Eigen::Vector3f(df.iloc("t265.x", start_index), df.iloc("t265.y", start_index), df.iloc("t265.z", start_index));
    for (int i = start_index+1; i < n; i++) {
        std::cout << i << "\n";
        Eigen::Quaternionf qt265(df.iloc("t265.qw", i-1), df.iloc("t265.qx", i-1), df.iloc("t265.qy", i-1), df.iloc("t265.qz", i-1));
        Eigen::Vector3f a(df.iloc("a.x", i), df.iloc("a.y", i), df.iloc("a.z", i));
        Eigen::Quaternionf q(df.iloc("q.w", i-1), df.iloc("q.x", i-1), df.iloc("q.y", i-1), df.iloc("q.z", i-1));
        Eigen::Vector3f w(df.iloc("w.x", i), df.iloc("w.y", i), df.iloc("w.z", i));
        Eigen::Vector<float, 5> encoder_lf(df.iloc("lf.theta", i), df.iloc("lf.beta", i), df.iloc("lf.beta_d", i), df.iloc("w.y", i), df.iloc("lf.theta_d", i));
        Eigen::Vector<float, 5> encoder_rf(df.iloc("rf.theta", i), df.iloc("rf.beta", i), df.iloc("rf.beta_d", i), df.iloc("w.y", i), df.iloc("rf.theta_d", i));
        Eigen::Vector<float, 5> encoder_rh(df.iloc("rh.theta", i), df.iloc("rh.beta", i), df.iloc("rh.beta_d", i), df.iloc("w.y", i), df.iloc("rh.theta_d", i));
        Eigen::Vector<float, 5> encoder_lh(df.iloc("lh.theta", i), df.iloc("lh.beta", i), df.iloc("lh.beta_d", i), df.iloc("w.y", i), df.iloc("lh.theta_d", i));
        Eigen::Matrix3f R = q.toRotationMatrix();
        u.push_data(a, w, dt);

        float alpha_lf,  alpha_rf, alpha_rh, alpha_lh;
        if (counter % 50) {
            alpha_lf = -100;
            alpha_rf = -100;
            alpha_rh = -100;
            alpha_lh = -100;
        }
        else {
            alpha_lf = - atan2(df.iloc("lf.dist", i) - df.iloc("lh.dist", i) , 0.357);
            alpha_rf = - atan2(df.iloc("rf.dist", i) - df.iloc("rh.dist", i) , 0.357);
            alpha_rh = - atan2(df.iloc("rf.dist", i) - df.iloc("rh.dist", i) , 0.357);
            alpha_lh = - atan2(df.iloc("lf.dist", i) - df.iloc("lh.dist", i) , 0.357);
        }
        lf.push_data(encoder_lf, w, dt, alpha_lf);
        rf.push_data(encoder_rf, w, dt, alpha_rf);
        rh.push_data(encoder_rh, w, dt, alpha_rh);
        lh.push_data(encoder_lh, w, dt, alpha_lh);
        if ((t265_pose - Eigen::Vector3f(df.iloc("t265.x", i), df.iloc("t265.y", i), df.iloc("t265.z", i))).norm() > 0)
            usec += 1;
        t265_pose = Eigen::Vector3f(df.iloc("t265.x", i), df.iloc("t265.y", i), df.iloc("t265.z", i));
        t265.push_data(t265_pose,
        qt265.toRotationMatrix(), Eigen::Matrix3f::Identity() * 9e-6, sec, usec);

        filter.predict();
        if (t265.is_update()) {
            filter.certain_valid(&t265_pkld);
        }
        filter.valid();
        x = filter.state();
        Eigen::Matrix3f P_cov = filter.Y_inv.block<3, 3>(3*j-3, 3*j-3);
        p += rot * R * R_init.transpose() * x.segment(3 * j - 3, 3) * dt;
        estimate_state.row(i-start_index-1).segment(0, 3) = x.segment(3 * j - 3, 3);
        estimate_state.row(i-start_index-1).segment(3, 3) = p;
        estimate_state.row(i-start_index-1).segment(6, 3) = 1. / dt / (float) j * lf.z(dt);
        estimate_state.row(i-start_index-1).segment(9, 3) = 1. / dt / (float) j * rf.z(dt);
        estimate_state.row(i-start_index-1).segment(12, 3) = 1. / dt / (float) j * rh.z(dt);
        estimate_state.row(i-start_index-1).segment(15, 3) = 1. / dt / (float) j * lh.z(dt);
        estimate_state.row(i-start_index-1).segment(18, 3) = 1. / dt / (float) j * t265.z(dt);
        estimate_state.row(i-start_index-1).segment(21, 3) = x.segment(6 * j - 3, 3);
        estimate_state.row(i-start_index-1).segment(24, 4) = Eigen::Vector4f(filter.exclude[0], filter.exclude[1], filter.exclude[2], filter.exclude[3]);
        estimate_state.row(i-start_index-1).segment(28, 4) = Eigen::Vector<float, 4>(filter.scores[0], filter.scores[1], filter.scores[2], filter.scores[3]);
        estimate_state.row(i-start_index-1).segment(32, 4) = Eigen::Vector4f(df.iloc("lf.contact", i), df.iloc("rf.contact", i), df.iloc("rh.contact", i), df.iloc("lh.contact", i));
        estimate_state.row(i-start_index-1)(36) = filter.threshold;
        estimate_state.row(i-start_index-1).segment(37, 9) = Eigen::Map<const Eigen::VectorXf>(P_cov.data(), P_cov.size());
        counter ++;
    }
    
    std::vector<std::string> cols = {
        "v_.x", "v_.y", "v_.z", 
        "p.x", "p.y", "p.z", 
        "zLF.x", "zLF.y", "zLF.z", 
        "zRF.x", "zRF.y", "zRF.z", 
        "zRH.x", "zRH.y", "zRH.z", 
        "zLH.x", "zLH.y", "zLH.z", 
        "zP.x", "zP.y", "zP.z", 
        "ba.x", "ba.y", "ba.z", 
        "lf.contact","rf.contact","rh.contact","lh.contact",
        "lf.cscore","rf.cscore","rh.cscore","lh.cscore",
        "lf.c","rf.c","rh.c","lh.c",
        "threshold",
        "cov.xx", "cov.xy", "cov.xz", "cov.yx", "cov.yy", "cov.yz", "cov.zx", "cov.zy", "cov.zz"
    };
    DataProcessor::write_csv(estimate_state, filepath+"out_"+filename+"_kld"+".csv", cols);
    return 0;
}