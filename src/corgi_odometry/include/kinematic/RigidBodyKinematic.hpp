#ifndef RIGID_BODY_KINEMATIC_HPP
#define RIGID_BODY_KINEMATIC_HPP
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <map>
#include <queue>
namespace Eigen
{
    Matrix3d skew3(Eigen::Vector3d vec)
    {
        Eigen::Matrix3d mo; 
        mo << 0.0, -vec(2), vec(1),
             vec(2), 0.0, -vec(0), 
             -vec(1), vec(0), 0.0;
        return mo;
    }
}
namespace RigidBodyDynamic {
    Eigen::Matrix3d AngleAxisToRotation(Eigen::Vector3d dw) {
        double dtheta = dw.norm();
        Eigen::Vector3d n_dw = dw / dtheta;
        Eigen::Matrix3d skew_dw; skew_dw << 0, -dw(2), dw(1), dw(2), 0, -dw(0), -dw(1), dw(0), 0;
        return Eigen::MatrixXd::Identity(3, 3) + sin(dtheta) * skew_dw + (1 - cos(dtheta)) * (dw * dw.transpose() - Eigen::MatrixXd::Identity(3, 3));
    }

    struct Joint {
        public:
            Joint() {}
            Joint (double x_, double y_, double z_) {position = {x_, y_, z_};}
            Joint (Eigen::Vector3d p) {position = p;}
            Eigen::Vector3d position = {0, 0, 0};
            double x() {return position(0);}
            double y() {return position(1);}
            double z() {return position(2);}

    };

    struct State {
        public:
            Eigen::Vector3d position = {0, 0, 0};
            Eigen::Vector3d velocity = {0, 0, 0};
            Eigen::Vector3d acceleration = {0, 0, 0};
            Eigen::Vector3d angular_velocity = {0, 0, 0};
            Eigen::Vector4d attitude = {0, 0, 0, 1}; // x y z w
            Eigen::Vector3d angular_alpha = {0, 0, 0};
    };

    class RigidBody {
        public:
            Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();
            double mass = 1.0;
            State state;
            std::map<std::string, Joint> joints;
            void apply_force(std::string joint, Eigen::Vector3d force);
            void apply_torque(Eigen::Vector3d torque);
            void update(double duration);
        private:
            std::queue< std::pair<std::string, Eigen::Vector3d> > tmp_forces;
            std::queue< Eigen::Vector3d> tmp_torques;
    };

    void RigidBody::apply_force(std::string joint, Eigen::Vector3d force) {
        tmp_forces.push(std::pair<std::string, Eigen::Vector3d>(joint, force) );
    }

    void RigidBody::apply_torque(Eigen::Vector3d torque) {
        tmp_torques.push(torque);
    }

    void RigidBody::update(double duration) {
        
        Eigen::Vector3d total_torque = {0, 0, 0};
        Eigen::Vector3d total_force = {0, 0, 0};
        Eigen::Matrix3d C = Eigen::Quaterniond(state.attitude).toRotationMatrix();
        while (! tmp_forces.empty()) {
            std::pair<std::string, Eigen::Vector3d> tmp_force = tmp_forces.front();
            total_torque += joints[tmp_force.first].position.cross(tmp_force.second);
            total_force += C * tmp_force.second;
            tmp_forces.pop();
        }
        while (! tmp_torques.empty()) {
            total_torque += tmp_torques.front();
            tmp_torques.pop();
        }
        state.acceleration = total_force / mass;
        state.position += state.velocity * duration + 0.5 * duration * state.acceleration * duration;
        state.velocity += state.acceleration * duration;
        state.angular_alpha = inertia.inverse() * total_torque - inertia.inverse() * Eigen::skew3(state.angular_velocity) * inertia * state.angular_velocity;
        state.attitude = Eigen::Quaterniond(AngleAxisToRotation(state.angular_velocity * duration + 0.5 * state.angular_alpha * duration * duration) * C).coeffs();
        state.angular_velocity += state.angular_alpha * duration;
    }
}

#endif