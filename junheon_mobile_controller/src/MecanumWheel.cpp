#include "junheon_mobile_controller/MecanumWheel.h"
#include <cassert>
#include <cmath>

namespace JunheonMobileController
{
    MecanumWheel::MecanumWheel(std::vector<std::string> wheel_names, double base_width, double base_length, double wheel_radius, double lin_vel_limit, double ang_vel_limit, double lin_acc_limit, double ang_acc_limit, double control_freq)
        : wheel_names_(wheel_names), 
          base_width_(base_width), 
          base_length_(base_length), 
          wheel_radius_(wheel_radius), 
          lin_vel_limit_(lin_vel_limit), 
          ang_vel_limit_(ang_vel_limit),
          lin_acc_limit_(lin_acc_limit), 
          ang_acc_limit_(ang_acc_limit), 
          hz_(control_freq) {}

    Eigen::VectorXd MecanumWheel::IK(const Eigen::Vector3d& desired_base_velocity) 
    {
        double lxly = base_length_ / 2 + base_width_ / 2;

        Eigen::MatrixXd transformation_matrix(4, 3);
        transformation_matrix << 1, -1, -lxly,
                                 1,  1,  lxly,
                                 1,  1, -lxly,
                                 1, -1,  lxly;
        transformation_matrix /= wheel_radius_;

        Eigen::VectorXd desired_wheel_vel = transformation_matrix * desired_base_velocity;
        Eigen::VectorXd desired_wheel_vel_align = Eigen::VectorXd::Zero(wheel_names_.size());

        for (size_t i = 0; i < wheel_names_.size(); ++i) 
        {
            if (wheel_names_[i].find("front") != std::string::npos) 
            {
                if (wheel_names_[i].find("left") != std::string::npos) 
                {
                    desired_wheel_vel_align[i] = desired_wheel_vel[0];
                } 
                else 
                {
                    desired_wheel_vel_align[i] = desired_wheel_vel[1];
                }
            } 
            else 
            {
                if (wheel_names_[i].find("left") != std::string::npos) 
                {
                    desired_wheel_vel_align[i] = desired_wheel_vel[2];
                } 
                else 
                {
                    desired_wheel_vel_align[i] = desired_wheel_vel[3];
                }
            }
        }
        return desired_wheel_vel_align;
    }

    Eigen::Vector3d MecanumWheel::FK(const Eigen::VectorXd& desired_wheel_vel) 
    {
        assert(desired_wheel_vel.size() == wheel_names_.size());
        
        Eigen::VectorXd desired_wheel_vel_dealign(4);
        for (size_t i = 0; i < wheel_names_.size(); ++i) 
        {
            if (wheel_names_[i].find("front") != std::string::npos) 
            {
                if (wheel_names_[i].find("left") != std::string::npos) 
                {
                    desired_wheel_vel_dealign[0] = desired_wheel_vel[i];
                }
                else 
                {
                    desired_wheel_vel_dealign[1] = desired_wheel_vel[i];
                }
            } 
            else 
            {
                if (wheel_names_[i].find("left") != std::string::npos) 
                {
                    desired_wheel_vel_dealign[2] = desired_wheel_vel[i];
                } 
                else 
                {
                    desired_wheel_vel_dealign[3] = desired_wheel_vel[i];
                }
            }
        }

        double lxly = base_length_ / 2 + base_width_ / 2;
        Eigen::MatrixXd transformation_matrix(3, 4);
        transformation_matrix << 0.25,        0.25,         0.25,        0.25,
                                -0.25,        0.25,         0.25,       -0.25,
                                -0.25 / lxly, 0.25 / lxly, -0.25 / lxly, 0.25 / lxly;
        transformation_matrix *= wheel_radius_;

        Eigen::Vector3d desired_base_vel = transformation_matrix * desired_wheel_vel_dealign;
        return desired_base_vel;
    }

    Eigen::VectorXd MecanumWheel::VelocityCommand(const Eigen::Vector3d& desired_base_vel, const Eigen::Vector3d& current_base_vel) 
    {
        Eigen::Vector3d desired_base_acc = (desired_base_vel - current_base_vel) * hz_;

        if (desired_base_acc.head<2>().norm() > lin_acc_limit_) 
        {
            desired_base_acc.head<2>() = lin_acc_limit_ * desired_base_acc.head<2>().normalized();
        }
        if (std::abs(desired_base_acc[2]) > ang_acc_limit_) 
        {
            desired_base_acc[2] = std::copysign(ang_acc_limit_, desired_base_acc[2]);
        }
        Eigen::Vector3d new_base_vel = current_base_vel + desired_base_acc / hz_;

        if (new_base_vel.head<2>().norm() > lin_vel_limit_)
        {
            new_base_vel.head<2>() = lin_vel_limit_ * new_base_vel.head<2>().normalized();
        }
        if (std::abs(new_base_vel[2]) > ang_vel_limit_) 
        {
            new_base_vel[2] = std::copysign(ang_vel_limit_, new_base_vel[2]);
        }

        return IK(new_base_vel);
    }
}
