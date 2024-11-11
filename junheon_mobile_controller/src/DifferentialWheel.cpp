#include "junheon_mobile_controller/DifferentialWheel.h"
#include <cassert>
#include <cmath>

namespace JunheonMobileController
{
    DifferentialWheel::DifferentialWheel(std::vector<std::string> wheel_names, double base_width, double wheel_radius, double lin_vel_limit, double ang_vel_limit, double lin_acc_limit, double ang_acc_limit, double control_freq)
        : wheel_names_(wheel_names), 
          base_width_(base_width), 
          wheel_radius_(wheel_radius), 
          lin_vel_limit_(lin_vel_limit), 
          ang_vel_limit_(ang_vel_limit),
          lin_acc_limit_(lin_acc_limit), 
          ang_acc_limit_(ang_acc_limit), 
          hz_(control_freq) 
    {

    }

    Eigen::VectorXd DifferentialWheel::IK(const Eigen::Vector3d& desired_base_velocity) 
    {
        assert(std::abs(desired_base_velocity[1]) < 1e-3 && "Desired velocity along y-axis must be zero!");

        Eigen::Vector2d base_velocity(desired_base_velocity[0], desired_base_velocity[2]);
        Eigen::Matrix2d transformation_matrix;
        transformation_matrix << 1,  base_width_ / 2,
                                 1, -base_width_ / 2;
        transformation_matrix /= wheel_radius_;

        Eigen::Vector2d desired_wheel_vel = transformation_matrix * base_velocity;
        Eigen::VectorXd desired_wheel_vel_align(wheel_names_.size());

        for (size_t i = 0; i < wheel_names_.size(); ++i) 
        {
            if (wheel_names_[i].find("right") != std::string::npos) 
            {
                desired_wheel_vel_align[i] = desired_wheel_vel[0];
            } 
            else 
            {
                desired_wheel_vel_align[i] = desired_wheel_vel[1];
            }
        }
        return desired_wheel_vel_align;
    }

    Eigen::Vector3d DifferentialWheel::FK(const Eigen::VectorXd& desired_wheel_vel)
    {
        assert(desired_wheel_vel.size() == wheel_names_.size());

        Eigen::Vector2d desired_wheel_vel_dealign;
        for (size_t i = 0; i < wheel_names_.size(); ++i) 
        {
            if (wheel_names_[i].find("right") != std::string::npos) 
            {
                desired_wheel_vel_dealign[0] = desired_wheel_vel[i];
            } 
            else
            {
                desired_wheel_vel_dealign[1] = desired_wheel_vel[i];
            }
        }

        Eigen::Matrix2d transformation_matrix;
        transformation_matrix << 0.5,              0.5,
                                 1 / base_width_, -1 / base_width_;
        transformation_matrix *= wheel_radius_;

        Eigen::Vector2d base_velocity = transformation_matrix * desired_wheel_vel_dealign;
        return Eigen::Vector3d(base_velocity[0], 0, base_velocity[1]);
    }

    Eigen::VectorXd DifferentialWheel::VelocityCommand(const Eigen::Vector3d& desired_base_vel, const Eigen::Vector3d& current_base_vel) 
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

