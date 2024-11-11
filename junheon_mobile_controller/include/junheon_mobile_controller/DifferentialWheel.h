#ifndef DIFFERENTIAL_WHEEL_H
#define DIFFERENTIAL_WHEEL_H

#include <vector>
#include <string>
#include <Eigen/Dense>

namespace JunheonMobileController
{
    class DifferentialWheel {
    public:
        DifferentialWheel(std::vector<std::string> wheel_names, double base_width, double wheel_radius, double lin_vel_limit, double ang_vel_limit, double lin_acc_limit, double ang_acc_limit, double control_freq);
        
        Eigen::VectorXd IK(const Eigen::Vector3d& desired_base_velocity);
        Eigen::Vector3d FK(const Eigen::VectorXd& desired_wheel_vel);
        Eigen::VectorXd VelocityCommand(const Eigen::Vector3d& desired_base_vel, const Eigen::Vector3d& current_base_vel);

    private:
        std::vector<std::string> wheel_names_;
        double base_width_;
        double wheel_radius_;
        double lin_vel_limit_;
        double ang_vel_limit_;
        double lin_acc_limit_;
        double ang_acc_limit_;
        double hz_;
    };
}

#endif // DIFFERENTIAL_WHEEL_H