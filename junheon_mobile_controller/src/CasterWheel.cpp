#include "junheon_mobile_controller/CasterWheel.h"
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

namespace JunheonMobileController
{
    CasterWheel::CasterWheel(std::vector<std::string> wheel_names, double base_width, double base_length,
                            double wheel_radius, double wheel_offset, double lin_vel_limit, double ang_vel_limit,
                            double lin_acc_limit, double ang_acc_limit, double control_freq)
        : wheel_names_(wheel_names), 
          base_width_(base_width), 
          base_length_(base_length), 
          wheel_radius_(wheel_radius), 
          wheel_offset_(wheel_offset), 
          lin_vel_limit_(lin_vel_limit), 
          ang_vel_limit_(ang_vel_limit), 
          lin_acc_limit_(lin_acc_limit),
          ang_acc_limit_(ang_acc_limit), 
          hz_(control_freq) 
    {
        
    }

    Eigen::VectorXd CasterWheel::IK(const Eigen::Vector3d& desired_base_velocity, const Eigen::VectorXd& current_wheel_angle) 
    {
        try
        {
            Eigen::VectorXd current_steer_angle_dealign(wheel_names_.size() / 2); // [front_left, front_right, rear_left, rear_right]
            for (size_t i = 0; i < wheel_names_.size(); ++i) 
            {
                if (wheel_names_[i].find("steer") != std::string::npos) 
                {
                    if (wheel_names_[i].find("front") != std::string::npos)
                    {
                        if (wheel_names_[i].find("left") != std::string::npos)      current_steer_angle_dealign[0] = current_wheel_angle[i];
                        else if(wheel_names_[i].find("right") != std::string::npos) current_steer_angle_dealign[1] = current_wheel_angle[i];
                    }
                    else if (wheel_names_[i].find("rear") != std::string::npos)
                    {
                        if (wheel_names_[i].find("left") != std::string::npos)      current_steer_angle_dealign[2] = current_wheel_angle[i];
                        else if(wheel_names_[i].find("right") != std::string::npos) current_steer_angle_dealign[3] = current_wheel_angle[i];
                    }
                }
            }
            
            double b = wheel_offset_;
            double r = wheel_radius_;
            Eigen::MatrixXd C(wheel_names_.size(), 3);
            for (size_t i = 0; i < current_steer_angle_dealign.size(); ++i) 
            {
                double K_xi = (i < 2) ? base_length_ / 2 : -base_length_ / 2;    // front is +1, rear is -1
                double K_yi = (i % 2 == 0) ? base_width_ / 2 : -base_width_ / 2; // left is +1, right is -1
                double phi = current_steer_angle_dealign[i];
                C.block<2, 3>(2 * i, 0) << -sin(phi) / b, cos(phi) / b, (cos(phi) * K_xi + sin(phi) * K_yi) / b - 1,
                                            cos(phi) / r, sin(phi) / r, (sin(phi) * K_xi - cos(phi) * K_yi) / r;
            }

            Eigen::VectorXd desired_wheel_vel = C * desired_base_velocity; // [front_left_steer, front_left_rotate, front_right_steer, front_right_rotate,
                                                                           //  rear_left_steer,  rear_left_rotate,  rear_right_steer,  rear_right_rotate]
            Eigen::VectorXd desired_wheel_vel_align(wheel_names_.size());
            for (size_t i = 0; i < wheel_names_.size(); ++i) 
            {
                if (wheel_names_[i].find("front") != std::string::npos)
                {
                    if (wheel_names_[i].find("left") != std::string::npos)
                    {
                        if (wheel_names_[i].find("steer") != std::string::npos)       desired_wheel_vel_align[i] = desired_wheel_vel[0];
                        else if (wheel_names_[i].find("rotate") != std::string::npos) desired_wheel_vel_align[i] = desired_wheel_vel[1];
                    }
                    else if (wheel_names_[i].find("right") != std::string::npos)
                    {
                        if (wheel_names_[i].find("steer") != std::string::npos)       desired_wheel_vel_align[i] = desired_wheel_vel[2];
                        else if (wheel_names_[i].find("rotate") != std::string::npos) desired_wheel_vel_align[i] = desired_wheel_vel[3];
                    }
                }
                else if (wheel_names_[i].find("rear") != std::string::npos)
                {
                    if (wheel_names_[i].find("left") != std::string::npos)
                    {
                        if (wheel_names_[i].find("steer") != std::string::npos)       desired_wheel_vel_align[i] = desired_wheel_vel[4];
                        else if (wheel_names_[i].find("rotate") != std::string::npos) desired_wheel_vel_align[i] = desired_wheel_vel[5];
                    }
                    else if (wheel_names_[i].find("right") != std::string::npos)
                    {
                        if (wheel_names_[i].find("steer") != std::string::npos)       desired_wheel_vel_align[i] = desired_wheel_vel[6];
                        else if (wheel_names_[i].find("rotate") != std::string::npos) desired_wheel_vel_align[i] = desired_wheel_vel[7];
                    }
                }
            }

            return desired_wheel_vel_align;
        } 
        catch (const std::exception& e) 
        {
            std::cerr << "Error in IK calculation: " << e.what() << std::endl;
            return Eigen::VectorXd();
        }
    }

    Eigen::Vector3d CasterWheel::FK(const Eigen::VectorXd& desired_wheel_vel, const Eigen::VectorXd& current_wheel_angle) 
    {
        try 
        {
            Eigen::VectorXd current_steer_angle_dealign(wheel_names_.size() / 2); // [front_left, front_right, rear_left, rear_right]
            Eigen::VectorXd desired_wheel_vel_dealign(wheel_names_.size()); // [front_left_steer, front_left_rotate, front_right_steer, front_right_rotate,
                                                                            //  rear_left_steer,  rear_left_rotate,  rear_right_steer,  rear_right_rotate]
            for (size_t i = 0; i < wheel_names_.size(); ++i) 
            {
                if (wheel_names_[i].find("front") != std::string::npos)
                {
                    if (wheel_names_[i].find("left") != std::string::npos)
                    {
                        if (wheel_names_[i].find("steer") != std::string::npos){      desired_wheel_vel_dealign[0] = desired_wheel_vel[i];
                                                                                      current_steer_angle_dealign[0] = current_wheel_angle[i];}
                        else if (wheel_names_[i].find("rotate") != std::string::npos) desired_wheel_vel_dealign[1] = desired_wheel_vel[i];
                    }
                    else if (wheel_names_[i].find("right") != std::string::npos)
                    {
                        if (wheel_names_[i].find("steer") != std::string::npos){      desired_wheel_vel_dealign[2] = desired_wheel_vel[i];
                                                                                      current_steer_angle_dealign[1] = current_wheel_angle[i];}
                        else if (wheel_names_[i].find("rotate") != std::string::npos) desired_wheel_vel_dealign[3] = desired_wheel_vel[i];
                    }
                }
                else if (wheel_names_[i].find("rear") != std::string::npos)
                {
                    if (wheel_names_[i].find("left") != std::string::npos)
                    {
                        if (wheel_names_[i].find("steer") != std::string::npos){      desired_wheel_vel_dealign[4] = desired_wheel_vel[i];
                                                                                      current_steer_angle_dealign[2] = current_wheel_angle[i];}
                        else if (wheel_names_[i].find("rotate") != std::string::npos) desired_wheel_vel_dealign[5] = desired_wheel_vel[i];
                    }
                    else if (wheel_names_[i].find("right") != std::string::npos)
                    {
                        if (wheel_names_[i].find("steer") != std::string::npos){      desired_wheel_vel_dealign[6] = desired_wheel_vel[i];
                                                                                      current_steer_angle_dealign[3] = current_wheel_angle[i];}
                        else if (wheel_names_[i].find("rotate") != std::string::npos) desired_wheel_vel_dealign[7] = desired_wheel_vel[i];
                    }
                }
            }

            double b = wheel_offset_;
            double r = wheel_radius_;
            Eigen::MatrixXd C_p(wheel_names_.size(), 3);
            Eigen::MatrixXd C_q_inv(wheel_names_.size(), wheel_names_.size());
            for (size_t i = 0; i < current_steer_angle_dealign.size(); ++i) 
            {
                double K_xi = (i < 2) ? base_length_ / 2 : -base_length_ / 2;    // front is +1, rear is -1
                double K_yi = (i % 2 == 0) ? base_width_ / 2 : -base_width_ / 2; // left is +1, right is -1
                double phi = current_steer_angle_dealign[i];
                C_p.block<2, 3>(2 * i, 0) << 1, 0, -b * sin(phi) + K_yi,
                                             0, 1,  b * cos(phi) - K_xi;
                C_q_inv.block<2, 2>(2 * i, 2 * i) <<  b * sin(phi), r * cos(phi),
                                                     -b * cos(phi), r * sin(phi);
            }

            Eigen::MatrixXd C_p_inv = (C_p.transpose() * C_p).inverse() * C_p.transpose();
            Eigen::Vector3d base_vel = C_p_inv * C_q_inv * desired_wheel_vel_dealign;
            return base_vel;
        } 
        catch (const std::exception& e) 
        {
            std::cerr << "Error in FK calculation: " << e.what() << std::endl;
            return Eigen::Vector3d();
        }
    }

    Eigen::VectorXd CasterWheel::VelocityCommand(const Eigen::Vector3d& desired_base_vel, const Eigen::Vector3d& current_base_vel, const Eigen::VectorXd& current_wheel_angle) 
    {
        try 
        {
            Eigen::Vector3d desired_base_acc = (desired_base_vel - current_base_vel) * hz_;
            if (desired_base_acc.head<2>().norm() > lin_acc_limit_) 
            {
                desired_base_acc.head<2>() *= (lin_acc_limit_ / desired_base_acc.head<2>().norm());
            }
            if (std::abs(desired_base_acc[2]) > ang_acc_limit_) 
            {
                desired_base_acc[2] = std::copysign(ang_acc_limit_, desired_base_acc[2]);
            }

            Eigen::Vector3d new_base_vel = current_base_vel + desired_base_acc / hz_;
            if (new_base_vel.head<2>().norm() > lin_vel_limit_) 
            {
                new_base_vel.head<2>() *= (lin_vel_limit_ / new_base_vel.head<2>().norm());
            }
            if (std::abs(new_base_vel[2]) > ang_vel_limit_) 
            {
                new_base_vel[2] = std::copysign(ang_vel_limit_, new_base_vel[2]);
            }

            return IK(new_base_vel, current_wheel_angle);
        } 
        catch (const std::exception& e) 
        {
            std::cerr << "Error in VelocityCommand calculation: " << e.what() << std::endl;
            return Eigen::VectorXd();
        }
    }
}
