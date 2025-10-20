/*
# Copyright (c) 2025 Adorno-Lab
#
#    This is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License.
#    If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana, email: juanjose.quirozomana@manchester.ac.uk
#
#   Based on https://github.com/MarinhoLab/sas_ur_control_template/blob/main/src/joint_interface_example.cpp
#
# ################################################################*/


#include <rclcpp/rclcpp.hpp>
#include <sas_core/sas_clock.hpp>
#include <sas_robot_driver/sas_robot_driver_client.hpp>
#include <dqrobotics/utils/DQ_Math.h>

using namespace DQ_robotics;

#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}

int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");

    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("sas_differential_wheeled_robot_example");

    // 1 ms clock
    sas::Clock clock{0.002};
    clock.init();

    // Initialize the RobotDriverClient
    sas::RobotDriverClient rdi(node, "/sas_mobile_robot/pioneer_1");

    // Wait for RobotDriverClient to be enabled
    while(!rdi.is_enabled() && !kill_this_process)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        rclcpp::spin_some(node);
    }

    // Get topic information
    RCLCPP_INFO_STREAM(node->get_logger(),"topic_prefix = " << rdi.get_topic_prefix());




    // For some iterations. Note that this can be stopped with CTRL+C.
    while (!kill_this_process)
    {
        clock.update_and_sleep();

        // Move the robot with target planar joint velocities (x_dot, y_dot, phi_dot) expressed at body frame
        rdi.send_target_joint_velocities((VectorXd(3)<<0.1, 0.0, 0.1).finished());

        // Read the values sent by the RobotDriverServer
        auto q = rdi.get_joint_positions();
        RCLCPP_INFO_STREAM(node->get_logger(),"configuration = " << q.transpose());

        rclcpp::spin_some(node);
    }


    // Statistics
    RCLCPP_INFO_STREAM(node->get_logger(),"Statistics for the entire loop");

    RCLCPP_INFO_STREAM(node->get_logger(),"  Mean computation time: " << clock.get_statistics(
                                               sas::Statistics::Mean, sas::Clock::TimeType::Computational));
    RCLCPP_INFO_STREAM(node->get_logger(),"  Mean idle time: " << clock.get_statistics(
                                               sas::Statistics::Mean, sas::Clock::TimeType::Idle));
    RCLCPP_INFO_STREAM(node->get_logger(),"  Mean effective thread sampling time: " << clock.get_statistics(
                                               sas::Statistics::Mean, sas::Clock::TimeType::EffectiveSampling));

}
