/*
# Copyright (c) 2025 Adorno-Lab
#
#    This file is part of sas_robot_driver_unitree_z1.
#
#    sas_robot_driver_unitree_z1 is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver_unitree_z1 is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver_unitree_z1.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana, email: juanjose.quirozomana@manchester.ac.uk
#   Based on sas_robot_driver_ur.hpp 
#   (https://github.com/MarinhoLab/sas_robot_driver_ur/blob/main/include/sas_robot_driver_ur/sas_robot_driver_ur.hpp)
#
# ################################################################*/

#pragma once
#include <atomic>
#include <thread>

#include <sas_core/sas_robot_driver.hpp>

using namespace Eigen;
namespace sas
{

struct DifferentialWheeledRobotConfiguration
{
    double wheel_radius;
    double distance_between_wheels;
    std::string coppeliasim_robot_name;
    std::string coppeliasim_ip;
    int coppeliasim_port;
    int coppeliasim_TIMEOUT_IN_MILISECONDS;
    std::string left_motor_name;
    std::string right_motor_name;
    double thread_sampling_time_sec;
};


class DifferentialWheeledRobotDriver: public RobotDriver
{
private:
    DifferentialWheeledRobotConfiguration configuration_;

    class Impl;
    std::unique_ptr<Impl> impl_;



public:

    DifferentialWheeledRobotDriver(const DifferentialWheeledRobotDriver&)=delete;
    DifferentialWheeledRobotDriver()=delete;
    ~DifferentialWheeledRobotDriver();

    DifferentialWheeledRobotDriver(const DifferentialWheeledRobotConfiguration &configuration,
                                   std::atomic_bool* break_loops);

    VectorXd get_joint_positions() override;
    void set_target_joint_positions(const VectorXd& desired_joint_positions_rad) override;

    void set_target_joint_velocities(const VectorXd& desired_joint_velocities_rad_s) override;

    VectorXd get_joint_velocities() override;
    VectorXd get_joint_torques() override;

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;
};



}
