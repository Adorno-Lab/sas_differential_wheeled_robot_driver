#include "sas_differential_wheeled_robot_driver/sas_differential_wheeled_robot_driver.hpp"
#include <memory>
#include <sas_core/eigen3_std_conversions.hpp>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>


namespace sas
{

class DifferentialWheeledRobotDriver::Impl
{

public:
    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> cs_;
    DQ robot_pose_;
    Impl()
    {

    };



};

DifferentialWheeledRobotDriver::~DifferentialWheeledRobotDriver()
{

}

/**
 * @brief DifferentialWheeledRobotDriver::DifferentialWheeledRobotDriver ctor of the class
 * @param configuration
 * @param break_loops
 */
DifferentialWheeledRobotDriver::DifferentialWheeledRobotDriver(const DifferentialWheeledRobotConfiguration &configuration, std::atomic_bool *break_loops):
    RobotDriver(break_loops),
    configuration_(configuration)
{
    impl_ = std::make_unique<DifferentialWheeledRobotDriver::Impl>();
    impl_->cs_ = std::make_shared<DQ_CoppeliaSimInterfaceZMQ>();
}


/**
 * @brief DifferentialWheeledRobotRobotDriver::get_joint_positions returns the robot configuration.
 * @return A vector 3x1 containing the robot configuration. E.g., q=[x,y, phi].
 */
VectorXd DifferentialWheeledRobotDriver::get_joint_positions()
{
    DQ x = impl_->robot_pose_ = impl_->cs_->get_object_pose(configuration_.coppeliasim_robot_name);
    auto axis = x.rotation_axis().vec4();
    if (axis(3)<0)
        x = -x;
    auto p = x.translation().vec3();
    auto rangle = x.P().rotation_angle();
    return (VectorXd(3)<< p(0), p(1), rangle).finished();
}

void DifferentialWheeledRobotDriver::set_target_joint_positions([[maybe_unused]] const VectorXd& desired_joint_positions_rad)
{
    //throw std::runtime_error(std::string(__FUNCTION__)+" is not available.");
}

/**
 * @brief DifferentialWheeledRobotRobotDriver::set_target_joint_velocities sets the target task-space velocities of the mobile platform. The
 *                      target velocities are expressed in the body frame.
 * @param target_velocities A vector of 3x1 containing the desired velocities. E.g, u = [x_dot, y_dot, phi_dot].
 */
void DifferentialWheeledRobotDriver::set_target_joint_velocities(const VectorXd &target_velocities)
{
    const double& v   = target_velocities(0);
    const double& w   = target_velocities(2);

    const VectorXd q = get_joint_positions();
    const double& l = configuration_.distance_between_wheels;
    const double& r = configuration_.wheel_radius;

    const double wr = (v + l*w)/r;
    const double wl = (v - l*w)/r;
    const VectorXd u = (VectorXd(2) << wl, wr).finished();

    impl_->cs_->set_joint_target_velocities({configuration_.left_motor_name,  configuration_.right_motor_name}, u);


}


VectorXd DifferentialWheeledRobotDriver::get_joint_velocities()
{
    //throw std::runtime_error(std::string(__FUNCTION__)+" is not available.");
    return VectorXd::Zero(3);
}


VectorXd DifferentialWheeledRobotDriver::get_joint_torques()
{
    return VectorXd::Zero(3);
}


/**
 * @brief DifferentialWheeledRobotRobotDriver::connect
 */
void DifferentialWheeledRobotDriver::connect()
{
    if (impl_->cs_)
    {
        try
        {
            if (!impl_->cs_->connect(configuration_.coppeliasim_ip,
                                     configuration_.coppeliasim_port,
                                     configuration_.coppeliasim_TIMEOUT_IN_MILISECONDS))
                throw std::runtime_error("Unable to connect to CoppeliaSim.");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::cerr<<"Connected to CoppeliaSim! "<<std::endl;
            impl_->robot_pose_ = impl_->cs_->get_object_pose(configuration_.coppeliasim_robot_name);
        }
        catch (std::exception& e)
        {
            std::cout<<e.what()<<std::endl;
        }
    }
}


/**
 * @brief DifferentialWheeledRobotRobotDriver::disconnect
 */
void DifferentialWheeledRobotDriver::disconnect()
{

}


/**
 * @brief DifferentialWheeledRobotRobotDriver::initialize
 */
void DifferentialWheeledRobotDriver::initialize()
{

}

/**
 * @brief DifferentialWheeledRobotRobotDriver::deinitialize
 */
void DifferentialWheeledRobotDriver::deinitialize()
{

}




}
