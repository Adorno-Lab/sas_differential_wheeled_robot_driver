
#include <rclcpp/rclcpp.hpp>
#include <sas_robot_driver/sas_robot_driver_ros.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <sas_differential_wheeled_robot_driver/sas_differential_wheeled_robot_driver.hpp>
#include <dqrobotics/utils/DQ_Math.h>


/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>

static std::atomic_bool kill_this_process(false);

void sig_int_handler(int);

void sig_int_handler(int)
{
    kill_this_process = true;
}

int main(int argc, char** argv)
{

    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");


    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("sas_differential_wheeled_robot_driver");


    try
    {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");

        sas::DifferentialWheeledRobotConfiguration configuration;

        sas::get_ros_parameter(node,"wheel_radius",configuration.wheel_radius);
        sas::get_ros_parameter(node,"distance_between_wheels",configuration.distance_between_wheels);
        sas::get_ros_parameter(node,"coppeliasim_robot_name",configuration.coppeliasim_robot_name);
        sas::get_ros_parameter(node,"coppeliasim_ip",configuration.coppeliasim_ip);
        sas::get_ros_parameter(node, "coppeliasim_port", configuration.coppeliasim_port);

        sas::get_ros_parameter(node,"coppeliasim_TIMEOUT_IN_MILISECONDS",
                               configuration.coppeliasim_TIMEOUT_IN_MILISECONDS);
        sas::get_ros_parameter(node,"left_motor_name",configuration.left_motor_name);
        sas::get_ros_parameter(node,"right_motor_name",configuration.right_motor_name);
        sas::get_ros_parameter(node, "thread_sampling_time_sec", configuration.thread_sampling_time_sec);



        sas::RobotDriverROSConfiguration robot_driver_ros_configuration;
        sas::get_ros_parameter(node,"thread_sampling_time_sec",robot_driver_ros_configuration.thread_sampling_time_sec);
        robot_driver_ros_configuration.robot_driver_provider_prefix = node->get_name();

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Parameters OK.");
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Instantiating DifferentialWheeledRobotDriver.");
        auto differential_robot_driver= std::make_shared<sas::DifferentialWheeledRobotDriver>(configuration,
                                                                                             &kill_this_process);

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Instantiating RobotDriverROS.");
        sas::RobotDriverROS robot_driver_ros(node,
                                             differential_robot_driver,
                                             robot_driver_ros_configuration,
                                             &kill_this_process);
        robot_driver_ros.control_loop();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
    }

    return 0;
}
