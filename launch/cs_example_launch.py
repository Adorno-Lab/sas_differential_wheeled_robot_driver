"""
This file is based on the sas_kuka_control_template
https://github.com/MarinhoLab/sas_kuka_control_template/blob/main/launch/real_robot_launch.py

Run this script in a different terminal window or tab. Be ready to close this, as this activates the real robot if the
connection is successful.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
                    'sigterm_timeout',
                    default_value='30'
                ),
        Node(
            package='sas_differential_wheeled_robot_driver',
            executable='sas_differential_wheeled_robot_driver_node',
            name='pioneer_1',
            namespace="sas_pioneer",
            output="screen",
            parameters=[{
                "wheel_radius": 0.1,
                "distance_between_wheels":0.331,
                "coppeliasim_robot_name": "PioneerP3DX", 
                "coppeliasim_ip": "localhost",
                "coppeliasim_port": 23000,
                "coppeliasim_TIMEOUT_IN_MILISECONDS": 1000,
                "left_motor_name": "PioneerP3DX/leftMotor",
                "right_motor_name": "PioneerP3DX/rightMotor",
                "thread_sampling_time_sec": 0.002,    
            }]
        ),

    ])
