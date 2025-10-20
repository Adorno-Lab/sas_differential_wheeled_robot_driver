import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    launch_cs_example = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sas_differential_wheeled_robot_driver'), 'launch'),
            '/sas_differential_wheeled_robot_driver_launch.py'])
    )

    return LaunchDescription([
        launch_cs_example,
        Node(
            package='sas_differential_wheeled_robot_driver',
            executable='open_loop_test',
            output='screen',
            emulate_tty=True,
            name='sas_differential_wheeled_robot_driver_example_cpp'
        )
    ])