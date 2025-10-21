![GitHub License](https://img.shields.io/github/license/Adorno-Lab/sas_robot_driver_unitree_z1)![Static Badge](https://img.shields.io/badge/ROS2-Jazzy-blue)![Static Badge](https://img.shields.io/badge/powered_by-DQ_Robotics-red)![Static Badge](https://img.shields.io/badge/SmartArmStack-green)![Static Badge](https://img.shields.io/badge/Ubuntu-24.04_LTS-orange)

# sas_differential_wheeled_robot_driver

<img width="600" alt="capybara_mobile" src="https://github.com/user-attachments/assets/fa2baa6d-d547-4883-8946-c265f5d73efc" />


A SAS driver implementation for differential mobile robots on CoppeliaSim

## Launch the driver

```shell
ros2 launch sas_differential_wheeled_robot_driver cs_example_launch.py
```

## Get the robot configuration

```shell
ros2 topic echo /sas_mobile_robot/pioneer_1/get/joint_states 
```
 
## Move your robot

```shell
ros2 topic pub /sas_mobile_robot/pioneer_1/set/target_joint_velocities std_msgs/msg/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data: [0.1, 0.0, 1.0]"
```




https://github.com/user-attachments/assets/aa7d553b-b273-49fa-b1d3-66d5f09696cd


