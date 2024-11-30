# Introduction

This package contains nodes and launch files, to make teleoperation of the big omnidirectional robot of IIT department easy. The package takes advantage of the standard `geometry_msgs/msg/Twist` message of ROS2, and translates this to the control interface of the robot.

The robot is currently running on ROS2 Foxy. If this system is not available, this package can be easily used with the (following docker image)[https://github.com/BME-IIT-robotcsapat/iit-bigomni-docker_teleop] from other Ubuntu versions.

# Nodes
- **twist2bigomni_control**: translates linear X and Y velocities and angular Z velocities of  `geometry_msgs/msg/Twist` for the low level `std_msgs/msg/UInt8MultiArray` control message of the robot, that contains the 6 PWM duty values for the 3 H-bridge motor drivers.

# Launch files
- **bigomni_joy_teleop.py**: launches the **twist2bigomni_control**, **joy** and **teleop_twist_joy** nodes to interface with a joystick. The latter having the parameters defined in `./config/joy_teleop_params.yaml`
