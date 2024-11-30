import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    joy2twist_params = os.path.join(
            get_package_share_directory('bigomni_teleop'),
            'config',
            'joy_teleop_params.yaml')
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node'),
        launch_ros.actions.Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='joy2twist',
            parameters=[joy2twist_params]),
        launch_ros.actions.Node(
            package='bigomni_teleop',
            executable='twist2bigomni_control',
            name='Twist2BigOmniControl'),
    ])
