"""Clean RViz view of the robot model (white background, no grid/TF/panels) + joint sliders."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('biped_description')
    desc = ParameterValue(Command(['xacro ', os.path.join(share, 'urdf', 'biped.xacro'),
                                   ' use_mock:=true']), value_type=str)
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': desc}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'config', 'screenshot.rviz')]),
    ])
