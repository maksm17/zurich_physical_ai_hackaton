"""Biped bringup: ros2_control (ST3215 or mock) + MPU6050 + Madgwick + gait.

  ros2 launch biped_control bringup.launch.py                   # real robot
  ros2 launch biped_control bringup.launch.py use_mock:=true imu:=false rviz:=true   # laptop
  ros2 launch biped_control bringup.launch.py gait:=false       # only hardware, e.g. calibration
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory('biped_description')
    ctrl = get_package_share_directory('biped_control')

    use_mock = LaunchConfiguration('use_mock')
    read_only = LaunchConfiguration('read_only')
    robot_description = ParameterValue(Command([
        'xacro ', os.path.join(desc, 'urdf', 'biped.xacro'),
        ' use_mock:=', use_mock,
        ' left_port:=', LaunchConfiguration('left_port'),
        ' right_port:=', LaunchConfiguration('right_port'),
    ]), value_type=str)

    controllers = os.path.join(ctrl, 'config', 'controllers.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='false'),
        DeclareLaunchArgument('left_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('right_port', default_value='/dev/ttyAMA2'),
        DeclareLaunchArgument('imu', default_value='true'),
        DeclareLaunchArgument('gait', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        # read_only: hardware stays INACTIVE (torque off), only joint_states are published
        DeclareLaunchArgument('read_only', default_value='false'),
        DeclareLaunchArgument('gait_params', default_value=os.path.join(
            ctrl, 'config', 'gait.yaml')),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),

        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[controllers], output='screen',
             condition=UnlessCondition(read_only)),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[controllers,
                         {'hardware_components_initial_state.inactive': ['BipedSystem']}],
             output='screen', condition=IfCondition(read_only)),

        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', 'position_controller'],
             condition=UnlessCondition(read_only)),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster'], condition=IfCondition(read_only)),

        Node(package='biped_control', executable='mpu6050_node',
             condition=IfCondition(LaunchConfiguration('imu'))),

        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='imu_filter', parameters=[os.path.join(ctrl, 'config', 'madgwick.yaml')],
             condition=IfCondition(LaunchConfiguration('imu'))),

        Node(package='biped_control', executable='gait_node', name='gait',
             parameters=[LaunchConfiguration('gait_params')], output='screen',
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('gait'), "' == 'true' and '", read_only,
                 "' != 'true'"]))),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(desc, 'config', 'display.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
