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
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
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
        DeclareLaunchArgument('policy', default_value=''),
        DeclareLaunchArgument('imu_power_gpio', default_value='17'),
        DeclareLaunchArgument('read_only', default_value='false'),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('camera_url', default_value='http://127.0.0.1:5000/status'),
        DeclareLaunchArgument('gesture_source', default_value='array'),
        DeclareLaunchArgument('gesture_topic', default_value='/camera/goal'),
        DeclareLaunchArgument('gesture_mode', default_value='hold'),
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

        Node(package='biped_control', executable='mpu6050_node', output='screen',
             respawn=True, respawn_delay=2.0,
             parameters=[{'power_gpio': ParameterValue(LaunchConfiguration('imu_power_gpio'),
                                                       value_type=int)}],
             condition=IfCondition(LaunchConfiguration('imu'))),

        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='imu_filter', parameters=[os.path.join(ctrl, 'config', 'madgwick.yaml')],
             condition=IfCondition(LaunchConfiguration('imu'))),

        Node(package='biped_control', executable='gait_node', name='gait',
             parameters=[LaunchConfiguration('gait_params')], output='screen',
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('gait'), "' == 'true' and '", read_only,
                 "' != 'true' and '", LaunchConfiguration('policy'), "' == ''"]))),

        Node(package='biped_control', executable='rl_policy_node', name='rl_policy',
             output='screen',
             parameters=[{'policy': PathJoinSubstitution(
                 [ctrl, 'policies', [LaunchConfiguration('policy'), '.npz']])}],
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('policy'), "' != '' and '", read_only, "' != 'true'"]))),

        Node(package='biped_control', executable='nav_fsm', name='nav_fsm', output='screen',
             parameters=[{'camera_url': LaunchConfiguration('camera_url'),
                          'gesture_source': LaunchConfiguration('gesture_source'),
                          'gesture_mode': LaunchConfiguration('gesture_mode'),
                          'gesture_array_topic': LaunchConfiguration('gesture_topic')}],
             condition=IfCondition(LaunchConfiguration('nav'))),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(desc, 'config', 'display.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
