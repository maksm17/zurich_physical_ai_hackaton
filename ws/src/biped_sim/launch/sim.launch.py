"""MuJoCo simulation + the same gait_node as on the robot.

  ros2 launch biped_sim sim.launch.py
  ros2 param set /gait walk true
  ros2 service call /sim/reset std_srvs/srv/Trigger
  ros2 launch biped_sim sim.launch.py sole_size:="[0.20, 0.045]"     # rocker sole R=20 cm
  ros2 launch biped_sim sim.launch.py flat_sole:=false foot_cut:=0.010 \
      gait_params:=$(ros2 pkg prefix biped_control)/share/biped_control/config/gait_filed10mm.yaml
  ros2 launch biped_sim sim.launch.py flat_sole:=false policy:=$PWD/rl/runs/round_v1/policy.npz
  ros2 param set /rl_policy enabled true
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
    xacro_file = os.path.join(desc, 'urdf', 'biped.xacro')

    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('gait', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('flat_sole', default_value='true'),
        DeclareLaunchArgument('sole_size', default_value='[0.04, 0.045, 0.004]',
                              description='[x,y,thick] flat | [R,width] rocker | [R] sphere'),
        DeclareLaunchArgument('foot_cut', default_value='0.0',
                              description='CAD feet filed flat by this much [m] (flat_sole:=false)'),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('camera_url', default_value='http://127.0.0.1:5000/status'),
        DeclareLaunchArgument('gesture_source', default_value='array'),
        DeclareLaunchArgument('gesture_topic', default_value='/camera/goal'),
        DeclareLaunchArgument('gesture_mode', default_value='hold'),
        DeclareLaunchArgument('gait_params', default_value=os.path.join(
            ctrl, 'config', 'gait.yaml')),
        DeclareLaunchArgument('policy', default_value='',
                              description='policy.npz -> RL policy instead of gait_node'),

        Node(package='biped_sim', executable='sim_node', name='mujoco_sim', output='screen',
             parameters=[{'xacro': xacro_file,
                          'viewer': ParameterValue(LaunchConfiguration('viewer'),
                                                   value_type=bool),
                          'flat_sole': ParameterValue(LaunchConfiguration('flat_sole'),
                                                      value_type=bool),
                          'sole_size': ParameterValue(LaunchConfiguration('sole_size'),
                                                      value_type=None),
                          'foot_cut': ParameterValue(LaunchConfiguration('foot_cut'),
                                                     value_type=float)}]),

        Node(package='biped_control', executable='gait_node', name='gait', output='screen',
             parameters=[LaunchConfiguration('gait_params')],
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('gait'), "' == 'true' and '",
                 LaunchConfiguration('policy'), "' == ''"]))),

        Node(package='biped_control', executable='rl_policy_node', name='rl_policy',
             output='screen', parameters=[{'policy': LaunchConfiguration('policy')}],
             condition=UnlessCondition(PythonExpression(
                 ["'", LaunchConfiguration('policy'), "' == ''"]))),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(
                 Command(['xacro ', xacro_file, ' use_mock:=true']), value_type=str)}]),

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
