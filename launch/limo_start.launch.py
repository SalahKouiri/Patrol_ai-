import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Define Arguments
    use_slam = LaunchConfiguration('use_slam', default='false')

    # 2. Setup Paths
    limo_base_dir = get_package_share_directory('limo_base')
    ydlidar_ros_dir = get_package_share_directory("ydlidar_ros2_driver")
    limo_description_dir = get_package_share_directory('limo_description')
    # Path for your bringup package where twist_mux.yaml lives
    limo_bringup_dir = get_package_share_directory('limo_bringup')

    # 3. Process the URDF
    xacro_file = os.path.join(limo_description_dir, 'urdf', 'limo_four_diff.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Path to your twist_mux config
    twist_mux_params = os.path.join(limo_bringup_dir, 'param', 'twist_mux.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_slam', default_value='false', description='Whether to start SLAM'),

        # The Physical Body
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_raw, 'use_sim_time': False}]
        ),

        # Twist Mux - The Switchboard
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out', '/cmd_vel')]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([limo_base_dir,'/launch','/limo_base.launch.py']),
        ),  

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ydlidar_ros_dir,'/launch','/ydlidar_launch.py']),
        ),

        # The Brain (SLAM)
        Node(
            condition=IfCondition(use_slam),
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                '/opt/ros/foxy/share/slam_toolbox/config/mapper_params_online_async.yaml',
                {'base_frame': 'base_link'}
            ]
        ),
    ])
