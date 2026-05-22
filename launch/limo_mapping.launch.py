import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Optional: Path to a specific .rviz config file 
    # If you have a custom one, put the path here. 
    # Otherwise, it will just open a fresh RViz window.
    # rviz_config_dir = os.path.join(get_package_share_directory('limo_bringup'), 'rviz', 'slam.rviz')

    return LaunchDescription([
        # 1. The SLAM Node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                '/opt/ros/foxy/share/slam_toolbox/config/mapper_params_online_async.yaml',
                {
                    'base_frame': 'base_link', 
                    'use_sim_time': False
                }
            ]
        ),

        # 2. The RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # arguments=['-d', rviz_config_dir], # Uncomment this if you have a config file
            output='screen'
        )
    ])
