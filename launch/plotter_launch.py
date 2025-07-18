from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odometry_diagnostics',
            executable='odom_plotter',
            name='odom_plotter',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
