from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pub_sub',           # Replace with your package name
            executable='odom_plotter',  # Replace with your script/executable name
            name='odom_plotter',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
