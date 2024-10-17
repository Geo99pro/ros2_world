from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='gpg_test',
            executable='my_robot_controller',
            output='screen',
        ),
        Node(
            package='gpg_test',
            executable='my_robot_goal',
            output='screen',
        ),
           
    ])

