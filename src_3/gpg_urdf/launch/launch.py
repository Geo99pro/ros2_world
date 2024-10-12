from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_line_name = 'gpg.urdf.xml'
    print("urdf_line_name : {}".format(urdf_line_name))
    urdf = os.path.join(get_package_share_directory('gpg_urdf'), urdf_line_name)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf]
        )
    ])