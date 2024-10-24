# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get URDF
    my_gazebo_world_file = os.path.join(get_package_share_directory('gpg_gazebo'), 'worlds',  'my_robot_world.sdf')

    gz_args = LaunchConfiguration('gz_args', default=my_gazebo_world_file + ' -r')

    my_urdf_file = os.path.join(get_package_share_directory('gpg_remote'), 'gopigo3.urdf')
    print(f'{my_urdf_file}')
    
    with open(my_urdf_file, 'r') as infp:
      robot_description_content = infp.read()
    robot_description = {"robot_description": robot_description_content, "use_sim_time": True}
    
                        
    robot_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[robot_description],
            )
    
    sim_publisher = Node(
        package="ros_gz_sim",
        executable="gz",
        name="gz_sim",
        output="screen",
        arguments=["sim", my_gazebo_world_file, "-r"],)


    sim_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
            ),
            launch_arguments={
                'gz_args': [my_gazebo_world_file],
                'on_exit_shutdown': 'True'
            }.items(),
        )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("gpg_remote"), "gpg_remote.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file], 
    )

    robot_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', "-z", "1"],
        output='screen',
    )

    nodes = [
        sim_publisher,
        robot_publisher,
        rviz_node,
        robot_spawner,
    ]

    return LaunchDescription(nodes)

    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("gpg_remote"),
    #         "controllers.yaml",
    #     ]
    # )
    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare("gpg_remote"), "gpg_remote.rviz"]
    # )
    
    #robot_publisher = ExecuteProcess(cmd=['ros2', 'topic', 'pub', '-1', '--keep-alive', '86400', '--qos-durability', 'transient_local', '/robot_description', 'std_msgs/String', 'data: \'' + robot_description_content + '\''])
    

    # control_node = Node(
    #    package="controller_manager",
    #    executable="ros2_control_node",
    #    parameters=[robot_controllers],
    #    output="both",
    # )
    
    # camera_config = os.path.join(
    #   get_package_share_directory('gpg_remote'),
    #   'camera.yaml'
    #   )
    # image_publisher_node = Node(
    #    package="gpg_remote",
    #    executable="image_publisher",
    #    parameters=[camera_config],
    #    output="both",
    # )

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    # gpg_remote_broadcaster_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner",
    #    arguments=["gpg_remote_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    # servo_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["servo_controller", "--controller-manager", "/controller_manager"],
    # )

    # robot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["diff_drive_controller", "-c", "/controller_manager"],
    # )

    # Delay rviz start after `joint_state_broadcaster`
    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )

    # # Delay start of robot_controller after `joint_state_broadcaster`
    # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[robot_controller_spawner, servo_controller_spawner, gpg_remote_broadcaster_spawner],
    #     )
    # )

    # nodes = [
    #    robot_publisher,
    #    control_node,
    #    image_publisher_node,
    #    joint_state_broadcaster_spawner,
    #    delay_rviz_after_joint_state_broadcaster_spawner,
    #    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    # ]
