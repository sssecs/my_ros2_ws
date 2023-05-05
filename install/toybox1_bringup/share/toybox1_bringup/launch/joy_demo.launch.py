# to launch nodes
from launch import LaunchDescription
from launch_ros.actions import Node

# to launch launch files
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()




    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('articubot_one'),'launch/launch_sim.launch.py')
        )
    )

    ld.add_action(launch_sim)


    launch_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('teleop_twist_joy'),'launch/teleop-launch.py')]
        ),
        launch_arguments={'joy_config':'xbox'}.items()
    )

    ld.add_action(launch_joy)


    joy_node = Node(
        package = 'joy_test',
        executable = 'joy_to_velo',
    )

    ld.add_action(joy_node)


    return ld