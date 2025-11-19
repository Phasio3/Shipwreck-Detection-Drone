from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    world_file = os.path.join(
        get_package_share_directory('drone_world'),
        'worlds',
        'flat_world.sdf'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file],
            output='screen'
        ),
    ])

