from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    pkg = 'drone_world'
    pkg_share = os.path.join(
        os.path.expanduser('~/ros2_drones_ws/install'),
        'drone_world/share/drone_world'
    )

    # --- Launch Gazebo world ---
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch/start_world.launch.py')
        )
    )

    # --- Random motion node ---
    random_move = Node(
        package='drone_world',
        executable='exploration_random.py',
        name='random_movement',
        output='screen'
    )

    # --- Exploration mapper node ---
    mapper = Node(
        package='drone_world',
        executable='exploration_mapper.py',
        name='exploration_mapper',
        output='screen'
    )

    # --- RVIZ2 with map ---
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz/exploration.rviz')],
        # si le fichier n'existe pas, RViz démarre vide — pas grave
    )

    return LaunchDescription([
        world_launch,
        random_move,
        mapper,
        rviz
    ])
