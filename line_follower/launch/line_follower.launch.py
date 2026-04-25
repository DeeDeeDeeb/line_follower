import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('line_follower')
    world_file = os.path.join(pkg_share, 'worlds', 'line_world.world')
    model_file = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger_cam/model.sdf'

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen'
    )

    # Delay spawn by 5 seconds so Gazebo is ready
    spawn = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', 'burger',
                    '-file', model_file,
                    '-x', '-1.5', '-y', '0.0', '-z', '0.01',
                ],
                output='screen'
            )
        ]
    )

    line_detector = Node(
        package='line_follower',
        executable='line_detector',
        name='line_detector',
        output='screen',
    )

    obstacle_detector = Node(
        package='line_follower',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen',
    )

    robot_controller = Node(
        package='line_follower',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        gazebo,
        spawn,
        line_detector,
        obstacle_detector,
        robot_controller,
    ])
