import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('line_follower')
    world_file = os.path.join(pkg_share, 'worlds', 'figure8_world.world')
    model_file = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger_cam/model.sdf'

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    spawn = TimerAction(period=5.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'burger', '-file', model_file,
                 '-x', '-4.4', '-y', '1.5', '-z', '0.01', '-Y', '0.0'],
            output='screen'
        )
    ])

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        gazebo, spawn,
        Node(package='line_follower', executable='line_detector',
             name='line_detector', output='screen'),
        Node(package='line_follower', executable='obstacle_detector',
             name='obstacle_detector', output='screen'),
        Node(package='line_follower', executable='robot_controller',
             name='robot_controller', output='screen'),
    ])
