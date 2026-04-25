 import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    pkg        = get_package_share_directory('imu_robot')
    urdf_file  = os.path.join(pkg, 'urdf',   'imu_robot.urdf.xacro')
    rviz_file  = os.path.join(pkg, 'rviz',   'imu_robot.rviz')
    param_file = os.path.join(pkg, 'config', 'imu_params.yaml')
    world_file = os.path.join(pkg, 'worlds', 'ramp_world.world')

    robot_description = xacro.process_file(urdf_file).toxml()

    return LaunchDescription([

        # Gazebo with ramp world
        ExecuteProcess(
            cmd=['gazebo', '--verbose',
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so',
                 world_file],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Spawn robot behind the ramp
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'imu_robot',
                '-x', '-1.0', '-y', '0.0', '-z', '0.1'
            ],
            output='screen'
        ),

        # Node 1 — Tilt detector (stops robot on tilt)
        Node(
            package='imu_robot',
            executable='tilt_detector',
            name='tilt_detector',
            output='screen',
            parameters=[param_file]
        ),

        # Node 2 — Motion classifier (STATIONARY / MOVING / TURNING)
        Node(
            package='imu_robot',
            executable='motion_classifier',
            name='motion_classifier',
            output='screen',
            parameters=[param_file]
        ),

        # Node 3 — IMU reader + CSV logger (9-channel filtered pipeline)
        Node(
            package='imu_robot',
            executable='imu_reader',
            name='imu_reader',
            output='screen',
            parameters=[param_file]
        ),

        # Node 4 — Live IMU graph (rolling plots + 2-D robot path)  ← ADDED
        Node(
            package='imu_robot',
            executable='imu_graph',
            name='imu_graph',
            output='screen',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            output='screen'
        ),

    ])
