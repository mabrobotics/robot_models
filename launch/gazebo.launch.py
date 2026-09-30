"""Spawn the robot in Gazebo Sim with ros2_control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_arg = DeclareLaunchArgument('robot', default_value='hb50')
    robot = LaunchConfiguration('robot')

    xacro_path = PathJoinSubstitution(
        [FindPackageShare('robot_models'), 'urdf', robot, robot]
    )
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_path, '.urdf.xacro', ' sim:=gazebo']),
            value_type=str,
        )
    }

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
            )
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    return LaunchDescription(
        [
            robot_arg,
            gz_sim,
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[robot_description, {'use_sim_time': True}],
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                ],
                output='screen',
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', 'robot_description', '-name', robot, '-z', '0.35'],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['forward_position_controller'],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['wheel_velocity_controller'],
                output='screen',
                condition=IfCondition(EqualsSubstitution(robot, 'hb50w')),
            ),
        ]
    )
