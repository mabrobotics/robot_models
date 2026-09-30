"""View the robot in rviz with joint sliders."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        'robot', default_value='hb50', description="Which robot's xacro to load."
    )
    robot = LaunchConfiguration('robot')

    xacro_path = PathJoinSubstitution(
        [FindPackageShare('robot_models'), 'urdf', robot, robot]
    )

    robot_description = {
        # value_type=str: skip YAML parsing of the URDF
        'robot_description': ParameterValue(
            Command(
                [
                    'xacro ',
                    xacro_path,
                    '.urdf.xacro',
                    ' use_ros2_control:=false',
                ]
            ),
            value_type=str,
        )
    }

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('robot_models'), 'rviz', 'view.rviz']
    )

    return LaunchDescription(
        [
            robot_arg,
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[robot_description],
            ),
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                output='screen',
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
            ),
        ]
    )
