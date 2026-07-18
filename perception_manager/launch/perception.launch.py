"""Launch the Robot Perception Manager system.

Starts both nodes under a configurable namespace and loads all parameters
from YAML (no hardcoded values in the nodes).

Usage:
    ros2 launch perception_manager perception.launch.py
    ros2 launch perception_manager perception.launch.py namespace:=robot1
    ros2 launch perception_manager perception.launch.py params_file:=/path/to/other.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='perception',
        description='Namespace for all perception nodes.',
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('perception_manager'), 'config', 'perception_params.yaml']
        ),
        description='Full path to the YAML parameter file.',
    )

    perception_manager_node = Node(
        package='perception_manager',
        executable='perception_manager_node',
        name='perception_manager',
        namespace=namespace,
        parameters=[params_file],
        output='screen',
    )

    camera_tf_broadcaster_node = Node(
        package='perception_manager',
        executable='camera_tf_broadcaster_node',
        name='camera_tf_broadcaster',
        namespace=namespace,
        parameters=[params_file],
        output='screen',
    )

    return LaunchDescription([
        declare_namespace,
        declare_params_file,
        perception_manager_node,
        camera_tf_broadcaster_node,
    ])
