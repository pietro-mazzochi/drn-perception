from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
)
import os

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    service_launch_path = os.path.join(
        get_package_share_directory('drn_perception_reference_detection'),
        'drn_perception_reference_detection_launch.py'
    )

    experiment_collector = Node(
            package='drn_perception_reference_detection',
            executable='experiment_node',
            name='experiment_node',
            namespace = "drn_perception/dock_detector",
            output="screen",
            emulate_tty=True,
            parameters=[{'direction': LaunchConfiguration('direction')},
                        ],
            remappings=[
                #service:
                ('/service/dock_detection', '/drn_perception/dock_detector/dock_detection')
                ])
    
    delayed_experiment_collector_node = TimerAction(
        period=10.0,
        actions=[experiment_collector]
        )
    # Run the node
    return LaunchDescription([
        DeclareLaunchArgument("direction", default_value="front", description="Direction to dock detection"),
        DeclareLaunchArgument("debug", default_value="False", description="Debug"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(service_launch_path),
        ),
        delayed_experiment_collector_node
        ]
    )
