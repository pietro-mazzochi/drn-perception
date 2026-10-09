#from launch import LaunchContext, parse_launch_arguments
import launch 
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('drn_perception_reference_detection'),
        'config',
        'dock_detector_params.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument("debug", default_value="False", description="Debug"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        Node(
            package='drn_perception_reference_detection',
            executable='dock_detector_service_node',
            name='dock_detector_service_node',
            namespace = "drn_perception/dock_detector",
            output="screen",
            emulate_tty=True,
            parameters=[config,
                    {'debug': LaunchConfiguration('debug')},
                    {'base_frame': LaunchConfiguration('base_frame')}
                    ],
            remappings=[
                # topics
                ('/input/front_lidar', '/front_lidar'),
                ('/input/rear_lidar', '/rear_lidar'),
                ('/output/dock_detector', '/drn_perception/dock_detector/dock_detector'),
                ('/output/debug', '/drn_perception/dock_detector/debug')
            ]
        )
    ])