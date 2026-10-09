#from launch import LaunchContext, parse_launch_arguments
import launch 
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('drn_perception_reference_detection'),
        'config',
        'dock_detector_params.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
                "direction",
                default_value=" ",
                description="Direction to dock detection",
            ),
        Node(
            package='drn_perception_reference_detection',
            executable='dock_detector_client_node',
            name='dock_detector_client_node',
            namespace = "drn_perception/dock_detector",
            output="screen",
            emulate_tty=True,
            parameters=[config,
                        {'direction': LaunchConfiguration('direction')},
                        ],
            remappings=[
                # topics
                ('/input/front_lidar', '/front_lidar'),
                ('/input/back_lidar', '/back_lidar'),
                ('/output/dock_detector', 'dock_detector'),
                #service:
                ('/service/dock_detection', '/drn_perception/dock_detector/dock_detection')
            ]
        ),
    ])