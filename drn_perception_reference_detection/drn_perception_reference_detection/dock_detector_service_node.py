import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from drn_perception_msgs.msg import Dock
from drn_perception_msgs.srv import DocksDetected
from std_msgs.msg import Header
from rcl_interfaces.msg import ParameterType
from python.dock_detection import DockDetection
from python.pcd_buffer import PointCloudBuffer

import open3d as o3d
import ast
from transforms3d.euler import euler2quat

import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
class DockDetectorService(Node):

    def __init__(self):
        super().__init__('dock_detector_service_node')
        self.srv = self.create_service(DocksDetected, 'dock_detection', self.dock_detection_callback)

        self.declare_parameter('z_min', -0.3)
        self.declare_parameter('z_max', 0.3)
        self.declare_parameter('min_reflectivity', 250)
        self.declare_parameter('eps', 0.1)
        self.declare_parameter('min_samples', 9)
        self.declare_parameter('cluster_distance', 0.75)
        self.declare_parameter('cluster_distance_threshold', 0.05)
        self.declare_parameter('cluster_size', [0.175, 0.175 ])# (width, height)
        self.declare_parameter('cluster_size_threshold', [0.02, 0.02 ])# (width, height)
        self.declare_parameter('filter_radius', 0.5)
        self.declare_parameter('filter_z_offset', 0.07)
        self.declare_parameter('position_offset', 0.02)
        self.declare_parameter('angle_offset', 0.5)
        self.declare_parameter('max_interations', 10)
        self.declare_parameter('voxel_size', 0.02)
        self.declare_parameter('similarity_threshold', 5.0)
        self.declare_parameter('max_interations_3d', 10)
        self.declare_parameter('buffer_max_size', 10)
        self.declare_parameter('reference', 'default')
        self.declare_parameter('debug', False)
        self.declare_parameter('base_frame', 'base_link')

        z_min = self.get_parameter('z_min').get_parameter_value().double_value
        z_max = self.get_parameter('z_max').get_parameter_value().double_value
        min_reflectivity = self.get_parameter('min_reflectivity').get_parameter_value().integer_value
        eps = self.get_parameter('eps').get_parameter_value().double_value
        min_samples = self.get_parameter('min_samples').get_parameter_value().integer_value
        cluster_distance = self.get_parameter('cluster_distance').get_parameter_value().double_value
        cluster_distance_threshold = self.get_parameter('cluster_distance_threshold').get_parameter_value().double_value
        cluster_size = self.get_parameter('cluster_size').get_parameter_value().double_array_value
        cluster_size_threshold = self.get_parameter('cluster_size_threshold').get_parameter_value().double_array_value
        filter_radius = self.get_parameter('filter_radius').get_parameter_value().double_value
        filter_z_offset = self.get_parameter('filter_z_offset').get_parameter_value().double_value
        position_offset = self.get_parameter('position_offset').get_parameter_value().double_value
        angle_offset = self.get_parameter('angle_offset').get_parameter_value().double_value
        max_interations = self.get_parameter('max_interations').get_parameter_value().integer_value
        voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        similarity_threshold = self.get_parameter('similarity_threshold').get_parameter_value().double_value
        max_interations_3d = self.get_parameter('max_interations_3d').get_parameter_value().integer_value
        buffer_size = self.get_parameter('buffer_max_size').get_parameter_value().integer_value
        reference = self.get_parameter('reference').get_parameter_value().string_value

        reference = np.array(ast.literal_eval(reference))

        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.base_frame_ = self.base_frame
        self.source_frame = 'default'

        self.subscription_front_lidar = self.create_subscription(
            PointCloud2,
            '/input/front_lidar',
            self.listener_callback_front,
            10)

        self.subscription_rear_lidar = self.create_subscription(
            PointCloud2,
            '/input/rear_lidar',
            self.listener_callback_rear,
            10)
        
        if self.debug:
            self.debug_publisher = self.create_publisher(PointCloud2, '/output/debug', 10)
        
        self.buffer_front = PointCloudBuffer(buffer_size)
        self.buffer_rear = PointCloudBuffer(buffer_size)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.rd = DockDetection(
            # set reference
            reference=reference,
            # filter
            z_min=z_min,
            z_max=z_max,
            min_reflectivity=min_reflectivity,
            # DBSCAN parameters
            eps=eps,
            min_samples=min_samples,
            # reflectivity cluster size check
            cluster_distance=cluster_distance,
            cluster_distance_threshold=cluster_distance_threshold,
            cluster_size=cluster_size, # (width, height)
            cluster_size_threshold=cluster_size_threshold, # (width, height)
            # filter
            filter_radius=filter_radius,
            filter_z_offset=filter_z_offset,
            # 2d match
            position_offset=position_offset,
            angle_offset=angle_offset,
            max_interations=max_interations,
            # 3d match
            voxel_size=voxel_size,
            similarity_threshold=similarity_threshold,
            max_interations_3d=max_interations_3d
            )


    def dock_detection_callback(self, request, response):
        match request.direction:
            case 'front':
                pointcloud_data = self.buffer_front.get_data()
            case 'rear':
                pointcloud_data = self.buffer_rear.get_data()
            case _:
                response.info = 'error: request direction'
                return response
        self.source_frame = f"{request.direction}_lidar"
        if (request.base_frame):
            self.base_frame = request.base_frame
        else:
            self.base_frame = self.base_frame_
        try:
            self.tf_buffer.lookup_transform(self.base_frame, self.source_frame, rclpy.time.Time())
        except:
            response.info = f"error: request base frame {self.base_frame} doesnt exist"
            return response
        response.dock_detected, response.docks, response.info = self.process_pointcloud(pointcloud_data)
        if self.debug:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = f"{request.direction}_lidar"
            fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
            debug_msg = point_cloud2.create_cloud(header, fields, pointcloud_data)
            debug_msg.is_dense = True
            
            self.debug_publisher.publish(debug_msg)

        return response

    def listener_callback_front(self, msg):
        pts = point_cloud2.read_points(msg, field_names=['x','y','z','intensity'])
        self.buffer_front.set_data(pts)

    def listener_callback_rear(self, msg):
        pts = point_cloud2.read_points(msg, field_names=['x','y','z','intensity'])
        self.buffer_rear.set_data(pts)

    def transform_pose(self, pose_in_source_frame, target_frame):
        transform = self.tf_buffer.lookup_transform(target_frame, pose_in_source_frame.header.frame_id, rclpy.time.Time())
        pose_in_target_frame = do_transform_pose_stamped(pose_in_source_frame, transform)
        return pose_in_target_frame
        
    def process_msg_instance(self, center, angle, error):
        msg_instance = Dock()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.source_frame

        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = center[0]
        pose_stamped.pose.position.y = center[1]
        pose_stamped.pose.position.z = center[2]
        q = euler2quat(0, 0, angle)
        pose_stamped.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
        pose_stamped.header = header
        msg_instance.pose_stamped = self.transform_pose(pose_stamped, self.base_frame)
        msg_instance.dock_distance = np.sqrt(msg_instance.pose_stamped.pose.position.x**2 + msg_instance.pose_stamped.pose.position.y**2)
        msg_instance.detection_error = error
        return msg_instance

    def process_pointcloud(self, msg):
        flag = False
        docks = []
        if len(msg) == 0:
            info = 'error: empty buffer'
            return flag, docks, info
        else:
            data_croped_z = self.rd.crop_zaxis(np.array(msg.tolist()))
            reflectivity_clusters, info = self.rd.get_reflectivity_cluster(data_croped_z)
            if len(reflectivity_clusters) == 0:
                return flag, docks, info
            docks_centers, docks_angles, info = self.rd.get_dock_pose(reflectivity_clusters)
            if len(docks_centers) == 0:
                return flag, docks, info
            else:
                docks_filtered = self.rd.filter_dock_points(data_croped_z, docks_centers)
                for i, (center, angle) in enumerate(zip(docks_centers, docks_angles)):
                    # self.get_logger().info(f'Cluster | Dock {i} | Center: {center} | Angle: {angle}')
                    msg_instance = self.process_msg_instance(center, angle, -1.0)
                    docks.append(msg_instance)

                centers_2d, angles_2d, errors_2d, iteractions_2d = self.rd.optimize_docks_position_2d(docks_filtered, docks_centers, docks_angles)
                for i, (center, angle, error, iter) in enumerate(zip(centers_2d, angles_2d, errors_2d, iteractions_2d)):
                    # self.get_logger().info(f'2D Match | Dock {i} | Center: {center} | Angle: {angle} | Error: {error:.4f} | Iter: {iter}')
                    msg_instance = self.process_msg_instance(center, angle, error)
                    docks.append(msg_instance)
                
                centers_3d, angles_3d, errors_3d = self.rd.optimize_docks_position_3d(docks_filtered, docks_centers, docks_angles)
                for i, (center, angle, error,) in enumerate(zip(centers_3d, angles_3d, errors_3d)):
                    # self.get_logger().info(f'3D Match | Dock {i} | Center: {center} | Angle: {angle} | Error: {error:.4f}')
                    msg_instance = self.process_msg_instance(center, angle, error)
                    docks.append(msg_instance)

                info = f'{i+1} docks clusters, {i+1} docks 2d, {i+1} docks 3d'
                flag = True
        return flag, docks, info

def main(args=None):
    rclpy.init(args=args)

    dock_detector_service = DockDetectorService()

    rclpy.spin(dock_detector_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()