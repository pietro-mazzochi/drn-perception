import rclpy
from drn_perception_msgs.srv import DocksDetected


class DockDetectionClient():
    def __init__(self, node, service_name):
        self.node = node
        self.client = self.node.create_client(DocksDetected, service_name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(f'Service {service_name} not available, waiting again...')
        self.request = DocksDetected.Request()

    def send_request(self, direction):
        self.request.direction = direction
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

