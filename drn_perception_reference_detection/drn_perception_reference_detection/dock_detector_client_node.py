import rclpy
from rclpy.node import Node
from python.dock_detector_client import DockDetectionClient

class ExampleClientAsync(Node):

    def __init__(self):
        super().__init__('example_client_async')
        self.service_client = DockDetectionClient(self, '/service/dock_detection')

def main(args=None):
    rclpy.init(args=args)

    example_client = ExampleClientAsync()
    direction = example_client.declare_parameter('direction', 'default').get_parameter_value().string_value
    
    response = example_client.service_client.send_request(direction)
    example_client.get_logger().info(f'Result of dock_detection for {example_client.service_client.request.direction}: dock_detected is {response.dock_detected}, {len(response.docks)} docks')
    example_client.get_logger().info(f'{response}')
    example_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()