import rclpy
from rclpy.node import Node
from python.dock_detector_client import DockDetectionClient
from drn_perception_msgs.msg import DocksDetected
import time

class ExampleClientAsync(Node):

    def __init__(self):
        super().__init__('example_client_async')
        self.service_client = DockDetectionClient(self, '/service/dock_detection')
        self.publisher = self.create_publisher(DocksDetected, 'service_response_topic', 10)

        self.declare_parameter('direction', 'default')
        self.direction = self.get_parameter('direction').get_parameter_value().string_value

        self.experiment()

    def experiment(self):
        while True:
        # for i in range(100):
            msg = DocksDetected()
            response = self.service_client.send_request(self.direction)
            msg.dock_detected = response.dock_detected
            msg.docks = response.docks
            msg.info = response.info
            # print(type(response))
            # print(response)
            self.publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)

    example_client = ExampleClientAsync()
    example_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()