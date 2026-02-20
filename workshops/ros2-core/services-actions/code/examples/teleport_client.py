# Import ROS2 Python client library
import rclpy

# Import the base Node class
from rclpy.node import Node

# Import the TeleportAbsolute service definition from turtlesim
from turtlesim.srv import TeleportAbsolute


# Create a class that inherits from ROS2 Node
class TeleportClient(Node):

    def __init__(self):
        # Initialize the node with the name 'teleport_client'
        super().__init__('teleport_client')

        # Create a service client
        # Arguments:
        # 1) Service type
        # 2) Service name
        self.client = self.create_client(
            TeleportAbsolute,
            '/turtle1/teleport_absolute'
        )

        # Wait until the service becomes available
        # This loop checks every 1 second
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        # Once the service is available, send the request
        self.send_request()

    def send_request(self):

        # Create a request object for the TeleportAbsolute service
        request = TeleportAbsolute.Request()

        # Set the target position (absolute coordinates)
        request.x = 1.0      # Target X position
        request.y = 8.0      # Target Y position
        request.theta = 0.0  # Target orientation (in radians)

        # Call the service asynchronously
        # call_async() returns a future object
        future = self.client.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        # If execution reaches here, the teleport is done
        self.get_logger().info('Teleport completed')


# Main function (entry point of the program)
def main():

    # Initialize ROS2 communication
    rclpy.init()

    # Create the node object
    node = TeleportClient()

    # Shutdown ROS2 cleanly
    rclpy.shutdown()


# This ensures the script runs only if executed directly
if __name__ == '__main__':
    main()
