import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, TeleportAbsolute


class SpawnTurtle(Node):
    def __init__(self):
        super().__init__('spawn_turtle')
        
        # Create client for spawn service
        self.spawn_client = self.create_client(Spawn, '/spawn')
        
        # Create client for teleport service
        self.teleport_client = self.create_client(
            TeleportAbsolute, 
            '/turtle2/teleport_absolute')
        
        self.get_logger().info('Spawn Turtle node started')
        
        # Execute the task
        self.execute_task()

    def execute_task(self):
        # Step 1: Spawn turtle2
        self.spawn_turtle2()
        
        # Step 2: Move turtle2 to target position
        self.move_turtle2()

    def spawn_turtle2(self):
        # Wait for spawn service to be available
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        # Create spawn request
        spawn_request = Spawn.Request()
        spawn_request.x = 5.0
        spawn_request.y = 5.0
        spawn_request.theta = 0.0
        spawn_request.name = 'turtle2'
        
        # Call spawn service
        future = self.spawn_client.call_async(spawn_request)
        rclpy.spin_until_future_complete(self, future)
        
        # Check result
        if future.result() is not None:
            self.get_logger().info(f'Spawned turtle: {future.result().name}')
        else:
            self.get_logger().error('Failed to spawn turtle')

    def move_turtle2(self):
        # Wait for teleport service to be available
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')
        
        # Create teleport request
        teleport_request = TeleportAbsolute.Request()
        teleport_request.x = 8.0
        teleport_request.y = 2.0
        teleport_request.theta = 0.0
        
        # Call teleport service
        future = self.teleport_client.call_async(teleport_request)
        rclpy.spin_until_future_complete(self, future)
        
        self.get_logger().info('Turtle moved to target position')


def main(args=None):
    rclpy.init(args=args)
    
    node = SpawnTurtle()
    
    # Shutdown after task completion
    rclpy.shutdown()


if __name__ == '__main__':
    main()