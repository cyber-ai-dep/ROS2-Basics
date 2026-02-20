import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from turtlesim.action import RotateAbsolute
import time


class DrawSquare(Node):
    def __init__(self):
        super().__init__('draw_square')
        
        # Create publisher for velocity commands
        self.velocity_publisher = self.create_publisher(
            Twist, 
            '/turtle1/cmd_vel', 
            10)
        
        # Create action client for rotation
        self.action_client = ActionClient(
            self,
            RotateAbsolute,
            '/turtle1/rotate_absolute')
        
        # Track current angle for absolute rotation
        self.current_angle = 0.0
        
        # Track which side we're drawing
        self.current_side = 0
        
        self.get_logger().info('Draw Square node started')

    def move_forward(self, duration=2.0):
        """Move turtle forward for specified duration"""
        msg = Twist()
        msg.linear.x = 1.0  # Forward speed
        msg.angular.z = 0.0
        
        # Publish velocity for duration
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.velocity_publisher.publish(msg)
            time.sleep(0.1)
        
        # Stop the turtle
        msg.linear.x = 0.0
        self.velocity_publisher.publish(msg)
        
        self.get_logger().info(f'Moved forward (side {self.current_side + 1}/4)')

    def rotate_90_degrees(self):
        """Rotate turtle 90 degrees using action"""
        # Calculate target angle (add 90 degrees = 1.57 radians)
        self.current_angle += 1.57
        
        # Create goal message
        goal_msg = RotateAbsolute.Goal()
        goal_msg.theta = self.current_angle
        
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        
        self.get_logger().info(f'Rotating to {self.current_angle:.2f} radians')
        
        # Send goal with feedback callback
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        # Add callback when goal is accepted/rejected
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """Called repeatedly during action execution"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Remaining: {feedback.remaining:.2f} radians')

    def goal_response_callback(self, future):
        """Called when goal is accepted or rejected"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        
        # Get result asynchronously
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Called when action completes"""
        result = future.result().result
        self.get_logger().info(f'Rotation complete: {result.delta:.2f} radians')
        
        # Move to next side
        self.current_side += 1
        
        # Continue drawing or finish
        if self.current_side < 4:
            # Draw next side
            self.move_forward()
            self.rotate_90_degrees()
        else:
            # Square complete
            self.get_logger().info('Square drawing complete!')
            rclpy.shutdown()

    def start_drawing(self):
        """Start drawing the square"""
        # Draw first side
        self.move_forward()
        self.rotate_90_degrees()


def main(args=None):
    rclpy.init(args=args)
    
    node = DrawSquare()
    
    # Start drawing the square
    node.start_drawing()
    
    # Keep node running until square is complete
    rclpy.spin(node)


if __name__ == '__main__':
    main()