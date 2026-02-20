# Action Task: Draw a Square Using ROS 2 Action

## Objective

Create a ROS 2 Python node that draws a square by:
1. Moving forward using **Topics**
2. Rotating 90° using **Actions**
3. Repeating 4 times

---

## Communication Patterns

### 1. Topic: `/turtle1/cmd_vel`

**Type:** `geometry_msgs/msg/Twist`

**Usage:** Move turtle forward
- `linear.x` → forward velocity
- `angular.z` → rotation velocity

---

### 2. Action: `/turtle1/rotate_absolute`

**Type:** `turtlesim/action/RotateAbsolute`

**Usage:** Rotate to exact angle
- Goal: `theta` (radians)
- Feedback: `remaining` (radians)
- Result: `delta` (radians)

---

## Required Behavior

For each of 4 sides:
1. Move forward (2 seconds at speed 1.0)
2. Stop
3. Rotate 90° (1.57 radians) using action
4. Wait for rotation to complete

---

## Implementation

### File Location
```
~/ros2_test/src/python_pkg/python_pkg/draw_square.py
```

### Required Imports
```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from turtlesim.action import RotateAbsolute
import time
```

### Node Structure
```python
class DrawSquare(Node):
    def __init__(self):
        # Create publisher for velocity
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Create action client for rotation
        self.action_client = ActionClient(self, RotateAbsolute, '/turtle1/rotate_absolute')
    
    def move_forward(self, duration=2.0):
        # Publish velocity then stop
        pass
    
    def rotate_90_degrees(self, target_angle):
        # Send action goal with feedback
        pass
    
    def draw_square(self):
        # Loop 4 times: move + rotate
        pass
```

## Expected Result

- Turtle draws approximate square
- 4 sides completed
- Feedback shown during each rotation
- Console logs each step

---

## Success Criteria

☐ Publisher created for velocity  
☐ Action client created for rotation  
☐ Rotates exactly 1.57 radians per turn  
☐ Completes all 4 sides  
☐ Feedback displayed during rotation

---

## Submission

Screenshot showing:
1. Turtlesim with completed square
2. Terminal output (all 4 sides)
3. Action client code snippet

**Deadline:** [DATE]

---

## Hints

### Moving Forward
```python
msg = Twist()
msg.linear.x = 1.0  # Forward
# Publish for duration
msg.linear.x = 0.0  # Stop
```

### Rotation Angle
```python
current_angle = 0.0
for side in range(4):
    current_angle += 1.57  # Add 90 degrees
    self.rotate_90_degrees(current_angle)
```

---

## Common Errors

| Error | Solution |
|-------|----------|
| Square not closing | Use exactly 1.57 radians |
| Turtle moves during rotation | Stop velocity before rotating |
| No feedback | Implement feedback_callback |
| Action not working | Start turtlesim first |

---

## Learning Outcomes

- When to use **Topics** (continuous control)
- When to use **Actions** (precise tasks)
- Combining multiple communication patterns
- Motion sequencing

---

## Comparison

| Aspect | Topic | Action |
|--------|-------|--------|
| Control | Continuous | Precise |
| Feedback | No | Yes |
| Completion | Time-based | Goal-based |

---

**Reference:** [Action Client Example](../README.md#6-action-client)