# Service Task: Spawn and Move a New Turtle

## Objective

Write a ROS 2 Python node that:
1. Spawns a new turtle named `turtle2`
2. Moves `turtle2` to a specific position

---

## Services to Use

### 1. `/spawn`

**Type:** `turtlesim/srv/Spawn`

**Request:**
```
float32 x
float32 y
float32 theta
string name
---
string name
```

---

### 2. `/turtle2/teleport_absolute`

**Type:** `turtlesim/srv/TeleportAbsolute`

**Request:**
```
float32 x
float32 y
float32 theta
---
```

---

## Required Behavior

Your node must:

1. Spawn `turtle2` at:
   - x = 5.0
   - y = 5.0
   - theta = 0.0

2. Move `turtle2` to:
   - x = 8.0
   - y = 2.0
   - theta = 0.0

---

## Discover Services

```bash
ros2 service list
ros2 service type /spawn
ros2 interface show turtlesim/srv/Spawn
```

---

## Implementation

### File Location
```
~/ros2_test/src/python_pkg/python_pkg/spawn_turtle.py
```

### Required Imports
```python
from turtlesim.srv import Spawn, TeleportAbsolute
```

---

## Expected Result

- turtle2 appears at (8.0, 2.0)
- Console shows confirmation messages
- turtle2 faces right (theta = 0.0)

---

## Success Criteria

☐ turtle2 spawned successfully  
☐ turtle2 moved to target position  
☐ Proper service client usage  
☐ Confirmation messages printed

---

## Submission

Screenshot showing:
1. Turtlesim with turtle2 at correct position
2. Terminal output
3. `ros2 service list | grep turtle2`

**Deadline:** [DATE]

---

## Common Errors

| Error | Solution |
|-------|----------|
| Service not available | Start turtlesim first |
| Name already exists | Restart turtlesim |
| Turtle doesn't move | Check service name `/turtle2/teleport_absolute` |

---

## Hint

Call spawn service BEFORE teleport service.

---

**Reference:** [Teleport Client Example](../README.md#4-service-example---teleport-client)