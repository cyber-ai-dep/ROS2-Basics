# ROS 2 Nodes and Topics (Python)

##  BEFORE THE WORKSHOP

### Prerequisites Check
You should have already completed:
-  Linux Basics Workshop
-  Python for Robotics Workshop

### System Requirements
-  Ubuntu 24.04 installed and working
-  ROS 2 Jazzy installed
-  Python 3 working
-  Comfortable with terminal usage

### Verify ROS 2 Installation
```bash
ros2 --version
```

Expected output:
```
ros2 cli version X.X.X
```

If `ros2` command is not found, fix this before proceeding.

### First Time Setup Checklist
Before starting the workshop, verify:

☐ ROS 2 environment sourced in `~/.bashrc`:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

☐ colcon build tool installed:
```bash
sudo apt install python3-colcon-common-extensions
```

☐ Python can import ROS libraries:
```bash
python3 -c "import rclpy"
# Should complete without error
```

If any check fails, fix it before proceeding.

---


## Table of Contents
- [Session Goal](#session-goal)
- [1. Concepts: Nodes and Topics](#1-concepts-nodes-and-topics)
- [2. Create ROS 2 Workspace](#2-create-ros-2-workspace)
- [3. Python Package vs ROS 2 Package](#3-python-package-vs-ros-2-package)
- [4. Create ROS 2 Python Package](#4-create-ros-2-python-package)
- [5. Publisher Node (Talker)](#5-publisher-node-talker)
- [6. Subscriber Node (Listener)](#6-subscriber-node-listener)
- [7. Configure setup.py (CRITICAL)](#7-configure-setuppy-critical)
- [8. ROS 2 CLI Tools](#8-ros-2-cli-tools)
- [9. Hands-on Practice](#9-hands-on-practice)
- [10. Task (Mandatory)](#10-task-mandatory)
- [Common Errors and Solutions](#common-errors-and-solutions)
- [Troubleshooting Checklist](#troubleshooting-checklist)
- [Quick Reference](#quick-reference)
- [Workshop Workflow Summary](#workshop-workflow-summary)
- [Why This Matters for ROS 2](#why-this-matters-for-ros-2)
- [Resources](#resources)

  

## Session Goal
Enable students to:
- Understand ROS 2 nodes and topics
- Create a ROS 2 Python package correctly
- Write Publisher and Subscriber nodes
- Run nodes using `ros2 run`
- Inspect communication using ROS 2 CLI tools

---
# 1. Concepts: Nodes and Topics

## Core Concepts

### Node
- A **node** is a single running program in ROS 2.
- Each node performs **one specific task**.
- Multiple nodes work together to build a robot system.

### Topic
- A **topic** is a named communication channel.
- Nodes exchange data using a **publish / subscribe** model.
- Nodes do not communicate directly with each other.

### Publisher
- A **publisher** is a node that sends data to a topic.

### Subscriber
- A **subscriber** is a node that receives data from a topic.

---

## Message Flow

- Data flows in **one direction**
- One topic can have **multiple publishers and subscribers**
- Publisher and subscriber must use the **same message type**

---

## Key Idea

ROS 2 systems are built by connecting **independent nodes** through **topics** to exchange data.


![Diagram](https://github.com/cyber-ai-dep/ROS2-Basics/blob/main/assets/images/ros2-core/Topic-SinglePublisherandSingleSubscriber.gif)

![Diagram](https://github.com/cyber-ai-dep/ROS2-Basics/blob/main/assets/images/ros2-core/Topic-MultiplePublisherandMultipleSubscriber.gif)
```

### Demo
Run them from terminal:

**Terminal 1:**
```bash
ros2 run demo_nodes_py talker
```

**Terminal 2:**
```bash
ros2 run demo_nodes_py listener
```

**Terminal 3:**
```bash
rqt_graph
```

![Diagram](https://github.com/cyber-ai-dep/ROS2-Basics/blob/main/assets/images/ros2-core/qrt_graph.png)


---

## 2. Create ROS 2 Workspace

### Commands
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
```

Expected output:
```

Summary: 0 packages finished [0.19s]
```

### Source the Workspace
```bash
source install/setup.bash
```

⚠️ **You must repeat build + source every time you change code.**

### Explanation

**What is a workspace?**
- A directory structure that holds ROS 2 packages.

**Why these folders?**
- `src/` - Source code
- `build/` - Build files
- `install/` - Installed executables
- `log/` - Build logs

**Why source?**
- Tells ROS 2 where to find your packages.

---

## 3. Python Package vs ROS 2 Package

![Diagram](https://github.com/cyber-ai-dep/ROS2-Basics/blob/main/assets/images/ros2-core/pythonVSrosPackages.png)


### Regular Python Package
- Used only by Python
- Run with `python3 file.py`
- No ROS integration

### ROS 2 Python Package
- Managed by ROS 2 tooling
- Uses `rclpy` library
- Executed via `ros2 run`
- Requires:
  - `package.xml`
  - `setup.py`
  - Entry points configuration

### Key Difference
ROS 2 packages are discoverable and runnable by ROS tooling.

This means:
- Other nodes can find them
- `ros2 run` can execute them
- ROS 2 can manage their dependencies

---

## 4. Create ROS 2 Python Package

### Command
```bash
cd ~/ros2_ws/src
ros2 pkg create my_py_pkg \
  --build-type ament_python \
  --dependencies rclpy std_msgs
```

### Explanation

**`--build-type ament_python`**
- Creates a Python package (not C++)

**`--dependencies rclpy std_msgs`**
- `rclpy` - ROS 2 Python library
- `std_msgs` - Standard message types (String, Int32, etc.)

### Expected Structure
```
my_py_pkg/
├── my_py_pkg/
│   ├── __init__.py
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
└── test/
```

---

## 5. Publisher Node (Talker)

### Concept
A Publisher (Talker):
- Sends messages continuously
- Runs independently
- Uses a timer to publish at regular intervals

### Create the File
```bash
cd ~/ros2_ws/src/my_py_pkg/my_py_pkg
touch talker.py
nano talker.py
```

### Talker Code
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.get_logger().info('Talker node started')

    def publish_message(self):
        msg = String()
        msg.data = 'Hello ROS 2'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main():
    rclpy.init()
    node = Talker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Explanation
Line by line:
```python
super().__init__('talker')  # Node name

self.create_publisher(String, 'chatter', 10)
# Message type, Topic name, Queue size

self.create_timer(1.0, self.publish_message)
# Publish every 1 second

rclpy.spin(node)
# Keep node running
```

⚠️ **Important Note:**
The code is written, but it won't run yet. We need to configure `setup.py` first (Section 7).

---

## 6. Subscriber Node (Listener)

### Concept
A Subscriber (Listener):
- Listens to a topic
- Callback function triggered when message arrives
- Processes incoming data

### Create the File
```bash
cd ~/ros2_ws/src/my_py_pkg/my_py_pkg
touch listener.py
nano listener.py
```

### Listener Code
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.callback,
            10)
        self.get_logger().info('Listener node started')

    def callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')

def main():
    rclpy.init()
    node = Listener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Explanation
```python
self.create_subscription(String, 'chatter', self.callback, 10)
# Message type, Topic name, Callback function, Queue size

def callback(self, msg):
    # This runs every time a message arrives
```

---

## 7. Configure setup.py (CRITICAL)

⚠️ **Why This Section Is Critical**
Without this configuration, `ros2 run` will NOT find your nodes.
This is the #1 most common beginner mistake.

### Location
```
~/ros2_ws/src/my_py_pkg/setup.py
```

### Open the File
```bash
nano ~/ros2_ws/src/my_py_pkg/setup.py
```

### Add Entry Points
Find the `entry_points` section and modify it:
```python
entry_points={
    'console_scripts': [
        'talker = my_py_pkg.talker:main',
        'listener = my_py_pkg.listener:main',
    ],
},
```

### Explanation
**Format:**
```
'executable_name = package_name.file_name:function_name'
```

**Example:**
```python
'talker = my_py_pkg.talker:main'
```
- `talker` - name used with `ros2 run`
- `my_py_pkg.talker` - Python module path
- `main` - function to call

### Build and Source (Mandatory)
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

⚠️ **You must repeat build + source every time you change code.**

Expected output:
```
Starting >>> my_py_pkg
Finished <<< my_py_pkg [X.Xs]

Summary: 1 package finished
```

### Run the Talker (Publisher)

⚠️ If this command fails, STOP and check the Common Errors section below.

**Terminal 1:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_py_pkg talker
```

Expected output:
```
[INFO] [talker]: Talker node started
[INFO] [talker]: Publishing: Hello ROS 2
[INFO] [talker]: Publishing: Hello ROS 2
[INFO] [talker]: Publishing: Hello ROS 2
...
```

If you see this output, your publisher works! ✅

### Run the Listener (Subscriber)

**Terminal 1 (keep running):**
```bash
ros2 run my_py_pkg talker
```

**Terminal 2 (new terminal):**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_py_pkg listener
```

Expected output in Terminal 2:
```
[INFO] [listener]: Listener node started
[INFO] [listener]: I heard: Hello ROS 2
[INFO] [listener]: I heard: Hello ROS 2
[INFO] [listener]: I heard: Hello ROS 2
...
```

This is the moment everything comes together! 🎉

---

## 8. ROS 2 CLI Tools

### List All Topics
```bash
ros2 topic list
```

Expected output:
```
/chatter
/parameter_events
/rosout
```

### View Messages on a Topic
```bash
ros2 topic echo /chatter
```

Expected output:
```
data: Hello ROS 2
---
data: Hello ROS 2
---
```

Press Ctrl+C to stop.

### Get Topic Information
```bash
ros2 topic info /chatter
```

Expected output:
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

### Check Publishing Rate
```bash
ros2 topic hz /chatter
```

Expected output:
```
average rate: 1.000
  min: 0.999s max: 1.001s std dev: 0.001s window: 10
```

---


## 9. Hands-on Practice

### Goal
Students practice the full workflow:
**Modify → Build → Source → Run → Verify**

### Guided Challenges

#### Challenge 1 
Change the message content:
```python
# In talker.py, modify:
msg.data = 'Hello from my robot!'
```

Then rebuild and run:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run my_py_pkg talker
```

Verify the new message appears.

#### Challenge 2 
Change the publish rate to 2 messages per second:
```python
# In talker.py, modify:
self.timer = self.create_timer(0.5, self.publish_message)  # was 1.0
```

Rebuild, source, and check the rate:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run my_py_pkg talker
```

In another terminal:
```bash
ros2 topic hz /chatter
# Should show ~2.0 Hz
```

#### Challenge 3 
Rename the topic from `chatter` to `robot_status`:
```python
# In talker.py:
self.publisher_ = self.create_publisher(String, 'robot_status', 10)

# In listener.py:
self.subscription = self.create_subscription(
    String,
    'robot_status',  # Changed from 'chatter'
    self.callback,
    10)
```

Rebuild, source, and verify:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 topic list  # Should show /robot_status
```

---

### Real Example
```
Camera Node (Publisher)
    ↓ publishes to
/camera/image topic
    ↓ subscribed by
Vision Node (Subscriber)
```


---

## 10. Task (Mandatory)

### Task Description
Create a ROS 2 Python package that:
- Contains a Publisher node that publishes the status of a virtual robot
- Contains a Subscriber node that listens to the published status
- Uses a custom topic name (not `/chatter`)
- Sends and receives messages using standard ROS 2 message types
- Runs using `ros2 run`

### Success Criteria
-  Publisher runs without errors
- Subscriber receives and displays messages
- Topic is visible using `ros2 topic list`
- Messages are verified using `ros2 topic echo`

### Expected Behavior
**Terminal 1:**
```
[INFO] [publisher]: Publishing: [robot status message]
```

**Terminal 2:**
```
[INFO] [subscriber]: I heard: [robot status message]
```

### Submission
Submit screenshots showing:
- Terminal 1: `ros2 run` command for the Publisher and its output
- Terminal 2: `ros2 run` command for the Subscriber and its output
- Terminal 3: `ros2 topic list` showing the custom topic

Submit to: **Google Classroom**

---

## Common Errors and Solutions

| Error | Cause | Solution |
|-------|-------|----------|
| Package 'my_py_pkg' not found | Not built or sourced | `colcon build && source install/setup.bash` |
| No executable found | Missing entry point in setup.py | Add to console_scripts section |
| ModuleNotFoundError: rclpy | ROS 2 environment not sourced | `source /opt/ros/jazzy/setup.bash` |
| Failed to create node | rclpy.init() not called | Check main() function |
| AttributeError: Node has no attribute 'get_logger' | Wrong class inheritance | Use `super().__init__('node_name')` |

---

## Troubleshooting Checklist

Before asking for help, verify:
- ☐ Built workspace? (`colcon build`)
- ☐ Sourced setup file? (`source install/setup.bash`)
- ☐ Entry points added? (in `setup.py`)
- ☐ File names correct? (`talker.py`, `listener.py`)
- ☐ Both nodes running? (two separate terminals)
- ☐ Same topic name? (in both Talker and Listener)

---

## Quick Reference

| Command | Purpose | Example |
|---------|---------|---------|
| `ros2 run` | Run a node | `ros2 run my_py_pkg talker` |
| `ros2 topic list` | List all topics | `ros2 topic list` |
| `ros2 topic echo` | View messages | `ros2 topic echo /chatter` |
| `ros2 topic info` | Topic details | `ros2 topic info /chatter` |
| `ros2 topic hz` | Check rate | `ros2 topic hz /chatter` |
| `colcon build` | Build workspace | `colcon build` |
| `source` | Load workspace | `source install/setup.bash` |

---

## Workshop Workflow Summary

```
1. Create workspace (mkdir, colcon build)
   ↓
2. Create package (ros2 pkg create)
   ↓
3. Write nodes (talker.py, listener.py)
   ↓
4. Configure setup.py ← CRITICAL!
   ↓
5. Build workspace (colcon build)
   ↓
6. Source (source install/setup.bash)
   ↓
7. Run nodes (ros2 run)
   ↓
8. Verify (ros2 topic echo)
```

**Remember: Every time you change code:**
**Build → Source → Run**

Forget to source? → Nodes won't be found! ⚠️

---

## Why This Matters for ROS 2

| Concept | Real Robot Usage | Example |
|---------|------------------|---------|
| Nodes | Independent processes | Camera node, motor controller, planner node |
| Topics | Asynchronous communication | Sensor data streams, robot state updates |
| Talker (Publisher) | Broadcast data | Lidar publishes scan data, GPS publishes position |
| Listener (Subscriber) | Receive and react | Navigation subscribes to scans, controller subscribes to commands |

### Real-World Scenario
```
Camera Node (Talker/Publisher)
    ↓ publishes to /camera/image
Vision Node (Listener/Subscriber + Talker/Publisher)
    ↓ publishes to /detected_objects
Navigation Node (Listener/Subscriber)
    ↓ plans path based on objects
    
```

This is exactly how autonomous robots work!

---

## Resources

### Official Documentation
- [ROS 2 Beginner Tutorials](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html)
- [ROS 2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
- [rclpy API Documentation](https://docs.ros2.org/latest/api/rclpy/)

### Course Materials
- Linux Basics Workshop (Week 1)
- Python for Robotics Workshop (Week 1)
- ROS 2 Cheat Sheet (PDF)
- Example Code Repository

### Practice Platforms
- [ROS 2 Documentation](https://docs.ros.org)

### Getting Help
- Class discussion channel
- Instructor contact

---





