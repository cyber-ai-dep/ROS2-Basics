# Fairino MoveIt2 Color Sorting Robot

This project configures the **Fairino 6-DOF robot** with **MoveIt2** and implements an automated **color-based pick-and-place system** using **OpenCV and ROS2**.

The robot detects object colors (**Red, Blue, Green**) using a camera and sorts them into different positions using **MoveIt2 motion planning**.

---

# Project Resources

### Base Package

Download the base package from:

[Fairino Robot Packages](https://github.com/Cyberai-Department/fairino3v6_package.git)


---
## Clone the Repository

```bash
cd ~/ros2_ws/src

git clone https://github.com/Cyberai-Department/fairino3v6_package.git

cd ~/ros2_ws

colcon build 

source install/setup.bash
```     


## Launch Command

```bash
ros2 launch fairino3_v6_moveit2_config demo.launch.py
```

---

# Task  — Color-Based Pick and Place System

After successfully creating and validating the MoveIt2 configuration package, implement the following **robot logic**.

---

# System Logic

The robot performs a **continuous sorting cycle**.

### Initial State

The robot always starts from:

```
home_pos
```

and returns to this position after completing each cycle.

---

## Step 1 — Color Detection

Use **OpenCV** to detect object colors.

The system must detect and classify objects as:

```
red
blue
green
```

A color must be detected continuously for **3 seconds** before being accepted.

---

## Step 2 — Pick Operation

Once the color is confirmed, the robot moves to the **shared pick positions**.

```python
'prepickpos': [65.0, -71.0, 76.0, -98.0, -91.0, 24.0]
'pickpos': [63.0, -63.0, 86.0, -114.0, -90.0, 20.0]
'postpickpos': [65.0, -71.0, 76.0, -98.0, -91.0, 24.0]
```

---

## Step 3 — Place Operation

After picking the object, the robot moves to the **color-specific placement location**.

---

### Red Position

```python
'rprepos': [118.0, -73.0, 75.0, -92.0, -90.0, -16.0]
'rpos': [120.0, -68.0, 93.0, -115.0, -90.0, -15.0]
'rpostpos': [118.0, -73.0, 75.0, -92.0, -90.0, -16.0]
```

---

### Blue Position

```python
'bprepos': [110.0, -70.0, 75.0, -96.0, -90.0, -28.0]
'bpos': [110.0, -65.0, 88.0, -113.0, -90.0, -28.0]
'bpostpos': [110.0, -70.0, 75.0, -96.0, -90.0, -28.0]
```

---

### Green Position

```python
'gprepos': [130.0, -71.0, 76.0, -94.0, -92.0, -6.0]
'gpos': [132.0, -66.0, 91.0, -114.0, -92.0, -6.0]
'gpostpos': [130.0, -71.0, 76.0, -94.0, -92.0, -6.0]
```

---

# Robot Positions

```python
POSITIONS = {

'home_pos': [108.0, -107.0, 97.0, -80.0, -90.0, -26.0],

# Shared pick positions
'prepickpos': [65.0, -71.0, 76.0, -98.0, -91.0, 24.0],
'pickpos': [63.0, -63.0, 86.0, -114.0, -90.0, 20.0],
'postpickpos': [65.0, -71.0, 76.0, -98.0, -91.0, 24.0],

# Red positions
'rprepos': [118.0, -73.0, 75.0, -92.0, -90.0, -16.0],
'rpos': [120.0, -68.0, 93.0, -115.0, -90.0, -15.0],
'rpostpos': [118.0, -73.0, 75.0, -92.0, -90.0, -16.0],

# Blue positions
'bprepos': [110.0, -70.0, 75.0, -96.0, -90.0, -28.0],
'bpos': [110.0, -65.0, 88.0, -113.0, -90.0, -28.0],
'bpostpos': [110.0, -70.0, 75.0, -96.0, -90.0, -28.0],

# Green positions
'gprepos': [130.0, -71.0, 76.0, -94.0, -92.0, -6.0],
'gpos': [132.0, -66.0, 91.0, -114.0, -92.0, -6.0],
'gpostpos': [130.0, -71.0, 76.0, -94.0, -92.0, -6.0],

}
```
 


# Motion Sequences

Each step represents either:

```
position_name
```

or

```
(position_name, gripper_action)
```

Gripper actions:

```
open
close
```

---

## Red Sequence

```python
[
'home_pos',
'prepickpos',
('pickpos', 'close'),
'postpickpos',
'rprepos',
('rpos', 'open'),
'rpostpos',
'home_pos'
]
```

---

## Blue Sequence

```python
[
'home_pos',
'prepickpos',
('pickpos', 'close'),
'postpickpos',
'bprepos',
('bpos', 'open'),
'bpostpos',
'home_pos'
]
```

---

## Green Sequence

```python
[
'home_pos',
'prepickpos',
('pickpos', 'close'),
'postpickpos',
'gprepos',
('gpos', 'open'),
'gpostpos',
'home_pos'
]
```

---

# Gripper Control

The gripper is controlled using the following class:

```python
from gripper import Gripper
```

### Behavior

* The gripper **starts open**
* It **closes at `pickpos` and delays for 0.5 second** 
* It **remains closed during transport**
* It **opens at the placement position and delays for 0.5 second** (`rpos`, `bpos`, or `gpos`)

---
# Submission

Google Classroom

Screenshots showing:

1. **RViz2 Window:** MoveIt Motion Planning interface with the robot model  
2. **Color Detection:** Camera window showing real-time color detection  
3. **Terminal:** Running the MoveIt package and robot sequence from the terminal  
4. **Video:** Recording the complete workflow execution  a
5. **GitHub (Optional):** Full source code uploaded to GitHub  

---


# References

### Robot Motion Example

[Robot Motion Example Code 3 DOF](https://github.com/cyber-ai-dep/ROS2-Basics/tree/dev/workshops/simulation/moveit2-basics/code/examples/src/demo_moveit/scripts)

[Robot Motion Example Code 6 DOF](https://github.com/cyber-ai-dep/ROS2-Basics/tree/dev/workshops/Course_project/References/6dof_code)


### OpenCV Color Detection

[OpenCV Color Detection Code](https://github.com/cyber-ai-dep/ROS2-Basics/tree/dev/workshops/Course_project/References/color_detection.py)

### Gripper Control Code

[Gripper Control Code](https://github.com/cyber-ai-dep/ROS2-Basics/tree/dev/workshops/Course_project/References/gripper.py)

---

