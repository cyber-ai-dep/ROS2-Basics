# Fairino MoveIt2 Color Sorting Robot

This project configures the **Fairino 6-DOF robot** with **MoveIt2** and implements an automated **color-based pick-and-place system** using **OpenCV and ROS2**.

The robot detects object colors (**Red, Blue, Green**) using a camera and sorts them into different positions using **MoveIt2 motion planning**.

---

# Project Resources

### Base Package

Download the base package from:

[Fairino Robot URDF](https://drive.google.com/drive/folders/1LsxnjiY_AJlfwkIP07AFtC-z00xRFfjb?usp=sharing)


---

# Task 1 — MoveIt2 Configuration

## Objective

Configure the **Fairino robot URDF** to work with **MoveIt2** using the **MoveIt2 Setup Assistant**.

The goal is to generate a full **MoveIt2 configuration package** that allows motion planning and execution in **RViz2**.

---

## Requirements

1. Import the **Fairino URDF** into **MoveIt2 Setup Assistant**
2. Generate the MoveIt2 configuration package:

```
fairino5_v6_moveit2_config
```

3. Create the planning group:

```
fairino5_v6_group
```

Include:

* All robot joints
* `fairino5_controller`

4. Configure:

* Kinematics solver
* Controllers
* Planning pipelines

5. Launch the robot using RViz2

```
demo.launch.py
```

6. Plan and execute **at least 3 different poses**

---

## Success Criteria

* MoveIt2 Setup Assistant completed successfully
* `demo.launch.py` runs without errors
* Robot visible in RViz2 MoveIt interface
* Successfully plan and execute **3 different poses**
* Robot moves smoothly without collisions
* Planning group `fairino5_v6_group` configured correctly

---

## Expected Behavior

### RViz2

The following should be visible:

* Fairino **6-DOF robot**
* **Motion Planning panel**
* **Interactive marker** for pose goals
* Robot moves when executing planned trajectories

### MoveIt Motion Planning

The system should allow:

* Setting target poses
* Generating motion plans
* Executing trajectories
* Collision checking

### Terminal

Expected behavior:

* MoveIt2 nodes launch successfully
* Planning pipeline initialized
* No critical error messages

---

## Launch Command

```bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

---

# Task 2 — Color-Based Pick and Place System

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
'prepickpos':  [20.0, -118.0, -62.0, -90.0, 90.0, 1.0]
'pickpos':     [19.0, -120.0, -75.0, -74.0, 90.0, 1.0]
'postpickpos': [20.0, -118.0, -62.0, -90.0, 90.0, 1.0]
```

---

## Step 3 — Place Operation

After picking the object, the robot moves to the **color-specific placement location**.

---

### Red Position

```python
'rprepos':  [-22.0, -100.0, -85.0, -79.0, 94.0, -40.0]
'rpos':     [-21.0, -105.0, -95.0, -65.0, 94.0, -39.0]
'rpostpos': [-22.0, -100.0, -85.0, -79.0, 94.0, -40.0]
```

---

### Blue Position

```python
'bprepos':  [0.0, -90.0, -90.0, 0.0, 90.0, 0.0]
'bpos':     [0.0, -90.0, -90.0, 0.0, 90.0, 0.0]
'bpostpos': [0.0, -90.0, -90.0, 0.0, 90.0, 0.0]
```

---

### Green Position

```python
'gprepos':  [0.0, -90.0, -90.0, 0.0, 90.0, 0.0]
'gpos':     [0.0, -90.0, -90.0, 0.0, 90.0, 0.0]
'gpostpos': [0.0, -90.0, -90.0, 0.0, 90.0, 0.0]
```

---

# Robot Positions

```python
POSITIONS = {

'home_pos': [19.0, -105.0, -84.0, -27.0, 88.0, 0.0],

# Shared pick positions
'prepickpos': [20.0, -118.0, -62.0, -90.0, 90.0, 1.0],
'pickpos': [19.0, -120.0, -75.0, -74.0, 90.0, 1.0],
'postpickpos': [20.0, -118.0, -62.0, -90.0, 90.0, 1.0],

# Red positions
'rprepos': [-22.0, -100.0, -85.0, -79.0, 94.0, -40.0],
'rpos': [-21.0, -105.0, -95.0, -65.0, 94.0, -39.0],
'rpostpos': [-22.0, -100.0, -85.0, -79.0, 94.0, -40.0],

# Blue positions
'bprepos': [0.0, -90.0, -90.0, 0.0, 90.0, 0.0],
'bpos': [0.0, -90.0, -90.0, 0.0, 90.0, 0.0],
'bpostpos': [0.0, -90.0, -90.0, 0.0, 90.0, 0.0],

# Green positions
'gprepos': [0.0, -90.0, -90.0, 0.0, 90.0, 0.0],
'gpos': [0.0, -90.0, -90.0, 0.0, 90.0, 0.0],
'gpostpos': [0.0, -90.0, -90.0, 0.0, 90.0, 0.0],

}
```

---

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
* It **closes at `pickpos`**
* It **remains closed during transport**
* It **opens at the placement position** (`rpos`, `bpos`, or `gpos`)

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
