
## Task

### Objective

Configure the Fairino robot **URDF** to work with **MoveIt2** for motion planning and execution using the MoveIt2 Setup Assistant.

**Fairino Robot URDF:**
[https://drive.google.com/drive/folders/1bH5Tv4j57KJIgYRWxjlQ8Mme07TRnTvq?usp=sharing](https://drive.google.com/drive/folders/1bH5Tv4j57KJIgYRWxjlQ8Mme07TRnTvq?usp=sharing)

---

### Requirements

1. Import the Fairino **URDF** into MoveIt2 Setup Assistant
2. Generate the MoveIt2 configuration package `fairino5_v6_moveit2_config`
3. Create planning group `fairino5_v6_group` including all robot joints and `fairino5_controller` controller 
4. Configure kinematics, controllers, and planning pipelines
5. Launch the robot using `demo.launch.py` in RViz2
6. Plan and execute at least **3 different poses**

---

### Success Criteria

☐ MoveIt2 Setup Assistant completed successfully
☐ `demo.launch.py` runs without errors
☐ Robot visible in RViz2 MoveIt interface
☐ Successfully plan and execute **3 different poses**
☐ Robot moves smoothly without collisions
☐ Planning group `fairino5_v6_group` configured correctly

---

### Expected Behavior

**RViz2:**

* Fairino 6DOF robot visible
* Motion Planning panel available
* Interactive marker for pose goal
* Robot moves when executing planned trajectories

**MoveIt Motion Planning:**

* User can set target poses
* Motion plan generated successfully
* Robot executes trajectory smoothly
* Collision checking active

**Terminal:**

* MoveIt2 nodes launch successfully
* Planning pipeline initialized
* No critical error messages

---

### Submission

Screenshot showing:

1. **RViz2 window:** MoveIt Motion Planning interface with robot model
2. **Planned trajectory:** Robot in a different pose
3. **Terminal:** Launch command running successfully

Launch command used:

```bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

**Submit to:** Google Classroom

---
