## 10. Task

### Objective

Running the Fairino robot package in RViz2 and Gazebo after creating the necessary changes.

**Fairino Robot URDF:**
[Download from Google Drive](https://drive.google.com/drive/folders/1bH5Tv4j57KJIgYRWxjlQ8Mme07TRnTvq?usp=sharing)

---

### Requirements

1. Install the Fairino package in `src`, then modify:
   - URDF (add camera plugin)
   - `controllers.yaml` (add joint trajectory controller)
2. Create a launch file to display the robot in both RViz2 and Gazebo simultaneously
3. Ensure all joints are controllable via GUI
4. Configure RViz2 to show the robot model, TF, and the Gazebo camera feed

---

### Success Criteria

- [ ] Fairino robot visible in both Gazebo and RViz2
- [ ] Camera feed from Gazebo displayed in RViz2
- [ ] All joints controllable via `rqt_joint_trajectory_controller`
- [ ] TF tree shows all frames correctly
- [ ] No errors in terminal
- [ ] Fixed Frame set to `base_link` or `world`
- [ ] RobotModel display added and configured

---

### Expected Behavior

**Gazebo:**
- 6DOF Fairino robot arm spawned in the world
- Camera sensor visible and publishing
- Joints respond to controller commands

**RViz2:**
- 6DOF robot arm visible with colored links
- Camera feed panel showing Gazebo camera output
- Smooth movement when controlling joints
- TF frames updating in real-time

**rqt Joint Trajectory Controller:**
- 6 sliders (one per joint)
- Sliders move the robot in Gazebo and RViz2 simultaneously
- Joint names match those defined in `controllers.yaml`

**Terminal:**
- No error messages
- `/camera/image` publishing at ~30 Hz
- Controllers spawned in correct order

---

### Submission

Screenshot showing:

1. **Gazebo window:** Fairino robot in the simulation
2. **RViz2 window:** Split view — robot model + camera feed
3. **rqt GUI:** All 6 joint sliders visible
4. **Terminal:** Launch command with no errors
5. **TF tree PDF:** generated with `ros2 run tf2_tools view_frames`

**Submit to:** Google Classroom

---

### Hints

**Hint 1 — Finding the URDF**
```bash
fairino_description/urdf/
fairino_description/meshes/
```

**Hint 2 — Package Name**

Replace `arm_description` with `fairino_description` in your launch file.

**Hint 3 — Controller Order**

Follow the same `RegisterEventHandler` pattern from `sim.launch.py`:
```
spawn_robot → joint_state_broadcaster → arm_controller
```

**Hint 4 — Camera Topic**

After bridging, the camera publishes to `/camera/image`.
In RViz2, set Image Topic to `/camera/image`.