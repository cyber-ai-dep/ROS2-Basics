
## Task 


### Objective

Convert the Fairino robot package to work with RViz2 and joint_state_publisher_gui.

**Fairino Robot URDF:**  
https://drive.google.com/drive/folders/1bH5Tv4j57KJIgYRWxjlQ8Mme07TRnTvq?usp=sharing

---

### Requirements

1. Install this Fairino package in src
2. Modify or create launch file to display robot in RViz2
3. Add joint_state_publisher_gui for manual control
4. Ensure all joints are controllable via GUI
5. Configure RViz2 to show robot model and TF

---

### Success Criteria

☐ Fairino robot visible in RViz2  
☐ All joints controllable via GUI  
☐ TF tree shows all frames correctly  
☐ No errors in terminal  
☐ Fixed Frame set to `base_link` or `world`  
☐ RobotModel display added and configured

---

### Expected Behavior

**RViz2:**
- 6DOF robot arm visible
- Multiple colored links
- Smooth movement when adjusting joints

**Joint State Publisher GUI:**
- 6 sliders (one per joint)
- Sliders move robot in real-time
- Joint names displayed

**Terminal:**
- No error messages
- TF updates publishing

---

### Submission

Screenshot showing:

1. **RViz2 window:** Fairino robot in custom pose
2. **Joint State Publisher GUI:** All 6 sliders visible
3. **Terminal:** Launch command with no errors
4. **TF tree PDF:** (from `ros2 run tf2_tools view_frames`)

**Submit to:** Google Classroom

---

### Hints

**Hint 1: Finding URDF**

Look in these directories:
```bash
fairino_description/urdf/
fairino_description/meshes/
```


**Hint 2: Package Name**

Replace `my_robot_description` with `fairino_description` in launch file.