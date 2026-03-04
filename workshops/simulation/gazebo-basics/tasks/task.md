
## Task

### Objective

Install **Gazebo simulator**, launch it successfully, and **add objects inside the environment**.

Students must demonstrate the ability to **run Gazebo and place objects in the simulation world**.

---

## Requirements

1. Install Gazebo simulator
2. Launch Gazebo successfully
3. Load the default world
4. Add at least **two objects** to the environment
5. Run the simulation with physics enabled
6. Submit a **screenshot of the Gazebo scene**

---

## Setup Instructions

### Step 1: Install Gazebo

Install Gazebo for ROS2:

```bash
sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-gz-ros2-control
sudo apt install ros-jazzy-rqt-controller-manager
```

Verify installation:

```bash
gz sim
```

Gazebo should open successfully.

---

### Step 2: Launch Gazebo with Default World

Run Gazebo with an empty world:

```bash
gz sim empty.sdf
```

Expected result:

* Gazebo window opens
* Empty world with ground plane
* Physics engine ready

---

### Step 3: Explore Gazebo Interface

Important panels:

**Left Panel**

* Insert tab → add objects

**Top Toolbar**

* Play / Pause simulation
* Camera controls

**Right Panel**

* Object properties

---

### Step 4: Add Objects

Using the **Insert panel**:

1. Click **Insert**
2. Choose objects such as:

Available objects:

* Box
* Sphere
* Cylinder

3. Click anywhere in the world to place the object.

Add **at least two objects**.

Example:

* One **Box**
* One **Sphere**

---

### Step 5: Enable Physics

Press **Play ▶**

Observe:

* Objects falling due to gravity
* Objects colliding with ground

---

### Step 6: Move Objects

Try:

1. Click object
2. Use **translate tool**
3. Move object above ground
4. Press **Play**

The object should **fall due to gravity**.

---

## Success Criteria

☐ Gazebo installed successfully
☐ Gazebo launches without errors
☐ Default world loads
☐ At least **two objects added**
☐ Physics simulation running
☐ Objects respond to gravity

---

## Expected Behavior

Inside Gazebo:

* Ground plane visible
* Added objects (box, sphere, etc.)
* Realistic lighting and shadows
* Objects fall when simulation starts

---

## Submission

Students must submit **one screenshot** showing:

### Screenshot must include:

1️⃣ **Gazebo window**

* At least **two objects in the scene**
* Simulation running

2️⃣ **Gazebo interface visible**

* Insert panel or toolbar

3️⃣ **Objects placed in different positions**

---

### Example Scene

Example objects:

* Box
* Sphere
* Cylinder

Placed at different locations in the world.

---

### Submit To

Google Classroom

---
