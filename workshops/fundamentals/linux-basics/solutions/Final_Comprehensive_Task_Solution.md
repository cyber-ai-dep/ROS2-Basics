# Final Comprehensive Task — Solution

## Objective

Create a directory structure, populate it with files, and document the setup.

---

## Requirements

1. Create the following directory structure in your home directory:

```
ros2_workspace/
├── src/
├── logs/
└── config/
```

2. Create a file named `README.md` in `ros2_workspace/` with the following content:

```
# ROS 2 Workspace
This workspace is prepared for ROS 2 development.
- Ubuntu 24.04 LTS
- ROS 2 Jazzy Jalisco
```

3. Create a file named `setup_notes.txt` in the `config/` directory with the text `"Configuration files will be placed here."`

4. Use the `tree` command to display the directory structure of `ros2_workspace/`

5. Copy the output into a file named `workspace_structure.txt` in your home directory.

---

## Submission

Submit the following:

- The output of `tree ~/ros2_workspace`
- The contents of `~/ros2_workspace/README.md`
- The contents of `~/ros2_workspace/config/setup_notes.txt`
- The picture of `tree ~/ros2_workspace` on terminal 

---

## Solution

```bash
cd ~

mkdir -p ros2_workspace/src
mkdir -p ros2_workspace/logs
mkdir -p ros2_workspace/config

echo "# ROS 2 Workspace" > ros2_workspace/README.md
echo "" >> ros2_workspace/README.md
echo "This workspace is prepared for ROS 2 development." >> ros2_workspace/README.md
echo "" >> ros2_workspace/README.md
echo "- Ubuntu 24.04 LTS" >> ros2_workspace/README.md
echo "- ROS 2 Jazzy Jalisco" >> ros2_workspace/README.md

echo "Configuration files will be placed here." > ros2_workspace/config/setup_notes.txt

tree ros2_workspace
tree ros2_workspace > ~/workspace_structure.txt
```
