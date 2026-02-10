# Linux Basics for ROS 2 Development

## What This Course Is

This is a practical technical guide for acquiring the Linux terminal skills required for ROS 2 robotics development. The focus is on command-line navigation, file operations, system administration, and environment setup on Ubuntu 24.04 LTS.

This course prepares beginners with no Linux experience to operate confidently in a terminal environment. It covers only what is immediately necessary for ROS 2 work. Advanced topics and ROS execution are covered in subsequent sessions.

By completing this guide, you will be able to navigate the file system, manage files and directories, understand permissions, install software, and configure development tools.

---

## Prerequisites

### Already Completed

- **Ubuntu 24.04 LTS** — Installed via Virtual Machine OR Dual Boot configuration
- **ROS 2 Jazzy Jalisco** — Installed and present on the system
- **Working network connection** — Required for package installation

### Required for This Session

- System is running (VM or Dual Boot)
- Terminal access is available
- Access to administrator privileges via `sudo`

---

## Section 0: Introduction and Verification

### System Verification

Before proceeding, verify that the system is operational.

**Boot the system:**

1. Start Ubuntu 24.04 LTS (via VirtualBox or native boot)
2. Log in with your credentials
3. Open a terminal (`Ctrl + Alt + T` or search "Terminal" in applications)

**Expected terminal prompt:**

```
username@hostname:~$
```

The system is ready when the terminal prompt appears.

**No further verification is required at this stage.**

---

## VS Code and ROS 2 Reference

### What is VS Code?

Visual Studio Code (VS Code) is a source code editor developed by Microsoft. It supports multiple programming languages and provides features such as syntax highlighting, code completion, debugging, and integrated terminal access.

VS Code is not a ROS 2 tool. It is a general-purpose code editor. It is installed now as preparation for future ROS 2 development work, but it will not be used for ROS operations in this session.

### Why Install VS Code Now?

VS Code will be the primary development environment for writing ROS 2 nodes, launch files, and configuration files in later sessions. Installing it now ensures the environment is ready when ROS 2 programming begins.

### Installing VS Code

**Download the official package:**

1. Open a web browser
2. Navigate to: [https://code.visualstudio.com/](https://code.visualstudio.com/)
3. Download the `.deb` package for Linux

**Install via terminal:**

```bash
cd ~/Downloads
sudo apt install ./code_*.deb -y
```

**Verify installation:**

```bash
code --version
```

**Expected output:**

```
1.x.x
<commit-hash>
<architecture>
```

**Launch methods:**

- From terminal: `code`
- From application menu: Search "Visual Studio Code"

### VS Code Extension: LaunchMap

**Install the LaunchMap extension:**

1. Open VS Code
2. Press `Ctrl + Shift + X` to open the Extensions panel
3. Search for "LaunchMap"
4. Click Install

**Or install via terminal:**

```bash
code --install-extension launchmap.launchmap
```

**Purpose:**

LaunchMap provides visualization and management for ROS 2 launch files. It will be explained and used in later sessions.

**Do NOT configure VS Code further at this stage.**

---

### ROS 2 Reference

ROS 2 Jazzy Jalisco is already installed on this system. Execution and verification of ROS 2 will occur in later sessions.

**Official reference documentation:**

[https://docs.ros.org/en/jazzy/Installation.html](https://docs.ros.org/en/jazzy/Installation.html)

This is a reference link. No action is required now.

---

## Section 1: Linux Overview

### Why Linux is Used in This Course

Linux is the primary operating system for ROS 2 development and deployment. Most production robotics systems run Linux-based distributions.

**Key reasons:**

- **System control** — Direct access to hardware, processes, and system resources
- **Stability** — Long-term support releases provide consistent environments
- **ROS 2 compatibility** — Official ROS 2 releases target Ubuntu LTS versions
- **Command-line access** — Remote robot systems are controlled via terminal, not graphical interfaces

### Ubuntu 24.04 LTS

Ubuntu is a Linux distribution maintained by Canonical. The "LTS" designation stands for Long Term Support, meaning this version receives security updates and support for five years.

ROS 2 Jazzy Jalisco officially targets Ubuntu 24.04 LTS. This pairing ensures compatibility between the operating system and the robotics framework.

### The Terminal as a Control Tool

The terminal is a text-based interface to the operating system. Commands are typed and executed, and the system responds with output or performs actions.

In robotics:

- Robots are often accessed remotely via SSH (Secure Shell), which provides terminal access only
- ROS 2 nodes, tools, and debugging utilities are launched from the terminal
- Configuration files are edited using terminal-based text editors
- File transfers, log analysis, and system monitoring occur via command-line tools

The terminal is not optional in ROS 2 development. It is the primary interface.

---

## Section 2: Terminal Basics

### Understanding `sudo`

`sudo` (Superuser Do) executes commands with administrator privileges.

**When `sudo` is required:**

- Installing or removing system-wide software
- Modifying system configuration files
- Accessing restricted hardware devices
- Starting or stopping system services

**Syntax:**

```bash
sudo <command>
```

**Example:**

```bash
sudo apt update
```

The system will prompt for the user password. Enter the password to proceed.

**All commands in this guide that require elevated privileges use `sudo` explicitly.**

---

### Navigation: `cd`

`cd` (Change Directory) moves the terminal's working location between directories.

**Basic usage:**

```bash
cd Documents
```

**Go to the parent directory:**

```bash
cd ..
```

**Go to the home directory:**

```bash
cd ~
```

**Practice sequence:**

```bash
cd Documents
cd ..
cd ~
```

---

### TAB Completion

Pressing the `Tab` key twice displays all available completions for the current input. This prevents typing errors and speeds up navigation.

**Example:**

```bash
cd Doc<TAB><TAB>
```

**Expected behavior:**

If a directory starting with "Doc" exists (e.g., `Documents`), the shell will display it. Pressing `Tab` once more completes the name.

**Why this matters:**

File and directory names in Linux are case-sensitive. TAB completion ensures correct capitalization and reduces command errors.

---

### Listing Contents: `ls`

`ls` (List) displays files and directories in the current location.

**Basic usage:**

```bash
ls
```

**Show detailed information:**

```bash
ls -l
```

**Show all files (including hidden):**

```bash
ls -la
```

**Practice:**

```bash
ls
ls -l
ls -la
```

Hidden files in Linux start with a dot (`.`). The `-a` flag reveals them.

---

### Print Working Directory: `pwd`

`pwd` (Print Working Directory) displays the absolute path of the current location.

**Usage:**

```bash
pwd
```

**Expected output:**

```
/home/username
```

This confirms the current location in the file system.

---

### File System Structure

Linux organizes files in a hierarchical tree structure. The top level is called the root directory and is represented by `/`.

**Key directories:**

```
/                   # Root directory (top level)
├── home/           # User home directories
│   └── username/   # Your personal files
├── usr/            # User programs and utilities
│   └── bin/        # Executable programs
└── etc/            # System configuration files
```

**The `~` shortcut:**

`~` represents the current user's home directory. For a user named `student`, `~` is equivalent to `/home/student`.

**Example:**

```bash
cd /home/username
```

is the same as:

```bash
cd ~
```

---

### Central Practical Example

**Objective:** Navigate to the home directory, list all contents including hidden files, create a new directory, move into it, verify the location, and return to the home directory.

**Commands:**

```bash
cd ~
pwd
ls -la
cd Documents
pwd
cd ..
pwd
```

**Expected sequence:**

1. `cd ~` → moves to `/home/username`
2. `pwd` → displays `/home/username`
3. `ls -la` → lists all files and directories, including hidden ones
4. `cd Documents` → moves to `/home/username/Documents`
5. `pwd` → displays `/home/username/Documents`
6. `cd ..` → moves to `/home/username`
7. `pwd` → displays `/home/username`

This sequence demonstrates basic navigation and location awareness.

---

## File Operations

### Creating Directories: `mkdir`

`mkdir` (Make Directory) creates new directories.

**Basic usage:**

```bash
mkdir my_folder
```

**Create nested directories:**

```bash
mkdir -p parent/child/grandchild
```

The `-p` flag creates parent directories as needed.

**Practice:**

```bash
cd ~
mkdir linux_practice
cd linux_practice
pwd
```

---

### Creating Files: `touch`

`touch` creates an empty file.

**Usage:**

```bash
touch file.txt
```

**Create multiple files:**

```bash
touch file1.txt file2.txt file3.txt
```

**Practice:**

```bash
cd ~/linux_practice
touch notes.txt
ls
```

---

### Writing to Files: `echo`

`echo` prints text to the terminal or writes it to a file.

**Print to terminal:**

```bash
echo "Hello"
```

**Write to a file (overwrite):**

```bash
echo "This is a note" > notes.txt
```

**Practice:**

```bash
cd ~/linux_practice
echo "Linux is used for ROS 2 development" > notes.txt
```

**Do NOT use `>>` (append) in this session.**

---

### Displaying File Contents: `cat`

`cat` (Concatenate) displays the contents of one or more files.

**Usage:**

```bash
cat notes.txt
```

**Display multiple files:**

```bash
cat file1.txt file2.txt
```

**Practice:**

```bash
cd ~/linux_practice
cat notes.txt
```

**Do NOT use `cat > combined.txt` or similar redirection in this session.**

---

### Editing Files: `nano`

`nano` is a simple terminal-based text editor.

**Open a file:**

```bash
nano notes.txt
```

**Basic controls:**

- Type normally to edit
- `Ctrl + O` → Save
- `Enter` → Confirm save
- `Ctrl + X` → Exit

**Practice:**

```bash
cd ~/linux_practice
nano notes.txt
```

Add a line, save, and exit.

---

### Copying Files: `cp`

`cp` (Copy) duplicates files or directories.

**Copy a file:**

```bash
cp source.txt destination.txt
```

**Copy to a directory:**

```bash
cp file.txt /path/to/directory/
```

**Copy a directory (recursive):**

```bash
cp -r folder/ new_folder/
```

**Practice:**

```bash
cd ~/linux_practice
cp notes.txt notes_backup.txt
ls
```

---

### Moving and Renaming: `mv`

`mv` (Move) moves or renames files and directories.

**Rename a file:**

```bash
mv old_name.txt new_name.txt
```

**Move a file:**

```bash
mv file.txt /path/to/directory/
```

**Practice:**

```bash
cd ~/linux_practice
mv notes_backup.txt backup_notes.txt
ls
```

---

### Deleting Files: `rm`

`rm` (Remove) deletes files and directories permanently. There is no recycle bin or undo.

**Delete a file:**

```bash
rm file.txt
```

**Practice:**

```bash
cd ~/linux_practice
touch temp.txt
ls
rm temp.txt
ls
```

**Do NOT use `rm -rf` in this session.**

---

### Absolute vs. Relative Paths

**Absolute path:** Starts from the root directory (`/`).

```bash
cd /home/username/linux_practice
```

**Relative path:** Relative to the current directory.

```bash
cd linux_practice
cd ../
```

**`.` represents the current directory.**
**`..` represents the parent directory.**

---

### Wildcards

Wildcards match multiple files based on patterns.

| Pattern | Matches |
|---------|---------|
| `*` | Any characters |
| `?` | Single character |

**Examples:**

```bash
ls *.txt
```

Matches all files ending in `.txt`.

```bash
ls file?.txt
```

Matches `file1.txt`, `filea.txt`, etc.

**Why wildcards matter:**

They allow batch operations without listing every file individually.

---

## Permissions (Concept Only)

### What Are Permissions?

Every file and directory in Linux has three types of permissions:

- **Read (r)** — View contents
- **Write (w)** — Modify contents
- **Execute (x)** — Run as a program

Permissions are assigned to three categories:

- **Owner** — The user who created the file
- **Group** — Users in the same group
- **Others** — All other users

### Viewing Permissions

```bash
ls -l
```

**Example output:**

```
-rw-r--r-- 1 username username 1234 Jan 15 10:30 file.txt
```

**Breaking it down:**

- `-rw-r--r--` — Permissions (owner: read/write, group: read, others: read)
- `username username` — Owner and group
- `file.txt` — File name

### Why Permissions Matter in Robotics

- **Hardware access** — Some devices require specific permissions
- **Script execution** — Executable scripts need the execute (`x`) permission
- **Security** — Prevents unauthorized access to robot control systems

**Numeric notation and `chmod` usage are NOT covered in this session.**

---

## Package Management with APT

### What is APT?

APT (Advanced Package Tool) is the package manager for Ubuntu. It installs, updates, and removes software.

### Essential APT Commands

**Update package lists:**

```bash
sudo apt update
```

This downloads the latest package information from repositories. It does not install or upgrade anything.

**Upgrade installed packages:**

```bash
sudo apt upgrade
```

This installs newer versions of currently installed packages.

**Install a package:**

```bash
sudo apt install <package-name>
```

**Remove a package:**

```bash
sudo apt remove <package-name>
```

**Remove unused dependencies:**

```bash
sudo apt autoremove
```

### Practice: Installing Essential Tools

**Update the system:**

```bash
sudo apt update
```

**Install common development tools:**

```bash
sudo apt install -y git curl wget
```

**Verify installation:**

```bash
git --version
curl --version
wget --version
```

**Expected output:**

Version numbers for each tool.

---

## ROS 2 Package Installation (No Explanation)

The following commands install ROS 2 packages required for later sessions. Install them now. They will be explained and used in future sessions.

### Controllers and ros2_control

```bash
sudo apt install \
ros-jazzy-ros2-control \
ros-jazzy-ros2-controllers \
ros-jazzy-controller-manager -y
```

```bash
sudo apt install ros-jazzy-ros2-control-cmake -y
```

```bash
sudo apt install ros-jazzy-rqt ros-jazzy-rqt-gui ros-jazzy-rqt-common-plugins -y
```

```bash
sudo apt install ros-jazzy-rqt-controller-manager -y
```

### MoveIt 2

```bash
sudo apt install ros-jazzy-moveit -y
```

**No verification is required. These packages will be used in later sessions.**

---

## Task Structure

This course uses three types of tasks:

### Type 1: Demonstration Task

These tasks are demonstrated fully. You observe but do not execute them during the demonstration.

### Type 2: Practice Task

These tasks are performed by you. They use only commands that have been explained in previous sections.

### Type 3: Final Comprehensive Task

This task combines all skills covered in the session. It is submitted for evaluation.

---

## Practice Task 1: Terminal Navigation

**Objective:** Navigate the file system, list contents, and verify your location.

**Instructions:**

1. Open a terminal
2. Navigate to your home directory
3. List all files, including hidden ones
4. Create a directory named `ros2_practice`
5. Navigate into `ros2_practice`
6. Verify your current location
7. Navigate back to your home directory

**Commands:**

```bash
cd ~
ls -la
mkdir ros2_practice
cd ros2_practice
pwd
cd ~
```

---

## Practice Task 2: File Operations

**Objective:** Create files, write content, copy, move, and delete files.

**Instructions:**

1. Navigate to `~/ros2_practice`
2. Create a file named `system_info.txt`
3. Write "Ubuntu 24.04 LTS" into the file
4. Display the file contents
5. Copy the file to `system_info_backup.txt`
6. Rename `system_info_backup.txt` to `backup.txt`
7. Delete `backup.txt`
8. List the directory contents to verify

**Commands:**

```bash
cd ~/ros2_practice
touch system_info.txt
echo "Ubuntu 24.04 LTS" > system_info.txt
cat system_info.txt
cp system_info.txt system_info_backup.txt
mv system_info_backup.txt backup.txt
rm backup.txt
ls
```

---

## Practice Task 3: Package Installation

**Objective:** Install a package and verify installation.

**Instructions:**

1. Update the package lists
2. Install the `tree` package
3. Verify installation by checking the version

**Commands:**

```bash
sudo apt update
sudo apt install -y tree
tree --version
```

**Expected output:**

Version number for `tree`.

---

## Final Comprehensive Task

**Objective:** Create a directory structure, populate it with files, and document the setup.

### Requirements

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

3. Create a file named `setup_notes.txt` in the `config/` directory with the text "Configuration files will be placed here."

4. Use the `tree` command to display the directory structure of `ros2_workspace/`

5. Copy the output into a file named `workspace_structure.txt` in your home directory

### Submission

Submit the following:

- The output of `tree ~/ros2_workspace`
- The contents of `~/ros2_workspace/README.md`
- The contents of `~/ros2_workspace/config/setup_notes.txt`

### Commands

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
```

---

## Resources

### Official Documentation

- **ROS 2 Official Documentation:** [https://docs.ros.org/en/jazzy/](https://docs.ros.org/en/jazzy/)
- **ROS 2 Installation Guide (Ubuntu):** [https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- **VS Code Linux Setup:** [https://code.visualstudio.com/docs/setup/linux](https://code.visualstudio.com/docs/setup/linux)
- **Ubuntu Documentation:** [https://help.ubuntu.com/](https://help.ubuntu.com/)

### Learning Resources

- **Linux Journey:** [https://linuxjourney.com/](https://linuxjourney.com/)
- **The Linux Command Line (free book):** [http://linuxcommand.org/tlcl.php](http://linuxcommand.org/tlcl.php)

### Cheat Sheets

- **Linux Cheat Sheet:** [Download PDF](https://drive.google.com/file/d/1vddIJlermV-xZ7TZ8eSUCvzHKv_Poj2A/view)
- **ROS 2 Cheat Sheet:** [Download PDF](https://drive.google.com/file/d/1vlimFy3CSk26AfZjB73rtG6GKqPZVUT9/view)

### Community

- **ROS Discourse:** [https://discourse.ros.org/](https://discourse.ros.org/)
- **ROS Answers:** [https://answers.ros.org/](https://answers.ros.org/)

---

## Common Issues and Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| Terminal does not open | Keyboard shortcut not configured | Search "Terminal" in application menu |
| `sudo` prompts for password repeatedly | Incorrect password entered | Re-enter the correct user password |
| `apt update` fails | No network connection | Verify network connectivity and retry |
| VS Code does not launch | Installation incomplete | Run `sudo apt install ./code_*.deb -y` again |
| `tree` command not found | Package not installed | Run `sudo apt install -y tree` |
| File not found error | Incorrect path or typo | Verify the file path with `ls` and `pwd` |

---

## Workshop Tips

- Some tasks may be completed outside of the session
- TAB completion prevents typing errors and saves time
- `pwd` confirms your current location before executing commands
- Always verify the output of a command before proceeding

---

## Key Skills

By the end of this session, you should be able to:

- Navigate the file system using `cd`, `ls`, and `pwd`
- Create files and directories using `touch` and `mkdir`
- Organize files using `cp` and `mv`
- Install software using APT
- Use VS Code as a code editor

---

## What's Next

The next session will cover:

- ROS 2 environment sourcing
- ROS 2 concepts (nodes, topics, messages)
- Creating ROS 2 packages
- Writing ROS 2 nodes

Ensure all tasks in this session are completed before proceeding.

---

**End of Session**
