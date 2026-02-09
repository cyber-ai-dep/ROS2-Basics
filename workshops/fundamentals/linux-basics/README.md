# Linux Basics and ROS 2 Environment Setup

**Workshop Duration:** 120 minutes (2 hours)

**Session Type:** Hands-on workshop with instructor guidance

---

## Learning Objectives

By the end of this session, you will be able to:

1. Navigate the Linux file system confidently using terminal commands
2. Create, move, copy, and delete files and directories
3. Understand and modify file permissions
4. Install and manage software packages using `apt`
5. Configure VS Code for ROS 2 development
6. Verify your ROS 2 installation and environment
7. Execute basic ROS 2 commands and check system status

---

## Prerequisites

✅ **Already Completed:**
- Ubuntu 24.04 LTS installed on VirtualBox
- ROS 2 Jazzy Jalisco installed
- Working Virtual Machine with network access

✅ **Required for this session:**
- Your VM powered on and running
- Basic computer literacy
- Willingness to use the terminal (we'll teach you!)

---

## Why This Workshop Matters for ROS 2

ROS 2 is built on Linux. **Every single robotics professional needs Linux skills.** Here's why:

- **ROS 2 runs primarily on Ubuntu**: Most robots run Linux-based systems
- **Terminal = Your main tool**: You'll launch nodes, check topics, and debug issues via terminal
- **Remote robot access**: Robots don't have screens—you'll SSH in and use command line
- **Package management**: Installing sensors, simulators, and tools requires `apt` and dependencies
- **File permissions**: Robot systems need proper security and access control
- **Development workflow**: VS Code + terminal + ROS 2 = professional robotics development

**Bottom line:** If you can't use Linux confidently, you can't develop for ROS 2 effectively.

---

## Session Timeline

| Section | Duration | Activity |
|---------|----------|----------|
| Introduction and Verification | 10 min | Check VMs, overview session |
| Linux Overview | 5 min | Why Linux? Ubuntu + ROS 2 ecosystem |
| Terminal Basics | 20 min | Navigation, commands, shortcuts |
| File Operations | 25 min | Create, move, copy, delete files |
| Permissions and sudo | 15 min | Understanding ownership and root access |
| Package Management | 15 min | apt commands, installing tools |
| VS Code Setup | 15 min | Install, configure, extensions |
| ROS 2 Verification | 10 min | Test ROS 2, check environment |
| Final Task Explanation | 5 min | Assignment overview |

---

## Section 0: Introduction and Verification (10 minutes)

### Goals
- Ensure all students have working VMs
- Set expectations for the workshop
- Establish the hands-on, experimental learning approach

### Instructor Script

**Welcome!** Today we're building the foundation you'll use in **every single ROS 2 session** going forward. No Linux knowledge? Perfect. That's exactly who this workshop is for.

**Workshop approach:**
- I'll demonstrate, you'll follow along immediately
- Make mistakes—that's how you learn terminal skills
- Ask questions anytime
- By the end, you'll be comfortable in the Linux environment

**Let's verify your setup:**

### Quick VM Check

Everyone, please:

1. **Power on your Ubuntu VM** in VirtualBox
2. **Log in** with your credentials
3. **Open a terminal** (Ctrl + Alt + T or search "Terminal")

You should see something like:

```bash
username@ubuntu:~$
```

**🎯 Instructor Checkpoint:** Walk around (or check screens if virtual). Ensure everyone sees a terminal prompt before continuing.

### What We'll Do Today

```
Linux Terminal Skills → File Management → System Administration → ROS 2 Integration
```

**Key message:** "Every command you learn today will be used in real ROS 2 development."

---

## Section 1: Linux Overview (5 minutes)

### Goals
- Understand why ROS 2 uses Linux
- Learn about Ubuntu and its role in robotics
- Grasp the open-source philosophy

### Instructor Talking Points

**Why Linux for Robotics?**

1. **Open Source**: Free, customizable, transparent code
2. **Stability**: Servers, robots, and critical systems run Linux
3. **Real-time Capabilities**: Linux can be configured for real-time robot control
4. **Community**: Massive robotics ecosystem on Ubuntu
5. **Performance**: Lightweight, efficient, no bloat

**Ubuntu Specifically:**

- **Ubuntu 24.04 LTS** = Long Term Support (5 years of updates)
- **ROS 2 Jazzy** officially targets Ubuntu 24.04
- Most ROS 2 documentation assumes Ubuntu

**Your Terminal = Your Superpower:**

In robotics:
- You'll SSH into robots (no GUI, terminal only)
- You'll launch ROS 2 nodes via command line
- You'll debug sensor data in real-time using terminal tools
- You'll write scripts that control physical hardware

**Think of the terminal as your robot's remote control.**

---

## Section 2: Terminal Basics (20 minutes)

### Goals
- Master basic navigation commands
- Understand the Linux file system structure
- Use command syntax effectively
- Learn keyboard shortcuts

### Understanding the Terminal Prompt

When you open terminal, you see:

```bash
username@hostname:~$
```

Breaking it down:
- `username`: Your login name
- `hostname`: Computer name (e.g., "ubuntu")
- `~`: Current directory (~ means home directory)
- `$`: Regular user prompt (# means root user)

### Essential Navigation Commands

#### 1. `pwd` - Print Working Directory

**What it does:** Shows your current location

```bash
pwd
```

**Expected output:**
```
/home/username
```

**Why it matters for ROS 2:** You need to know where you are when building packages or sourcing workspaces.

---

#### 2. `ls` - List Directory Contents

**What it does:** Shows files and folders

```bash
ls
```

**Useful variations:**

```bash
ls -l      # Long format (detailed info)
ls -a      # Show hidden files (start with .)
ls -lh     # Human-readable file sizes
ls -la     # Combine: long format + hidden files
```

**Example output:**
```
drwxr-xr-x 2 user user 4096 Jan 15 10:30 Documents
drwxr-xr-x 2 user user 4096 Jan 15 10:30 Downloads
-rw-r--r-- 1 user user  220 Jan 15 09:12 .bashrc
```

**🎯 Try it now:**
```bash
ls -la
```

**Instructor note:** Point out `.bashrc` - "This file is crucial for ROS 2 environment setup. We'll edit it later."

---

#### 3. `cd` - Change Directory

**What it does:** Move between folders

```bash
cd Documents        # Go to Documents folder
cd ..               # Go up one level (parent directory)
cd ~                # Go to home directory
cd /                # Go to root directory
cd -                # Go to previous directory
```

**Practice sequence:**

```bash
cd Documents
pwd
cd ..
pwd
cd ~
pwd
```

---

#### 4. The Linux File System Structure

**Key directories you'll use in ROS 2:**

```
/                  # Root (top of everything)
├── home/          # User home directories
│   └── username/  # Your files (~ shortcut)
├── opt/           # ROS 2 installation location (/opt/ros/jazzy)
├── usr/           # User programs
│   ├── bin/       # Executable programs
│   └── local/     # Locally compiled software
└── etc/           # Configuration files
```

**Navigate to ROS 2 installation:**

```bash
cd /opt/ros/jazzy
ls
```

**You should see:**
```
bin  include  lib  setup.bash  share  ...
```

**Instructor note:** "This is where ROS 2 lives. That `setup.bash` file? Critical. We'll use it constantly."

---

### Keyboard Shortcuts (Terminal Productivity)

| Shortcut | Action |
|----------|--------|
| `Ctrl + C` | Stop running command |
| `Ctrl + D` | Exit terminal / logout |
| `Ctrl + L` | Clear screen (same as `clear`) |
| `Tab` | Auto-complete commands/paths |
| `↑` / `↓` | Navigate command history |
| `Ctrl + R` | Search command history |
| `Ctrl + A` | Jump to start of line |
| `Ctrl + E` | Jump to end of line |
| `Ctrl + U` | Delete from cursor to start |
| `Ctrl + K` | Delete from cursor to end |

---

### 🎯 Hands-On Challenge 1: Terminal Navigation

**Time: 5 minutes**

Complete these tasks:

1. Find your current directory
2. List all files including hidden ones
3. Navigate to `/opt/ros/jazzy`
4. List the contents
5. Return to your home directory
6. Use `↑` to see your command history

**Solution:**

```bash
pwd
ls -la
cd /opt/ros/jazzy
ls
cd ~
# Press up arrow to see previous commands
```

**🎯 Instructor Checkpoint:** Ask 2-3 students to share their screens or describe what they see.

---

## Section 3: File Operations (25 minutes)

### Goals
- Create directories and files
- Copy, move, and delete files safely
- Understand absolute vs relative paths
- Use wildcards and patterns

### Creating a ROS 2 Workspace Structure

**Instructor context:** "In ROS 2, we organize code in workspaces. Let's practice creating that structure."

---

#### 1. `mkdir` - Make Directory

**What it does:** Creates folders

```bash
mkdir ros2_ws                    # Create single folder
mkdir -p ros2_ws/src            # Create nested folders (-p = parent)
mkdir folder1 folder2 folder3   # Create multiple folders
```

**Practice:**

```bash
cd ~
mkdir ros2_workshop
cd ros2_workshop
mkdir -p practice/src
ls
```

---

#### 2. `touch` - Create Empty File

**What it does:** Creates new empty files

```bash
touch file.txt
touch script.py config.yaml launch.py
```

**Practice - Create ROS 2 package structure:**

```bash
cd ~/ros2_workshop/practice/src
mkdir my_robot_pkg
cd my_robot_pkg
touch package.xml setup.py setup.cfg
touch __init__.py
ls -l
```

**Instructor note:** "Every ROS 2 Python package needs these files. We'll fill them with content in future sessions."

---

#### 3. `echo` and Output Redirection

**What it does:** Print text or write to files

```bash
echo "Hello ROS 2"                    # Print to screen
echo "Hello ROS 2" > file.txt         # Write to file (overwrite)
echo "Another line" >> file.txt       # Append to file
```

**Create a README:**

```bash
cd ~/ros2_workshop
echo "# ROS 2 Workshop Practice" > README.md
echo "This folder contains practice files." >> README.md
echo "" >> README.md
echo "Author: Your Name" >> README.md
cat README.md  # Display file contents
```

---

#### 4. `cat` - Concatenate and Display Files

**What it does:** Shows file contents

```bash
cat filename.txt              # Display file
cat file1.txt file2.txt       # Display multiple files
cat file1.txt > combined.txt  # Combine files
```

**Practice:**

```bash
echo "ROS 2 Node 1" > node1.txt
echo "ROS 2 Node 2" > node2.txt
cat node1.txt node2.txt
cat node1.txt node2.txt > nodes_combined.txt
cat nodes_combined.txt
```

---

#### 5. `cp` - Copy Files and Directories

**What it does:** Duplicates files/folders

```bash
cp source.txt destination.txt           # Copy file
cp source.txt /path/to/destination/     # Copy to directory
cp -r folder/ new_folder/               # Copy directory (-r = recursive)
cp *.txt backup/                        # Copy all .txt files
```

**Practice - Backup configuration:**

```bash
cd ~/ros2_workshop
mkdir backup
cp README.md backup/
cp README.md backup/README_backup.md
ls backup/
```

---

#### 6. `mv` - Move or Rename Files

**What it does:** Moves or renames files/folders

```bash
mv oldname.txt newname.txt              # Rename file
mv file.txt /path/to/destination/       # Move file
mv folder/ /path/to/destination/        # Move directory
mv *.py scripts/                        # Move all Python files
```

**Practice - Organize files:**

```bash
cd ~/ros2_workshop
mkdir scripts config
touch test_script.py sensor_config.yaml
mv test_script.py scripts/
mv sensor_config.yaml config/
ls scripts/
ls config/
```

---

#### 7. `rm` - Remove Files (USE WITH CAUTION!)

**What it does:** Deletes files/folders **permanently** (no Recycle Bin!)

```bash
rm file.txt                     # Delete file
rm -r folder/                   # Delete directory (-r = recursive)
rm -f file.txt                  # Force delete (no confirmation)
rm -rf folder/                  # Force delete directory (DANGEROUS!)
rm *.txt                        # Delete all .txt files
```

**⚠️ WARNING:** There is **NO UNDO** in Linux terminal. Deleted = gone forever.

**Safe practice:**

```bash
cd ~/ros2_workshop
touch temp1.txt temp2.txt temp3.txt
ls
rm temp1.txt          # Delete single file
ls
rm temp*.txt          # Delete remaining temp files
ls
```

**Instructor note:** "Always double-check before using `rm -rf`. One wrong command can delete your entire workspace."

---

### Understanding Paths

#### Absolute Path
Starts from root (`/`)

```bash
cd /home/username/ros2_workshop
cat /opt/ros/jazzy/setup.bash
```

#### Relative Path
Relative to current directory

```bash
cd ros2_workshop        # Relative to current location
cd ../                  # Up one level
cd ./scripts           # Current directory, then scripts
```

---

### Wildcards and Patterns

| Pattern | Matches |
|---------|---------|
| `*` | Any characters |
| `?` | Single character |
| `[abc]` | Any of: a, b, or c |
| `[0-9]` | Any digit |
| `{py,cpp}` | py OR cpp |

**Examples:**

```bash
ls *.txt                    # All .txt files
ls node_?.py                # node_1.py, node_a.py, etc.
ls script[123].py           # script1.py, script2.py, script3.py
ls *.{py,cpp}               # All Python and C++ files
```

---

### 🎯 Hands-On Challenge 2: File Operations

**Time: 10 minutes**

Create this structure in `~/ros2_workshop`:

```
ros2_workshop/
├── my_robot/
│   ├── launch/
│   │   └── robot.launch.py
│   ├── config/
│   │   └── params.yaml
│   └── README.md
└── backup/
    └── README_backup.md
```

**Tasks:**

1. Create all directories
2. Create all files (use `touch` or `echo`)
3. Copy README.md to backup/
4. Add text to robot.launch.py: "# ROS 2 Launch File"
5. List all .py files in the workspace
6. Delete any temporary files you created

**Solution:**

```bash
cd ~/ros2_workshop
mkdir -p my_robot/launch
mkdir -p my_robot/config
mkdir -p backup

touch my_robot/launch/robot.launch.py
touch my_robot/config/params.yaml
echo "# My Robot Package" > my_robot/README.md

cp my_robot/README.md backup/README_backup.md

echo "# ROS 2 Launch File" > my_robot/launch/robot.launch.py

find . -name "*.py"
# or
ls -R | grep .py
```

---

## Section 4: Permissions and sudo (15 minutes)

### Goals
- Understand Linux file permissions
- Learn about users, groups, and others
- Use `sudo` safely
- Modify file permissions when needed

### Why Permissions Matter in Robotics

**Real scenarios:**

- **Robot hardware access**: USB ports, serial devices need proper permissions
- **ROS 2 nodes**: Some nodes need root access for hardware control
- **System configuration**: Editing network settings, installing drivers
- **Security**: Prevent unauthorized access to robot systems

---

### Understanding File Permissions

When you run `ls -l`:

```bash
ls -l /opt/ros/jazzy/setup.bash
```

**Output:**
```
-rw-r--r-- 1 root root 1234 Jan 15 10:30 setup.bash
```

**Breaking it down:**

```
-rw-r--r--  1  root  root  1234  Jan 15 10:30  setup.bash
│└─┬─┘│└─┬─┘│  │     │     │     │              │
│  │  │  │  │  │     │     │     │              └─ filename
│  │  │  │  │  │     │     │     └─ date/time
│  │  │  │  │  │     │     └─ size (bytes)
│  │  │  │  │  │     └─ group
│  │  │  │  │  └─ owner
│  │  │  │  └─ number of links
│  │  │  └─ others permissions
│  │  └─ group permissions
│  └─ user permissions
└─ file type (- = file, d = directory)
```

---

### Permission Types

| Symbol | Permission | Meaning |
|--------|------------|---------|
| `r` | Read | View file contents / List directory |
| `w` | Write | Modify file / Create files in directory |
| `x` | Execute | Run file as program / Enter directory |
| `-` | None | No permission |

---

### Permission Groups

1. **Owner (User)**: Person who created the file
2. **Group**: Users in the same group
3. **Others**: Everyone else

**Example:**

```
-rwxr-xr-x
```

- Owner: `rwx` (read, write, execute)
- Group: `r-x` (read, execute)
- Others: `r-x` (read, execute)

---

### Numeric Permission Notation

Permissions can be represented as numbers:

| Number | Permission | Binary |
|--------|------------|--------|
| 0 | `---` | 000 |
| 1 | `--x` | 001 |
| 2 | `-w-` | 010 |
| 3 | `-wx` | 011 |
| 4 | `r--` | 100 |
| 5 | `r-x` | 101 |
| 6 | `rw-` | 110 |
| 7 | `rwx` | 111 |

**Common combinations:**

- `755`: `rwxr-xr-x` (executable scripts)
- `644`: `rw-r--r--` (normal files)
- `777`: `rwxrwxrwx` (all permissions - usually bad practice!)

---

### `chmod` - Change File Permissions

**Syntax:**

```bash
chmod [permissions] filename
```

**Using symbols:**

```bash
chmod +x script.py      # Add execute permission for everyone
chmod u+x script.py     # Add execute for user (owner)
chmod g-w file.txt      # Remove write for group
chmod o-r file.txt      # Remove read for others
chmod a+r file.txt      # Add read for all (a = all)
```

**Using numbers:**

```bash
chmod 755 script.py     # rwxr-xr-x
chmod 644 config.yaml   # rw-r--r--
chmod 700 private.sh    # rwx------ (owner only)
```

**Practice - Make a script executable:**

```bash
cd ~/ros2_workshop
echo '#!/bin/bash' > test_script.sh
echo 'echo "Hello from ROS 2!"' >> test_script.sh

ls -l test_script.sh    # Check current permissions
chmod +x test_script.sh # Make executable
ls -l test_script.sh    # Check again

./test_script.sh        # Run the script
```

---

### `sudo` - Superuser Do

**What it does:** Run commands as administrator (root)

**When you need it:**

- Installing software system-wide
- Editing system configuration files
- Accessing hardware devices
- Starting system services

**Syntax:**

```bash
sudo command
```

**Common uses:**

```bash
sudo apt update                    # Update package lists
sudo apt install python3-pip       # Install packages
sudo nano /etc/hosts               # Edit system files
sudo chmod 666 /dev/ttyUSB0        # Change device permissions
```

**⚠️ IMPORTANT:** `sudo` gives full system access. Always:
1. Understand what the command does
2. Double-check before pressing Enter
3. Never run `sudo rm -rf /` or similar destructive commands

---

### Checking Your User and Groups

```bash
whoami              # Show current username
groups              # Show groups you belong to
id                  # Show user ID and group IDs
sudo -l             # Show what commands you can run with sudo
```

---

### 🎯 Hands-On Challenge 3: Permissions

**Time: 5 minutes**

1. Create a Python script called `robot_controller.py`
2. Add this content:
   ```python
   #!/usr/bin/env python3
   print("Robot initialized")
   ```
3. Make it executable
4. Run it
5. Check its permissions in long format

**Solution:**

```bash
cd ~/ros2_workshop
echo '#!/usr/bin/env python3' > robot_controller.py
echo 'print("Robot initialized")' >> robot_controller.py

chmod +x robot_controller.py
./robot_controller.py

ls -l robot_controller.py
```

**Expected output:**
```
-rwxr-xr-x 1 username username 54 Jan 15 14:30 robot_controller.py
Robot initialized
```

---

## Section 5: Package Management (15 minutes)

### Goals
- Understand APT package manager
- Install and remove software
- Update system packages
- Install ROS 2 dependencies

### Why Package Management Matters

**In ROS 2 development, you'll constantly install:**

- ROS 2 packages (`ros-jazzy-nav2-*`)
- Python libraries (`python3-numpy`, `python3-opencv`)
- Build tools (`colcon`, `rosdep`)
- Development tools (`git`, `curl`, `wget`)
- Simulation software (`gazebo`, `rviz2`)

**APT = Your software installer for Ubuntu.**

---

### APT Command Overview

| Command | Purpose |
|---------|---------|
| `sudo apt update` | Refresh package lists from repositories |
| `sudo apt upgrade` | Upgrade installed packages |
| `sudo apt install package` | Install new package |
| `sudo apt remove package` | Uninstall package |
| `sudo apt autoremove` | Remove unused dependencies |
| `sudo apt search keyword` | Search for packages |
| `apt show package` | Show package details |
| `apt list --installed` | List installed packages |

---

### Essential APT Workflow

**Always start with update:**

```bash
sudo apt update
```

**What it does:** Downloads latest package lists from Ubuntu servers. Doesn't install anything.

**Then upgrade (optional):**

```bash
sudo apt upgrade
```

**What it does:** Upgrades all installed packages to latest versions.

---

### Installing Packages

**Single package:**

```bash
sudo apt install git
```

**Multiple packages:**

```bash
sudo apt install git curl wget
```

**Answer "yes" to prompts:**

```bash
sudo apt install -y python3-pip    # -y flag auto-confirms
```

---

### Practice - Install Essential Tools

**Let's install common ROS 2 development tools:**

```bash
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions git
```

**Verify installations:**

```bash
pip3 --version
colcon version-check
git --version
```

---

### Installing ROS 2 Packages

**ROS 2 packages follow naming convention:**

```
ros-<distro>-<package-name>
```

**Examples:**

```bash
# Navigation stack
sudo apt install ros-jazzy-nav2-bringup

# Simulation tools
sudo apt install ros-jazzy-gazebo-ros-pkgs

# Robot description tools
sudo apt install ros-jazzy-xacro

# Visualization
sudo apt install ros-jazzy-rviz2
```

**Search for ROS 2 packages:**

```bash
apt search ros-jazzy-
apt search ros-jazzy-turtlesim
```

---

### Removing Packages

**Uninstall package:**

```bash
sudo apt remove package-name
```

**Remove package + configuration files:**

```bash
sudo apt purge package-name
```

**Clean up unused dependencies:**

```bash
sudo apt autoremove
```

**Practice:**

```bash
sudo apt install cowsay          # Fun terminal program
cowsay "ROS 2 is awesome!"
sudo apt remove cowsay
```

---

### Package Information Commands

**Show package details:**

```bash
apt show python3-pip
```

**List installed packages:**

```bash
apt list --installed | grep ros
apt list --installed | grep python3
```

**Check if package is installed:**

```bash
dpkg -l | grep package-name
```

---

### 🎯 Hands-On Challenge 4: Package Management

**Time: 5 minutes**

1. Update package lists
2. Install `tree` (displays directory structure)
3. Use `tree` to view your ros2_workshop structure
4. Install `htop` (system monitor)
5. Run `htop` and close it (press `q`)
6. Verify both are installed

**Solution:**

```bash
sudo apt update
sudo apt install -y tree htop

tree ~/ros2_workshop

htop
# Press 'q' to quit

dpkg -l | grep tree
dpkg -l | grep htop
```

---

## Section 6: VS Code Setup (15 minutes)

### Goals
- Install Visual Studio Code
- Configure essential extensions
- Set up workspace for ROS 2
- Learn basic VS Code shortcuts

### Why VS Code for ROS 2?

**Industry standard IDE for robotics:**

- IntelliSense (code completion)
- Integrated terminal
- Git integration
- Python debugging
- ROS extensions
- Multi-language support (Python, C++, YAML, XML)

**Most ROS 2 professionals use VS Code.**

---

### Installing VS Code

**Method 1: From Ubuntu Software Center**

1. Open "Ubuntu Software" application
2. Search "Visual Studio Code"
3. Click Install

**Method 2: Terminal (Snap)**

```bash
sudo snap install code --classic
```

**Method 3: Official .deb package**

```bash
cd ~/Downloads
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/
sudo sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt update
sudo apt install code
```

**Launch VS Code:**

```bash
code
```

**🎯 Instructor Checkpoint:** Ensure all students have VS Code open.

---

### Essential VS Code Extensions for ROS 2

**Open Extensions panel:** `Ctrl + Shift + X`

**Install these extensions:**

1. **Python** (Microsoft)
   - Python language support, IntelliSense, debugging

2. **Pylance** (Microsoft)
   - Advanced Python language server

3. **ROS** (Microsoft)
   - ROS/ROS 2 integration, launch file support

4. **C/C++** (Microsoft)
   - For C++ ROS 2 nodes

5. **CMake** (twxs)
   - CMakeLists.txt support

6. **XML** (Red Hat)
   - package.xml and launch file support

7. **YAML** (Red Hat)
   - YAML configuration files

8. **GitLens** (GitKraken)
   - Enhanced Git integration

**To install via terminal:**

```bash
code --install-extension ms-python.python
code --install-extension ms-python.vscode-pylance
code --install-extension ms-iot.vscode-ros
code --install-extension ms-vscode.cpptools
code --install-extension twxs.cmake
code --install-extension redhat.vscode-xml
code --install-extension redhat.vscode-yaml
```

---

### Opening Workspace in VS Code

**From terminal:**

```bash
cd ~/ros2_workshop
code .
```

**The `.` means "current directory"**

**From VS Code:**

1. File → Open Folder
2. Navigate to `ros2_workshop`
3. Click Open

---

### Essential VS Code Shortcuts

| Shortcut | Action |
|----------|--------|
| `Ctrl + ~` | Toggle integrated terminal |
| `Ctrl + Shift + E` | File Explorer |
| `Ctrl + Shift + F` | Search across files |
| `Ctrl + P` | Quick file open |
| `Ctrl + Shift + P` | Command Palette |
| `Ctrl + B` | Toggle sidebar |
| `Ctrl + /` | Toggle comment |
| `Ctrl + Space` | Trigger IntelliSense |
| `F2` | Rename symbol |
| `Ctrl + Shift + [` | Fold code |
| `Ctrl + Shift + ]` | Unfold code |

---

### Configuring Python Interpreter

1. `Ctrl + Shift + P`
2. Type "Python: Select Interpreter"
3. Choose Python 3.12 (or your system Python 3)

**Verify:**

```bash
which python3
```

Should show: `/usr/bin/python3`

---

### VS Code Settings for ROS 2

**Open settings:** `Ctrl + ,`

**Recommended settings:**

```json
{
    "files.associations": {
        "*.launch": "xml",
        "*.xacro": "xml",
        "*.urdf": "xml"
    },
    "python.autoComplete.extraPaths": [
        "/opt/ros/jazzy/lib/python3.12/site-packages"
    ],
    "python.analysis.extraPaths": [
        "/opt/ros/jazzy/lib/python3.12/site-packages"
    ],
    "ros.distro": "jazzy",
    "editor.rulers": [100],
    "files.trimTrailingWhitespace": true
}
```

**To add these settings:**

1. `Ctrl + Shift + P`
2. Type "Preferences: Open Settings (JSON)"
3. Paste the configuration above

---

### Creating a ROS 2 Python File in VS Code

**Let's create a proper ROS 2 node structure:**

```bash
cd ~/ros2_workshop/practice/src/my_robot_pkg
code .
```

**Create file:** `my_first_node.py`

**Add this code:**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.get_logger().info('Hello from ROS 2!')

def main(args=None):
    rclpy.init(args=args)
    node = MyFirstNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Make it executable:**

```bash
chmod +x my_first_node.py
```

**Instructor note:** "This is a minimal ROS 2 node. We'll explain every line in the next workshop. For now, focus on the workflow."

---

### 🎯 Hands-On Challenge 5: VS Code Setup

**Time: 5 minutes**

1. Open `~/ros2_workshop` in VS Code
2. Install at least 3 required extensions
3. Open integrated terminal (`Ctrl + ~`)
4. Create a new Python file: `hello_ros2.py`
5. Write a simple print statement
6. Run it from VS Code terminal

**Solution:**

```bash
# In VS Code terminal
cd ~/ros2_workshop
touch hello_ros2.py
```

**In `hello_ros2.py`:**

```python
#!/usr/bin/env python3
print("Hello from ROS 2 Workshop!")
```

**Run:**

```bash
python3 hello_ros2.py
```

---

## Section 7: ROS 2 Verification (10 minutes)

### Goals
- Verify ROS 2 installation
- Source ROS 2 environment
- Run basic ROS 2 commands
- Test example nodes

### Sourcing the ROS 2 Environment

**What "sourcing" means:**

- Adds ROS 2 commands to your terminal session
- Sets up environment variables
- Enables ROS 2 tools (`ros2`, `colcon`, etc.)

**Source command:**

```bash
source /opt/ros/jazzy/setup.bash
```

**Verify it worked:**

```bash
ros2 --help
```

**You should see a list of ROS 2 commands.**

---

### Making Sourcing Automatic

**Edit `.bashrc` file (runs every time terminal opens):**

```bash
nano ~/.bashrc
```

**Add this line at the end:**

```bash
source /opt/ros/jazzy/setup.bash
```

**Save and exit:**

- Press `Ctrl + O` (save)
- Press `Enter` (confirm)
- Press `Ctrl + X` (exit)

**Apply changes:**

```bash
source ~/.bashrc
```

**Verify:**

```bash
echo $ROS_DISTRO
```

**Expected output:** `jazzy`

---

### Basic ROS 2 Commands

**Check ROS 2 version:**

```bash
ros2 --version
```

**List available commands:**

```bash
ros2 -h
```

**Common commands you'll use:**

| Command | Purpose |
|---------|---------|
| `ros2 run` | Run a node from a package |
| `ros2 launch` | Launch multiple nodes |
| `ros2 node list` | Show active nodes |
| `ros2 topic list` | Show active topics |
| `ros2 topic echo` | Display topic data |
| `ros2 pkg list` | List installed packages |
| `ros2 pkg create` | Create new package |

---

### Testing ROS 2 with Turtlesim

**Turtlesim is ROS 2's "Hello World" - a simple simulator.**

**Install turtlesim:**

```bash
sudo apt install ros-jazzy-turtlesim
```

**Run turtlesim node:**

```bash
ros2 run turtlesim turtlesim_node
```

**You should see a window with a turtle!**

**Open NEW terminal** (`Ctrl + Shift + T` or File → New Tab)

**Control the turtle:**

```bash
source /opt/ros/jazzy/setup.bash
ros2 run turtlesim turtle_teleop_key
```

**Use arrow keys to move the turtle.**

---

### Checking ROS 2 System

**List active nodes:**

```bash
ros2 node list
```

**Expected output:**
```
/turtlesim
/teleop_turtle
```

**List active topics:**

```bash
ros2 topic list
```

**Expected output:**
```
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

**Echo a topic (see real-time data):**

```bash
ros2 topic echo /turtle1/pose
```

**Move the turtle and watch values change!**

**Stop echoing:** `Ctrl + C`

---

### Environment Variables Check

**Important ROS 2 environment variables:**

```bash
echo $ROS_DISTRO           # Should show: jazzy
echo $ROS_VERSION          # Should show: 2
echo $ROS_PYTHON_VERSION   # Should show: 3
echo $ROS_LOCALHOST_ONLY   # Should show: 0 or 1
```

---

### 🎯 Hands-On Challenge 6: ROS 2 Verification

**Time: 5 minutes**

1. Open a fresh terminal
2. Verify ROS 2 is sourced automatically (check `$ROS_DISTRO`)
3. Run turtlesim
4. List all active nodes
5. List all active topics
6. Echo the `/turtle1/cmd_vel` topic
7. In another terminal, control the turtle and observe the topic data

**Solution:**

```bash
# Terminal 1
echo $ROS_DISTRO
ros2 run turtlesim turtlesim_node

# Terminal 2
ros2 node list
ros2 topic list
ros2 topic echo /turtle1/cmd_vel

# Terminal 3
ros2 run turtlesim turtle_teleop_key
# Use arrow keys
```

**🎯 Instructor Checkpoint:** Confirm everyone sees the turtle moving and topic data streaming.

---

## Section 8: Final Task Explanation (5 minutes)

### Final Mandatory Task

You will submit a **single Markdown file** demonstrating your Linux and ROS 2 setup.

**Deadline:** [Instructor: specify date/time]

**Submission method:** [Instructor: specify - email, LMS, GitHub, etc.]

---

### Task Requirements

**Create a file:** `ros2_setup_report.md`

**Include the following sections:**

#### 1. System Information

```bash
# Commands to run:
uname -a
lsb_release -a
ros2 --version
```

Copy output to your report.

#### 2. ROS 2 Environment

```bash
# Commands to run:
echo $ROS_DISTRO
echo $ROS_VERSION
cat ~/.bashrc | grep ros
```

Include the output and explain what sourcing does.

#### 3. Installed ROS 2 Packages

```bash
apt list --installed | grep ros-jazzy | head -10
```

List at least 10 ROS 2 packages.

#### 4. Workspace Structure

Create this structure and document it:

```
~/ros2_workspace/
├── src/
│   └── my_first_package/
│       ├── my_first_package/
│       │   └── __init__.py
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
└── README.md
```

Run `tree ~/ros2_workspace` and include output.

#### 5. VS Code Configuration

- Screenshot of VS Code with ROS extensions installed
- List of extensions you installed

#### 6. Turtlesim Test

- Screenshot of turtlesim running
- Output of `ros2 node list`
- Output of `ros2 topic list`

#### 7. Custom Script

Create a bash script `ros2_check.sh`:

```bash
#!/bin/bash
echo "=== ROS 2 Environment Check ==="
echo "ROS Distro: $ROS_DISTRO"
echo "ROS Version: $ROS_VERSION"
echo "Python Version:"
python3 --version
echo "Installed ROS 2 packages:"
apt list --installed | grep ros-jazzy | wc -l
echo "Check complete!"
```

Make it executable and run it. Include output in your report.

#### 8. Reflection

Answer these questions (2-3 sentences each):

1. What Linux command do you think will be most useful for ROS 2 development? Why?
2. What was the most challenging part of this workshop?
3. What ROS 2 topic are you most excited to learn about next?

---

### Submission Checklist

✅ Markdown file is properly formatted  
✅ All 8 sections are complete  
✅ Commands and outputs are in code blocks  
✅ Screenshots are included (Turtlesim, VS Code)  
✅ Workspace structure is documented  
✅ Custom script is included and tested  
✅ Reflection questions are answered  
✅ File is named: `ros2_setup_report.md`

---

### Example Markdown Structure

```markdown
# ROS 2 Environment Setup Report

**Name:** [Your Name]  
**Date:** [Submission Date]  
**Course:** ROS 2 Workshop Series

---

## 1. System Information

```bash
$ uname -a
Linux ubuntu 6.8.0-49-generic #49-Ubuntu SMP...
```

## 2. ROS 2 Environment

[Continue with each section...]
```

---

### Grading Criteria

| Section | Points |
|---------|--------|
| System Information | 10 |
| ROS 2 Environment | 15 |
| Installed Packages | 10 |
| Workspace Structure | 15 |
| VS Code Config | 10 |
| Turtlesim Test | 15 |
| Custom Script | 15 |
| Reflection | 10 |
| **Total** | **100** |

---

## Quick Reference

### Essential Linux Commands

```bash
pwd                      # Print working directory
ls -la                   # List all files with details
cd [directory]           # Change directory
mkdir [name]             # Create directory
touch [file]             # Create empty file
cp [source] [dest]       # Copy
mv [source] [dest]       # Move/rename
rm [file]                # Delete file
rm -r [directory]        # Delete directory
cat [file]               # Display file
nano [file]              # Edit file
chmod +x [file]          # Make executable
```

### Essential APT Commands

```bash
sudo apt update          # Update package lists
sudo apt upgrade         # Upgrade packages
sudo apt install [pkg]   # Install package
sudo apt remove [pkg]    # Remove package
apt search [keyword]     # Search packages
apt show [pkg]           # Package details
```

### Essential ROS 2 Commands

```bash
source /opt/ros/jazzy/setup.bash    # Source ROS 2
ros2 run [pkg] [node]                # Run node
ros2 launch [pkg] [launch]           # Launch file
ros2 node list                       # List nodes
ros2 topic list                      # List topics
ros2 topic echo [topic]              # Monitor topic
ros2 pkg list                        # List packages
```

---

## Additional Resources

### Official Documentation

- **ROS 2 Documentation:** https://docs.ros.org/en/jazzy/
- **ROS 2 Tutorials:** https://docs.ros.org/en/jazzy/Tutorials.html
- **Ubuntu Documentation:** https://help.ubuntu.com/

### Learning Resources

- **Linux Journey:** https://linuxjourney.com/
- **The Linux Command Line (free book):** http://linuxcommand.org/tlcl.php
- **ROS 2 Humble Tutorials:** https://docs.ros.org/en/humble/Tutorials.html

### Community

- **ROS Discourse:** https://discourse.ros.org/
- **ROS Answers:** https://answers.ros.org/
- **r/ROS (Reddit):** https://reddit.com/r/ROS

---

## Instructor Notes

### Common Issues and Solutions

**Issue 1: "ros2 command not found"**
- **Solution:** Source setup file: `source /opt/ros/jazzy/setup.bash`
- **Prevention:** Add to `~/.bashrc`

**Issue 2: Permission denied when running script**
- **Solution:** `chmod +x script.sh`

**Issue 3: VS Code won't open**
- **Solution:** Try `sudo snap install code --classic`

**Issue 4: Turtlesim window doesn't appear**
- **Solution:** Check if package is installed: `sudo apt install ros-jazzy-turtlesim`

### Time Management Tips

- If running behind, skip Challenge 2 or Challenge 4
- Turtlesim demo can be shortened to 5 minutes
- VS Code extension installation can be assigned as pre-work

### Assessment Points

Focus on:
- Can students navigate using terminal?
- Can students create files and directories confidently?
- Do students understand permissions?
- Can students source ROS 2 and run basic commands?

### Extension Activities (if time allows)

- Create a bash alias for sourcing ROS 2
- Explore `ros2 doctor` command
- Set up GitHub and clone a repository
- Install `tmux` for terminal multiplexing

---

## What's Next

**In the next session, you'll learn:**

- ROS 2 concepts: nodes, topics, messages
- Creating your first ROS 2 package
- Writing publisher and subscriber nodes
- Understanding the ROS 2 graph
- Launching multiple nodes

**Prerequisites for next session:**
- Everything from today's workshop
- VS Code fully configured
- ROS 2 environment sourced automatically

---

## Session Complete! 🎉

**You now have:**

✅ Solid Linux terminal skills  
✅ File management expertise  
✅ Package management knowledge  
✅ VS Code configured for ROS 2  
✅ Verified ROS 2 installation  
✅ Foundation for robot development

**Remember:** The terminal is your superpower in robotics. Practice these commands daily!

**Questions?** Ask now or contact: [Instructor contact info]

---

**End of Workshop Session: Linux Basics and ROS 2 Environment Setup** 
