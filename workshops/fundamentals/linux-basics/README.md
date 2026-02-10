# Linux Basics for ROS 2 Development

---

## Opening of This Repository

This repository provides a structured technical course for acquiring the Linux skills required for professional ROS 2 robotics development. It targets individuals with no prior Linux experience who need command-line competency for robot programming.

**Repository contents:**

- Linux terminal operations and file system management
- System administration with package managers
- Development environment configuration
- ROS 2 environment setup and verification
- Progressive hands-on tasks with cumulative complexity

**Skills you will acquire:**

- File system navigation and management using terminal commands
- File permissions for hardware access and script execution
- Software installation and management with APT
- VS Code configuration for ROS 2 development
- ROS 2 environment sourcing and basic command execution
- Functional robotics development environment setup

This is a practical course designed to be worked through sequentially. Each section builds on the previous one.

---

## Why Linux Is Fundamental for ROS 2

ROS 2 is architected around Linux. Understanding this relationship is essential for effective robotics development.

### Linux as the Execution Layer of ROS 2

ROS 2 is middleware built on Linux operating system primitives:

- **Process management**: ROS 2 nodes are Linux processes
- **Inter-process communication**: ROS 2 uses Linux IPC mechanisms (shared memory, Unix domain sockets) for data exchange
- **File system structure**: ROS 2 packages follow Linux Filesystem Hierarchy Standard (FHS)
- **Environment variables**: ROS 2 uses shell environment variables for workspace sourcing and configuration
- **Package management**: ROS 2 packages are distributed as Debian packages via APT

### Why Ubuntu Is the Reference Platform

Ubuntu is the officially supported platform for ROS 2:

- **Long Term Support**: Ubuntu 24.04 LTS provides 5 years of security updates and stability
- **Official compatibility**: ROS 2 Jazzy Jalisco targets Ubuntu 24.04 LTS specifically
- **Package ecosystem**: APT is the primary distribution mechanism for ROS 2 binary packages
- **Community standard**: Most ROS 2 documentation and third-party packages assume Ubuntu

### Why Terminal-Based Workflows Are Mandatory

Graphical interfaces are unavailable in production robotics scenarios:

- **Remote access**: Robots are accessed via SSH, providing terminal-only access
- **Headless systems**: Production robots run without monitors or graphical environments
- **Automation**: Launch sequences, testing, and deployment are scripted for terminal execution
- **Performance**: Graphical environments consume resources needed for real-time control
- **Debugging**: ROS 2 introspection tools are terminal-based

### Critical Terminal Workflows in ROS 2

1. **Sourcing workspaces**: `source setup.bash` required for ROS 2 command access
2. **Building packages**: `colcon build` via terminal
3. **Running nodes**: `ros2 run` and `ros2 launch` are terminal commands
4. **Debugging**: Topic inspection and node monitoring via terminal tools
5. **Hardware configuration**: USB permissions and driver installation require `sudo`
6. **Dependency management**: Package installation via APT and pip

Terminal proficiency is mandatory for ROS 2 development, deployment, and debugging.

---

## Linux Overview & Scope of This Course

### What Linux Means in This Course

Linux is a family of open-source operating systems built on the Linux kernel. In robotics:

- **Open source**: Customizable for specific robot hardware
- **Stability**: Powers servers, embedded systems, and mission-critical infrastructure
- **Real-time capabilities**: Configurable for deterministic control (PREEMPT_RT patches)
- **Performance**: Minimal overhead compared to desktop operating systems
- **Community**: Massive robotics ecosystem centered on Ubuntu and ROS

### Ubuntu 24.04 LTS

Ubuntu is a Linux distribution maintained by Canonical. **LTS** (Long Term Support) provides:

- 5 years of security updates (April 2024 to April 2029)
- Package stability with no breaking changes
- Official ROS 2 Jazzy Jalisco target platform

This pairing ensures maximum compatibility and community support.

### The Terminal as the Control Interface

The terminal is a text-based interface to the operating system:

- Commands are typed and executed
- The system responds with output or performs actions
- No graphical interface is available on remote robot systems
- All ROS 2 nodes, tools, and debugging utilities launch from the terminal

The terminal is the primary interface for robotics professionals, not a fallback option.

### Course Scope

**In scope:**

- Terminal navigation and command syntax
- File and directory operations
- File permissions and ownership
- Package management with APT
- Text editing with nano
- VS Code configuration for ROS 2
- ROS 2 environment sourcing
- Basic ROS 2 verification commands

**Postponed to later sessions:**

- Bash scripting and automation
- ROS 2 package creation and colcon build systems
- Launch file authoring
- C++ compilation and CMake
- Git version control workflows
- Real-time kernel configuration

---

## Downloads & System Preparation

### Prerequisites

Before proceeding, ensure:

✅ Ubuntu 24.04 LTS installed (Virtual Machine or Dual Boot)
✅ ROS 2 Jazzy Jalisco installed (verification covered later)
✅ Working network connection for package downloads
✅ System running (VM powered on or dual boot booted)
✅ Terminal access available

**Open terminal:**

- Keyboard: `Ctrl + Alt + T`
- Application menu: Search "Terminal"

**Expected prompt:** `username@hostname:~$`

If you see this prompt, your system is ready.

---

### ROS 2 Jazzy Jalisco Installation

**IMPORTANT**: Use only the official ROS 2 documentation for installation. Third-party guides become outdated and cause broken installations.

**Official installation documentation:**

[https://docs.ros.org/en/jazzy/Installation.html](https://docs.ros.org/en/jazzy/Installation.html)

**Installation steps:**

1. Open the link above
2. Select **"Ubuntu (Debian packages)"**
3. Follow steps sequentially:
   - Set up sources
   - Install ROS 2 packages
   - Source the environment
4. Choose installation type:
   - **Desktop Install (Recommended)**: Includes RViz, demos, tutorials
   - **ROS-Base Install**: Communication libraries only

**Why official steps are mandatory:**

- Repository keys and sources change between releases
- Binary package names are version-specific
- Third-party instructions cause incompatibilities

After installation, return to this guide to verify your ROS 2 environment.

---

### VS Code Installation

#### What is VS Code?

Visual Studio Code (VS Code) is a source code editor that provides syntax highlighting, code completion (IntelliSense), integrated debugging, terminal access, Git integration, and extensibility via plugins.

VS Code is the industry standard IDE for robotics development due to its Python and C++ support, terminal integration, and ROS-specific extensions.

#### Why Install VS Code Now?

VS Code will be the primary environment for writing ROS 2 nodes, authoring launch files, editing configuration files, debugging applications, and managing packages and workspaces.

Installing it now ensures the development environment is ready when ROS 2 programming begins.

#### Download VS Code

**Official download page:**

[https://code.visualstudio.com/](https://code.visualstudio.com/)

**Steps:**

1. Open a web browser
2. Navigate to the link above
3. Click **"Download for Linux"**
4. Select **".deb"** package (for Ubuntu/Debian)
5. Save the file to `~/Downloads`

#### Install VS Code via Terminal

```bash
cd ~/Downloads
sudo apt install ./code_*.deb -y
```

**What this does:**

- `cd ~/Downloads`: Navigate to the Downloads directory
- `sudo`: Execute with administrator privileges (required for system-wide installation)
- `apt install ./code_*.deb`: Install the `.deb` package file
- `-y`: Automatically answer "yes" to confirmation prompts

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

#### Launch VS Code

**From terminal:**

```bash
code
```

**From application menu:**

Search "Visual Studio Code" in the application launcher.

---

### VS Code Extensions for ROS 2

Extensions add language support and tooling to VS Code. Open the Extensions panel with `Ctrl + Shift + X`.

#### Required Extensions

##### 1. Python (Microsoft)
Python language support, IntelliSense, and debugging for ROS 2 Python nodes.

##### 2. Pylance (Microsoft)
Advanced Python language server with type checking and faster code completion.

##### 3. ROS (Microsoft)
ROS/ROS 2 integration, launch file support, and URDF preview.

##### 4. C/C++ (Microsoft)
C++ language support, IntelliSense, and debugging for ROS 2 C++ nodes.

##### 5. CMake (twxs)
CMakeLists.txt syntax highlighting and IntelliSense for ROS 2 C++ packages.

##### 6. XML (Red Hat)
XML syntax highlighting and validation for package.xml manifests and launch files.

##### 7. YAML (Red Hat)
YAML syntax highlighting and validation for ROS 2 configuration and parameter files.

##### 8. GitLens (GitKraken)
Enhanced Git integration with blame annotations, history, and diffs.

##### 9. LaunchMap (launchmap)
Visualization and management for ROS 2 launch files and multi-node configurations.

#### Install Extensions via Terminal

```bash
code --install-extension ms-python.python
code --install-extension ms-python.vscode-pylance
code --install-extension ms-iot.vscode-ros
code --install-extension ms-vscode.cpptools
code --install-extension twxs.cmake
code --install-extension redhat.vscode-xml
code --install-extension redhat.vscode-yaml
code --install-extension eamodio.gitlens
code --install-extension launchmap.launchmap
```

#### Install Extensions via GUI

1. Open VS Code
2. Press `Ctrl + Shift + X` to open Extensions panel
3. Search for each extension name
4. Click **Install** for each one

**Do NOT configure VS Code settings or Python interpreters at this stage.** Configuration will be performed after workspace setup.

---

## Core Linux Environment Concepts

### The .bashrc File

#### What Is .bashrc?

`.bashrc` is a shell configuration file that runs every time a new terminal session is opened. It is located at `~/.bashrc` in your home directory. The leading dot (`.`) makes it a hidden file.

#### When .bashrc Runs

`.bashrc` executes automatically when:

- A new terminal window or tab is opened
- An SSH session is established
- A terminal is launched within VS Code

It does NOT run when the system boots or when scripts execute (unless explicitly sourced).

#### Why ROS 2 Depends on .bashrc

ROS 2 requires environment variables before commands are available:

- `ROS_DISTRO`: ROS 2 distribution (e.g., `jazzy`)
- `ROS_VERSION`: ROS version (1 or 2)
- `PYTHONPATH`: ROS 2 Python libraries path
- `PATH`: ROS 2 executable directories
- `LD_LIBRARY_PATH`: ROS 2 shared libraries

Without these variables, commands like `ros2`, `colcon`, and `rviz2` will not be found.

#### How ROS 2 Environment Setup Works

ROS 2 provides a setup script at `/opt/ros/jazzy/setup.bash`. Sourcing this script sets required environment variables:

```bash
source /opt/ros/jazzy/setup.bash
echo $ROS_DISTRO
```

**Expected output:** `jazzy`

#### Making ROS 2 Sourcing Automatic

Add the source command to `.bashrc` to avoid manual sourcing in every terminal:

```bash
nano ~/.bashrc
```

Scroll to the end and add:

```bash
source /opt/ros/jazzy/setup.bash
```

Save (`Ctrl + O`, `Enter`) and exit (`Ctrl + X`).

Apply changes:

```bash
source ~/.bashrc
```

Verify by opening a new terminal:

```bash
echo $ROS_DISTRO
```

**Expected output:** `jazzy`

Every new terminal will now have ROS 2 available automatically.

---

### Files vs Folders: Understanding the Linux File System

#### Conceptual Overview

Linux organizes data hierarchically:

- **Files**: Contain data (text, code, images, binaries)
- **Directories (Folders)**: Contain files and other directories
- **Root directory**: Top of hierarchy (`/`)

Everything is either a file or directory. Hardware devices appear as files in `/dev`.

#### Hierarchical Structure

```
/                           # Root (top level)
├── home/                   # User home directories
│   └── username/           # Personal files (accessible via ~)
│       ├── Documents/
│       ├── Downloads/
│       └── ros2_ws/        # ROS 2 workspace
├── opt/                    # Optional software
│   └── ros/jazzy/          # ROS 2 Jazzy installation
├── usr/                    # User programs and utilities
│   ├── bin/                # Executables
│   ├── lib/                # Shared libraries
│   └── local/              # Locally compiled software
└── etc/                    # System configuration
```

#### Working Example: Creating a ROS 2 Workspace Structure

Create a typical ROS 2 workspace with directories and files:

```bash
cd ~
mkdir ros2_workshop
cd ros2_workshop
mkdir -p practice/src/my_robot_pkg      # -p creates parent dirs as needed
cd practice/src/my_robot_pkg
touch package.xml setup.py setup.cfg    # Create package files
mkdir my_robot_pkg
touch my_robot_pkg/__init__.py          # Python module marker
ls -la                                  # View structure
```

Navigate up and view full structure:

```bash
cd ../../..                             # Up three levels to workspace root
pwd                                      # Output: /home/username/ros2_workshop
sudo apt install -y tree
tree                                    # View hierarchical structure
```

**Output:**

```
.
└── practice
    └── src
        └── my_robot_pkg
            ├── my_robot_pkg
            │   └── __init__.py
            ├── package.xml
            ├── setup.cfg
            └── setup.py

3 directories, 4 files
```

**Standard ROS 2 Python package structure:**

- `package.xml`: Package manifest (dependencies, metadata)
- `setup.py`: Python installation script
- `setup.cfg`: Python configuration
- `my_robot_pkg/__init__.py`: Python module marker

---

## Command Line Foundations

### Understanding the Terminal Prompt

Terminal prompt format: `username@hostname:~$`

- `username`: Your login name
- `hostname`: Computer name (e.g., "ubuntu")
- `:~`: Current directory (`~` = home directory)
- `$`: Regular user prompt (`#` = root user)

### Why the Command Line Exists

The command line provides direct operating system access without graphical abstraction. Essential in robotics for:

- **Precision**: Explicit, repeatable commands
- **Automation**: Scriptable for autonomous execution
- **Remote access**: SSH connections to robots are terminal-only
- **Performance**: Minimal overhead compared to GUI applications
- **Documentation**: Command sequences can be shared exactly as written

The command line is the professional standard for robotics development.

### ROS 2 Terminal Workflows

Every ROS 2 workflow uses the terminal:

1. **Building**: `colcon build`
2. **Running nodes**: `ros2 run package_name node_name`
3. **Launching systems**: `ros2 launch package_name launch_file.py`
4. **Inspecting topics**: `ros2 topic list`, `ros2 topic echo /topic_name`
5. **Checking nodes**: `ros2 node list`, `ros2 node info /node_name`
6. **Managing parameters**: `ros2 param list`, `ros2 param set`

No graphical alternative exists for most operations.

---

### Navigation Commands

#### 1. `cd` - Change Directory

Moves the terminal's working location between directories.

```bash
cd Documents          # Navigate to Documents
cd ~                  # Home directory
cd /                  # Root directory
cd ..                 # Parent directory
cd -                  # Previous directory
```

**Practice:**

```bash
cd ~
cd Documents
cd ..
cd /opt/ros/jazzy
cd ~
```

**ROS 2 usage**: Navigate to workspaces before building (`cd ~/ros2_ws`) or to package directories when editing code.

---

#### 2. `ls` - List Directory Contents

Displays files and directories in the current location.

```bash
ls              # Basic listing
ls -la          # Long format + hidden files
```

**Example output (`ls -la`):**

```
drwxr-xr-x 2 username username 4096 Jan 15 10:30 Documents
-rw-r--r-- 1 username username  220 Jan 15 09:12 .bashrc
```

**Output columns**: permissions, links, owner, group, size, date/time, name

**ROS 2 usage**: Check workspace contents, verify package structures, identify build artifacts.

---

#### 3. `pwd` - Print Working Directory

Displays the absolute path of the current location.

```bash
pwd
```

**Example output:** `/home/username/ros2_ws/src`

**ROS 2 usage**: Confirm correct workspace location before running `colcon build` or sourcing setup scripts.

---

### Terminal Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Ctrl + C` | Stop running command |
| `Ctrl + D` | Exit terminal |
| `Ctrl + L` | Clear screen |
| `Tab` | Auto-complete commands and paths |
| `Tab` `Tab` | Show all completions |
| `↑` / `↓` | Navigate command history |
| `Ctrl + R` | Search command history |
| `Ctrl + A` | Jump to start of line |
| `Ctrl + E` | Jump to end of line |
| `Ctrl + U` | Delete to start of line |
| `Ctrl + K` | Delete to end of line |
| `Ctrl + W` | Delete previous word |

**Tab completion example:**

```bash
cd /opt/ros/ja<TAB>    # Completes to: cd /opt/ros/jazzy/
```

Tab completion ensures correct capitalization and prevents "file not found" errors.

---

## File & Folder Operations

### 1. `mkdir` - Make Directory

Creates new directories.

```bash
mkdir folder_name                    # Single directory
mkdir folder1 folder2 folder3        # Multiple directories
mkdir -p parent/child/grandchild     # Nested directories
```

**Practice:**

```bash
cd ~
mkdir -p ros2_ws/src
cd ros2_ws
ls                                   # Output: src
```

**ROS 2 usage**: Create workspace structures (`ros2_ws/src`), package directories, configuration folders.

---

### 2. `touch` - Create Empty File

Creates a new empty file or updates the timestamp of an existing file.

```bash
touch filename.txt                   # Single file
touch file1.txt file2.txt file3.txt  # Multiple files
```

**Practice:**

```bash
cd ~/ros2_ws/src
mkdir my_first_pkg
cd my_first_pkg
touch package.xml setup.py setup.cfg
ls                                   # Output: package.xml setup.cfg setup.py
```

**ROS 2 usage**: Create placeholder files for package manifests, launch files, configuration files.

---

### 3. `echo` - Print Text or Write to Files

Prints text to terminal or writes text to a file.

```bash
echo "Hello ROS 2"                   # Print to terminal
echo "This is content" > file.txt    # Write to file (overwrite)
```

**Practice:**

```bash
cd ~/ros2_ws
echo "# ROS 2 Workspace" > README.md
cat README.md                        # Output: # ROS 2 Workspace
```

**ROS 2 usage**: Create simple configuration files, document workspaces, scripting.

---

### 4. `cat` - Display File Contents

Displays the contents of one or more files.

```bash
cat filename.txt                     # Single file
cat file1.txt file2.txt              # Multiple files
```

**ROS 2 usage**: View log files, configuration files, package manifests without opening an editor.

---

### 5. `nano` - Terminal Text Editor

Opens a terminal-based text editor.

```bash
nano filename.txt
```

**Controls:**
- `Ctrl + O`: Save, `Enter`: Confirm
- `Ctrl + X`: Exit
- `Ctrl + K`: Cut line, `Ctrl + U`: Paste line
- `Ctrl + W`: Search, `Ctrl + \`: Replace

**Practice:**

```bash
cd ~/ros2_ws
nano README.md
# Add: This workspace contains ROS 2 packages for robotics development.
# Save (Ctrl+O, Enter) and exit (Ctrl+X)
cat README.md                        # Verify changes
```

**ROS 2 usage**: Edit configuration files on remote robots via SSH where graphical editors are unavailable.

---

### 6. `cp` - Copy Files and Directories

Duplicates files or directories.

```bash
cp source.txt destination.txt               # Copy file
cp file.txt /path/to/directory/             # Copy to directory
cp -r folder/ new_folder/                   # Copy directory (recursive)
cp file1.txt file2.txt /path/to/directory/  # Copy multiple files
```

**Practice:**

```bash
cd ~/ros2_ws
mkdir backup
cp README.md backup/
ls backup/                                  # Output: README.md
```

**ROS 2 usage**: Backup configuration files, duplicate package templates, create test copies.

---

### 7. `mv` - Move or Rename Files

Moves files/directories to a new location or renames them.

```bash
mv old_name.txt new_name.txt                # Rename file
mv file.txt /path/to/directory/             # Move to directory
mv folder/ /path/to/destination/            # Move directory
```

**Practice:**

```bash
cd ~/ros2_ws
mkdir config
touch params.yaml
mv params.yaml config/
ls config/                                  # Output: params.yaml
```

**ROS 2 usage**: Organize package structures, rename nodes, restructure workspaces.

---

### 8. `rm` - Remove Files (USE WITH CAUTION)

Deletes files and directories **permanently**. No recycle bin or undo.

```bash
rm file.txt                                 # Delete file
rm file1.txt file2.txt                      # Delete multiple files
rm -r folder/                               # Delete directory (recursive)
rm -f file.txt                              # Force delete without confirmation
rm -rf folder/                              # ⚠️ DANGEROUS: Force delete directory
```

**Warning**: `rm -rf` deletes everything without confirmation or recovery. One typo (e.g., `rm -rf / home/user` instead of `rm -rf /home/user`) can destroy the entire system.

**Safe practice:**

```bash
cd ~/ros2_ws
touch temp.txt
ls
rm temp.txt
ls
```

**ROS 2 usage**: Clean build artifacts (`rm -rf build/ install/ log/`), remove old packages, maintain workspace cleanliness.

---

### Understanding Paths

#### Absolute Path

Starts from root directory (`/`). Always unambiguous.

```bash
cd /home/username/ros2_ws
cat /opt/ros/jazzy/setup.bash
ls /usr/bin/python3
```

**Use when**: Scripting, uncertain about current directory, specifying paths in configuration files.

#### Relative Path

Relative to current directory.

```bash
cd ros2_ws              # Relative to current location
cd ../                  # Up one level
cd ./src                # Into src in current location
```

**Special symbols:**
- `.` = Current directory
- `..` = Parent directory
- `~` = Home directory (`/home/username`)

**Use when**: Working interactively or when paths should adapt to different user environments.

---

### Wildcards and Pattern Matching

Wildcards enable operations on multiple files matching a pattern.

| Pattern | Matches |
|---------|---------|
| `*` | Any characters (zero or more) |
| `?` | Exactly one character |
| `[abc]` | Any of: a, b, or c |
| `[0-9]` | Any digit |
| `[a-z]` | Any lowercase letter |

**Examples:**

```bash
ls *.txt                # List all .txt files
ls *.py                 # List all Python files
ls node_?.py            # Match node_1.py, node_2.py
rm *.log                # Delete all log files
cp *.yaml config/       # Copy all YAML files to config/
```

**ROS 2 usage**: Batch operations on launch files, clean build artifacts (`rm -rf build/* install/* log/*`), pattern-based searches.

---

## Permissions (Concept and Practice)

### What Are Permissions?

Every file and directory has three permission types:

- **Read (r)** — View contents
- **Write (w)** — Modify contents
- **Execute (x)** — Run as program (files) or enter (directories)

Permissions apply to three categories:

- **Owner (User)** — File creator
- **Group** — Users in the same group
- **Others** — All other users

### Viewing Permissions

```bash
ls -l
```

**Example output:**

```
-rw-r--r-- 1 username username 1234 Jan 15 10:30 file.txt
drwxr-xr-x 2 username username 4096 Jan 15 10:30 folder
```

**Permission string breakdown:**

```
-rw-r--r--
│└┬┘└┬┘└┬┘
│ │  │  └─ Others: r-- (read)
│ │  └─── Group: r-- (read)
│ └────── Owner: rw- (read, write)
└──────── Type: - (file), d (directory), l (link)
```

### Numeric Permission Notation

| Number | Permission | Symbolic |
|--------|------------|----------|
| 0 | None | `---` |
| 1 | Execute | `--x` |
| 2 | Write | `-w-` |
| 3 | Write + Execute | `-wx` |
| 4 | Read | `r--` |
| 5 | Read + Execute | `r-x` |
| 6 | Read + Write | `rw-` |
| 7 | Read + Write + Execute | `rwx` |

**Format:** `chmod 755 file.txt` → Owner: 7 (rwx), Group: 5 (r-x), Others: 5 (r-x)

**Common combinations:**

- `644` (`rw-r--r--`): Normal files (owner writes, others read)
- `755` (`rwxr-xr-x`): Executable scripts
- `700` (`rwx------`): Private files (owner only)
- `777` (`rwxrwxrwx`): Full permissions (security risk)

### `chmod` - Change File Permissions

**Symbolic mode:**

```bash
chmod u+x script.sh     # Add execute for owner
chmod +x script.sh      # Add execute for all
chmod g-w file.txt      # Remove write for group
chmod o+r file.txt      # Add read for others
chmod u=rwx file.txt    # Set exact permissions for owner
```

**Symbols:** `u` (user/owner), `g` (group), `o` (others), `a` (all), `+` (add), `-` (remove), `=` (set exact)

**Numeric mode:**

```bash
chmod 755 script.sh     # Executable script
chmod 444 file.txt      # Read-only for all
chmod 700 key.pem       # Private file
```

### Practice - Make a ROS 2 Script Executable

Create a simple ROS 2 node:

```bash
cd ~/ros2_ws
nano test_node.py
```

Add:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('Test node is running!')

def main():
    rclpy.init()
    node = TestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Save and exit. Check permissions:

```bash
ls -l test_node.py          # Output: -rw-r--r-- (NOT executable)
```

Make executable:

```bash
chmod +x test_node.py
ls -l test_node.py          # Output: -rwxr-xr-x (NOW executable)
```

Run:

```bash
source /opt/ros/jazzy/setup.bash
./test_node.py              # Or: python3 test_node.py
```

### Why Permissions Matter in Robotics

1. **Hardware access**: USB devices (`/dev/ttyUSB0`, `/dev/video0`) require specific permissions for ROS 2 nodes to access sensors and actuators
2. **Script execution**: ROS 2 launch files and node scripts need execute permission to run
3. **Security**: Production systems restrict access to control scripts and configuration files
4. **Multi-user systems**: Proper permissions prevent accidental modification of others' workspaces

---

## Package Management with APT

### What is APT?

APT (Advanced Package Tool) is the package manager for Ubuntu and Debian-based distributions. It installs, updates, and removes software.

**Why APT matters in ROS 2:**

- ROS 2 packages distributed as Debian packages (`ros-jazzy-*`)
- Automatic dependency resolution
- System-wide software management (Python libraries, build tools, drivers)
- Updates and security patches delivery

### Essential APT Commands

| Command | Purpose |
|---------|---------|
| `sudo apt update` | Refresh package lists from repositories |
| `sudo apt upgrade` | Upgrade all installed packages to latest versions |
| `sudo apt install <package>` | Install a new package |
| `sudo apt remove <package>` | Uninstall a package (keep configuration) |
| `sudo apt purge <package>` | Uninstall package and remove configuration files |
| `sudo apt autoremove` | Remove unused dependencies |
| `sudo apt search <keyword>` | Search for packages |
| `apt show <package>` | Show detailed package information |
| `apt list --installed` | List all installed packages |

### APT Workflow

Always start with update:

```bash
sudo apt update              # Refresh package lists (does NOT install/upgrade)
```

Then upgrade when needed:

```bash
sudo apt upgrade             # Upgrade all installed packages to latest versions
```

### Installing Packages

```bash
sudo apt install git                      # Single package
sudo apt install git curl wget            # Multiple packages
sudo apt install -y python3-pip           # Auto-confirm with -y
```

**Practice:**

```bash
sudo apt update
sudo apt install -y git curl wget python3-pip
git --version                             # Verify installation
```

### Installing ROS 2 Packages

ROS 2 packages follow naming convention: `ros-<distro>-<package-name>`

```bash
sudo apt install ros-jazzy-turtlesim        # ROS 2 demo
sudo apt install ros-jazzy-rviz2            # 3D visualization
sudo apt install ros-jazzy-nav2-bringup     # Navigation stack
sudo apt install ros-jazzy-xacro            # Robot description tools
sudo apt install ros-jazzy-gazebo-ros-pkgs  # Gazebo integration
```

Search packages:

```bash
apt search ros-jazzy-
apt search ros-jazzy-navigation
```

### Removing Packages

```bash
sudo apt remove package-name        # Uninstall (keep config)
sudo apt purge package-name         # Uninstall + remove config
sudo apt autoremove                 # Remove unused dependencies
```

### ROS 2 Package Installation (Required for Later Sessions)

The following commands install ROS 2 packages that will be used in future workshops. Install them now. They will be explained in later sessions.

#### Controllers and ros2_control

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-controller-manager -y
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

#### MoveIt 2

```bash
sudo apt install ros-jazzy-moveit -y
```

**No verification or explanation is required at this stage.** These packages will be used in later sessions.

---

## Understanding `sudo`

### What is `sudo`?

`sudo` (Superuser Do) executes commands with administrator (root) privileges.

**When `sudo` is required:**

- Installing/removing system-wide software
- Modifying system configuration files (`/etc/hosts`)
- Accessing restricted hardware (`/dev/ttyUSB0`)
- Starting/stopping system services
- Modifying file permissions on files you don't own

**Usage:**

```bash
sudo apt update
```

System prompts for your password. **Password is not displayed** (normal security behavior).

### Why `sudo` Exists

Linux separates privileges to prevent:

- Accidental system damage
- Unauthorized modifications by malicious software
- Users modifying others' files

### When NOT to Use `sudo`

**Do NOT use for:**

- Creating files in home directory (`~/`)
- Editing your own files
- Running ROS 2 nodes (unless accessing hardware)
- Building ROS 2 packages (`colcon build`)

**Problems from unnecessary `sudo`:**

- Files owned by root, difficult to edit later
- ROS 2 workspace permission issues
- Unnecessary security risks

### Check Ownership Before Using `sudo`

```bash
ls -l /opt/ros/jazzy/setup.bash     # Owner: root (needs sudo)
ls -l ~/ros2_ws/README.md           # Owner: username (no sudo needed)
```

---

## ROS 2 Environment Verification

### Verify ROS 2 Installation

```bash
ros2 --version              # Output: ros2 cli version 0.x.x
echo $ROS_DISTRO            # Output: jazzy
```

If `$ROS_DISTRO` is empty, source ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
echo $ROS_DISTRO
```

### Check Environment Variables

```bash
echo $ROS_VERSION           # Output: 2
echo $ROS_PYTHON_VERSION    # Output: 3
echo $ROS_LOCALHOST_ONLY    # Output: 0
```

### List Available ROS 2 Commands

```bash
ros2 --help
```

**Common ROS 2 commands:**

| Command | Purpose |
|---------|---------|
| `ros2 run` | Run a node from a package |
| `ros2 launch` | Launch multiple nodes from a launch file |
| `ros2 node list` | List active nodes |
| `ros2 node info` | Show information about a node |
| `ros2 topic list` | List active topics |
| `ros2 topic echo` | Display messages published to a topic |
| `ros2 topic pub` | Publish messages to a topic |
| `ros2 pkg list` | List installed ROS 2 packages |
| `ros2 pkg create` | Create a new ROS 2 package |

### Testing ROS 2 with Turtlesim

Turtlesim is the standard "Hello World" example for ROS 2.

**Install and run:**

```bash
sudo apt install ros-jazzy-turtlesim -y
ros2 run turtlesim turtlesim_node        # Window appears with turtle
```

**Open new terminal** (`Ctrl + Shift + T`), source ROS 2, and run teleop:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run turtlesim turtle_teleop_key     # Use arrow keys to control turtle
```

### Inspecting the ROS 2 System

Open third terminal and source ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

**List nodes and topics:**

```bash
ros2 node list              # Output: /turtlesim, /teleop_turtle
ros2 topic list             # Output: /turtle1/cmd_vel, /turtle1/pose, etc.
```

**Echo topic for real-time data:**

```bash
ros2 topic echo /turtle1/pose
```

**Output (updates as turtle moves):**

```
x: 5.544444561004639
y: 5.544444561004639
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
---
```

**Echo velocity commands:**

```bash
ros2 topic echo /turtle1/cmd_vel         # Shows velocity commands as you move turtle
```

Stop all terminals with `Ctrl + C`.

---

## Progressive Task System

This course uses three task types, each building on skills from previous sections.

### Task Type 1: Demonstration Task

Fully demonstrated tasks. You observe without executing during demonstration. Used for introducing new concepts or complex procedures.

### Task Type 2: Practice Task

Tasks you perform using only previously explained commands. Used for reinforcing individual skills immediately after instruction.

### Task Type 3: Final Comprehensive Task

Combines all session skills. Submitted for evaluation. Assesses cumulative learning.

### Task Progression

1. Section teaches concept (e.g., file operations)
2. Practice task reinforces concept (e.g., create and copy files)
3. Later tasks incorporate concept cumulatively (e.g., workspace structure + file management + permissions)

All tasks referenced in [tasks/](tasks/) directory.

---

## Practice Tasks

### Practice Task 1: Terminal Navigation

**Objective**: Navigate file system, list contents, verify location.

**Skills**: `cd`, `ls`, `pwd`

```bash
cd ~
ls -la
mkdir ros2_practice
cd ros2_practice
pwd
cd ~
```

---

### Practice Task 2: File Operations

**Objective**: Create, write, copy, move, delete files.

**Skills**: `mkdir`, `touch`, `echo`, `cat`, `cp`, `mv`, `rm`

```bash
cd ~/ros2_practice
touch system_info.txt
echo "Ubuntu 24.04 LTS with ROS 2 Jazzy" > system_info.txt
cat system_info.txt
cp system_info.txt system_info_backup.txt
mv system_info_backup.txt backup.txt
rm backup.txt
ls
```

---

### Practice Task 3: Permissions

**Objective**: Create script, make executable, run.

**Skills**: `nano`, `chmod`, file permissions

```bash
cd ~/ros2_practice
touch hello.sh
nano hello.sh
# Add:
#!/bin/bash
echo "Hello from ROS 2 Workshop!"
# Save (Ctrl+O, Enter) and exit (Ctrl+X)
ls -l hello.sh
chmod +x hello.sh
ls -l hello.sh
./hello.sh
```

---

### Practice Task 4: Package Management

**Objective**: Install package and verify.

**Skills**: `apt update`, `apt install`, verification

```bash
sudo apt update
sudo apt install -y tree
tree --version
cd ~
tree ros2_practice
```

---

## Final Comprehensive Task

Create a complete ROS 2 workspace structure, populate with files, document setup, and demonstrate all skills learned.

See [tasks/task.md](tasks/task.md) for full requirements and submission instructions.

---

## Resources

### Official Documentation

- [ROS 2 Official Documentation](https://docs.ros.org/en/jazzy/)
- [ROS 2 Installation Guide (Ubuntu)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [VS Code Linux Setup](https://code.visualstudio.com/docs/setup/linux)
- [Ubuntu Documentation](https://help.ubuntu.com/)

### Learning Resources

- [Linux Journey](https://linuxjourney.com/)
- [The Linux Command Line (free book)](http://linuxcommand.org/tlcl.php)
- [ROS 2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)

### Cheat Sheets

- [Linux Cheat Sheet (PDF)](https://drive.google.com/file/d/1vddIJlermV-xZ7TZ8eSUCvzHKv_Poj2A/view)
- [ROS 2 Cheat Sheet (PDF)](https://drive.google.com/file/d/1vlimFy3CSk26AfZjB73rtG6GKqPZVUT9/view)

### Community

- [ROS Discourse](https://discourse.ros.org/)
- [ROS Answers](https://answers.ros.org/)

---

## Quick Reference

### Essential Linux Commands

```bash
pwd                       # Print working directory
ls                        # List files
ls -la                    # List all files with details
cd [directory]            # Change directory
cd ~                      # Go to home directory
cd ..                     # Go up one level
mkdir [name]              # Create directory
mkdir -p [path]           # Create nested directories
touch [file]              # Create empty file
echo "text" > [file]      # Write text to file
cat [file]                # Display file contents
nano [file]               # Edit file
cp [source] [dest]        # Copy file
cp -r [source] [dest]     # Copy directory
mv [source] [dest]        # Move or rename
rm [file]                 # Delete file
rm -r [directory]         # Delete directory
chmod +x [file]           # Make file executable
chmod 755 [file]          # Set permissions (rwxr-xr-x)
```

### Essential APT Commands

```bash
sudo apt update           # Update package lists
sudo apt upgrade          # Upgrade installed packages
sudo apt install [pkg]    # Install package
sudo apt remove [pkg]     # Uninstall package
sudo apt autoremove       # Remove unused dependencies
apt search [keyword]      # Search for packages
apt show [pkg]            # Show package information
apt list --installed      # List installed packages
```

### Essential ROS 2 Commands

```bash
source /opt/ros/jazzy/setup.bash    # Source ROS 2 environment
ros2 --version                       # Check ROS 2 version
ros2 run [pkg] [node]                # Run a node
ros2 launch [pkg] [launch]           # Launch nodes from file
ros2 node list                       # List active nodes
ros2 node info [node]                # Show node information
ros2 topic list                      # List active topics
ros2 topic echo [topic]              # Display topic messages
ros2 topic pub [topic] [msg] [data]  # Publish to topic
ros2 pkg list                        # List installed packages
ros2 pkg create [name]               # Create new package
```

---

## Common Issues and Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| `ros2: command not found` | ROS 2 environment not sourced | Run `source /opt/ros/jazzy/setup.bash` |
| `sudo` asks for password repeatedly | Incorrect password | Re-enter correct user password |
| `apt update` fails | No network connection | Check network connectivity |
| VS Code won't launch | Installation incomplete | Run `sudo apt install ./code_*.deb -y` again |
| `tree` command not found | Package not installed | Run `sudo apt install -y tree` |
| Permission denied when running script | No execute permission | Run `chmod +x script.sh` |
| File not found error | Incorrect path or typo | Verify path with `ls` and `pwd` |
| Turtlesim window doesn't appear | Package not installed | Run `sudo apt install ros-jazzy-turtlesim` |

---

## Workshop Tips

- **Tab completion**: Autocomplete file/directory names to prevent typos
- **Check location**: Run `pwd` to confirm correct directory before executing commands
- **Verify output**: Check command output to ensure operation succeeded
- **Read errors**: Error messages explain what went wrong and how to fix it
- **Practice regularly**: Terminal skills improve with consistent practice

---

## Key Skills Summary

Upon completion, you will be able to:

- Navigate file system (`cd`, `ls`, `pwd`)
- Create and organize files/directories (`touch`, `mkdir`, `cp`, `mv`)
- Edit files (`nano`)
- Manage file permissions (`chmod`)
- Install and manage packages (APT)
- Configure VS Code for ROS 2
- Source ROS 2 environment via `.bashrc`
- Verify ROS 2 installation and run basic commands
- Inspect nodes and topics using `ros2` CLI tools

---

## What's Next

**Next session topics:**

- ROS 2 concepts: nodes, topics, messages, services
- Creating your first ROS 2 package
- Writing publisher and subscriber nodes
- Understanding ROS 2 computational graph
- Launching multiple nodes with launch files

**Prerequisites:**

- All tasks in this session completed
- VS Code configured with extensions
- ROS 2 environment sourced automatically
- Turtlesim successfully tested

---

## Tasks Index

All hands-on tasks are located in the [tasks/](tasks/) directory:

- **[tasks/task.md](tasks/task.md)** - Final comprehensive task with submission requirements

---

**End of Session**
