# Ubuntu 24.04 VirtualBox Installation Guide for ROS 2 Jazzy

---

## Phase 0 — Pre-Installation Checks (MANDATORY)

**⚠️ CRITICAL:** This phase MUST be completed before installing VirtualBox or Ubuntu. Skipping any step here is the main reason students face crashes, freezes, or missing options later.

---

### 0.1 Purpose of This Phase

This phase ensures that the host machine (Windows) is ready for virtualization.

**By the end of Phase 0:**
- VirtualBox will be able to run 64-bit Linux
- Ubuntu will not freeze randomly
- Disk and memory issues are prevented early
- Common hidden Windows conflicts are eliminated

---

### 0.2 Minimum Hardware Requirements (Host Machine)

Each student must verify the following:

#### CPU
- **64-bit processor**
- **Supports virtualization:**
  - Intel VT-x, or
  - AMD-V

> Almost all modern CPUs support this, but it may be disabled in BIOS.

#### RAM
- **Minimum:** 8 GB (works but not ideal)
- **Recommended:** 16 GB

> Below 8 GB → frequent freezes inside the VM.

#### Disk Space
- **Minimum free space:** 120 GB
- **Recommended:** 150 GB or more

**⚠️ Important:**
- VirtualBox disks are dynamic, but they still consume real disk space
- We previously encountered:
```
  VERR_DISK_FULL
  BLKCACHE_IOERR
```
  because host disk space was insufficient

---

### 0.3 Enable Virtualization in BIOS (CRITICAL)

#### Why this matters
If virtualization is disabled:
- VirtualBox will not show "Ubuntu (64-bit)"
- VM may not start
- Performance will be extremely poor

#### How to check (Quick Test in Windows)
1. Press `Ctrl + Shift + Esc` → open Task Manager
2. Go to **Performance** tab
3. Select **CPU**
4. Look for:
```
   Virtualization: Enabled
```

> If it says **Disabled** → BIOS configuration is required.

#### Enabling Virtualization in BIOS (General Steps)

BIOS layout differs by manufacturer, but the logic is the same.

1. Restart the computer
2. Enter BIOS (commonly: `F2`, `DEL`, `F10`, or `ESC`)
3. Navigate to:
   - **Advanced**
   - **Advanced BIOS Features**
   - **Advanced CPU Configuration**
4. Enable:
   - **Intel Virtualization Technology**, or
   - **SVM Mode** (AMD)
5. **Save & Exit**

**⚠️ This step is non-negotiable.**

---

### 0.4 Disable Windows Features That Conflict With VirtualBox

This is one of the most common hidden problems.

#### 0.4.1 Disable Hyper-V

VirtualBox cannot coexist properly with Hyper-V.

**Steps:**
1. Press `Win + R`
2. Type:
```
   optionalfeatures
```
3. Press Enter
4. Uncheck:
   - Hyper-V
   - Windows Hypervisor Platform
   - Virtual Machine Platform
5. Click **OK**
6. **Restart Windows**

#### 0.4.2 Disable Windows Subsystem for Linux (WSL)

WSL uses the same hypervisor backend.

**Steps:**
- In the same Windows Features window:
  - Uncheck **Windows Subsystem for Linux**
- **Restart Windows**

#### 0.4.3 Disable Core Isolation (Memory Integrity)

This one silently breaks VirtualBox performance.

**Steps:**
1. Open **Windows Security**
2. Go to **Device Security**
3. Click **Core Isolation Details**
4. Turn **Memory Integrity OFF**
5. **Restart Windows**

---

### 0.5 Power & Performance Settings (Recommended)

#### Set Power Mode to High Performance

1. Open **Control Panel**
2. Go to **Power Options**
3. Select:
```
   High performance
```

> This prevents CPU throttling inside the VM.

---

### 0.6 Antivirus & Firewall Notes

- **Windows Defender** is fine
- **Do NOT install:**
  - Third-party aggressive antivirus
  - "Performance booster" tools

They often block:
- VirtualBox drivers
- Disk access
- Network interfaces

---

### 0.7 Final Phase 0 Checklist (Student Must Confirm)

Before moving to Phase 1, the student must confirm:

- ✔ Virtualization enabled (Task Manager shows "Enabled")
- ✔ Hyper-V disabled
- ✔ WSL disabled
- ✔ Memory Integrity disabled
- ✔ At least 120 GB free disk space
- ✔ At least 8 GB RAM available

---

## Phase 1 — Installing Oracle VirtualBox (Host Setup)

**Goal of this phase:** Install VirtualBox correctly on Windows so that:
- Ubuntu 24.04 works without crashes
- 64-bit guests are available
- No hidden conflicts exist

---

### 1.1 Download Oracle VirtualBox (Correct Source Only)

#### What the student must do

1. Open a web browser
2. Go to the official VirtualBox downloads page:
```
   https://www.virtualbox.org/wiki/Downloads
```

#### What to download

Under **VirtualBox platform packages**, click:
```
Windows hosts
```

This downloads a file similar to:
```
VirtualBox-7.2.x-xxxx-Win.exe
```

#### ⚠️ Important rules

- **Do NOT** download from third-party websites
- **Do NOT** use older versions
- All students must use the same major version (7.x)

---

### 1.2 Start the VirtualBox Installer

1. Double-click the downloaded `.exe` file
2. If Windows asks for permission:
   - Click **Yes**

You should now see the **Oracle VM VirtualBox Setup Wizard**.

---

### 1.3 Welcome Screen

#### What you see
A welcome screen with a **Next** button.

#### What to do
- Click **Next**

📌 No configuration happens here.

---

### 1.4 Custom Setup Screen (VERY IMPORTANT)

This screen shows which components will be installed.

#### Leave ALL of the following enabled:

- ✔ Oracle VM VirtualBox Application
- ✔ USB Support
- ✔ Networking
- ✔ Python Support

**Do NOT remove anything.**

#### Why?
- USB support is needed later (even if not now)
- Networking creates a virtual adapter required for internet access
- Python support avoids silent runtime issues

**Click Next**

---

### 1.5 Network Interface Warning

#### What you see
A warning that:
```
Network connections will be reset temporarily
```

#### What to do
- Click **Yes**

#### Why this is safe
- VirtualBox creates a virtual network adapter
- Internet may disconnect for a few seconds
- This is expected and harmless

---

### 1.6 Ready to Install Screen

#### What you see
A summary of selected components.

#### What to do
- Click **Install**

At this stage:
- Windows may ask to install drivers
- Click **Install** or **Allow** every time

**❌ Never click "Cancel" here.**

---

### 1.7 Installation Process

- Wait until installation finishes
- This may take 1–2 minutes

**Do NOT open other programs during installation.**

---

### 1.8 Finish Screen

#### What to do
- Ensure **Start Oracle VM VirtualBox after installation** is checked
- Click **Finish**

VirtualBox Manager should open automatically.

---

### 1.9 Install VirtualBox Extension Pack (MANDATORY)

**This step is NOT optional.** Missing Extension Pack caused multiple issues in previous attempts.

#### Step-by-step

1. Go back to the same download page:
```
   https://www.virtualbox.org/wiki/Downloads
```

2. Download:
```
   VirtualBox Extension Pack
```
   (MUST match the exact VirtualBox version)

3. Double-click the downloaded `.vbox-extpack` file

4. VirtualBox opens and asks to install:
   - Click **Install**
   - Accept the license agreement

#### What this enables
- USB 2.0 / 3.0 support
- Better device handling
- Improved VM stability

#### Without it:
- Random freezes
- USB failures
- Display bugs

---

### 1.10 Verify VirtualBox Installation

Student must verify:
- ✔ VirtualBox Manager opens without errors
- ✔ No warning messages appear
- ✔ Version number matches Extension Pack version

---

## Phase 2 — Creating the Virtual Machine (Correct & Safe Way)

**Goal of this phase:** Create the virtual machine with the exact settings that avoid:
- Missing 64-bit Ubuntu
- Installer crashes
- Disk full errors later
- Display / graphics freezes

---

### 2.1 Download Ubuntu ISO (Verification Step)

Before creating the VM, the student must confirm:

- File name similar to:
```
  ubuntu-24.04.3-desktop-amd64.iso
```

- File size ≈ 5–6 GB

- File downloaded from:
```
  https://ubuntu.com/download/desktop
```

#### ❌ Do NOT use:
- Ubuntu Server
- Ubuntu 22.xx
- Any non-official mirror

---

### 2.2 Open VirtualBox Manager

- Launch **Oracle VM VirtualBox**
- You should see the main manager window
- No VMs are running

---

### 2.3 Create a New Virtual Machine

Click the **New** button.

You will now see the **Create Virtual Machine** screen.

---

### 2.4 Name and Operating System (CRITICAL)

#### Virtual Machine Name

Enter:
```
Ubuntu-ROS2-Jazzy
```

**Why this matters:**
- Avoids overwriting old VMs
- Easy identification later
- Prevents "Can't overwrite machine folder" errors

#### Machine Folder

Leave the default location:
```
C:\Users\<YourUsername>\VirtualBox VMs
```

**❌ Do NOT place the VM on:**
- External drives
- USB sticks
- Nearly full partitions

#### ISO Image

Click **Browse** and select:
```
ubuntu-24.04.3-desktop-amd64.iso
```

After selecting the ISO:
- VirtualBox should automatically detect:
  - **Type:** Linux
  - **Distribution:** Ubuntu
  - **Version:** Ubuntu (64-bit)

**If Ubuntu (64-bit) does NOT appear → STOP and return to Phase 0 (virtualization not active).**

---

### 2.5 Unattended Installation (MUST BE DISABLED)

You will see an option similar to:
```
Proceed with Unattended Installation
```

**Action:**
```
❌ Disable / Uncheck this option
```

#### Why we disable it:
- It hides installer steps
- Creates users automatically
- Caused broken installs and confusion earlier
- Makes debugging impossible

**We want a manual, visible install.**

**Click Next**

---

### 2.6 Hardware Configuration (RAM & CPU)

#### Memory (RAM)

Set:
```
8192 MB (8 GB)
```

**Rules:**
- Do NOT go below 8 GB
- Do NOT exceed 60% of host RAM

**Why:**
- 4 GB caused freezes during updates
- 8 GB is stable for Ubuntu 24.04

#### CPUs

Set:
```
4 CPUs
```

**Rules:**
- Stay inside the green zone
- Never use all host cores

**Why:**
- 1–2 CPUs caused slow boot and hangs
- 4 CPUs is stable and responsive

**Click Next**

---

### 2.7 Boot Mode (EFI) — IMPORTANT DECISION

You will see an option:
```
Use EFI (special OSes only)
```

**Action:**
```
❌ DO NOT enable EFI
```

**Why:**
- EFI caused boot loops and black screens
- Ubuntu installs cleanly without EFI in VirtualBox
- Legacy BIOS is more stable for students

---

### 2.8 Virtual Hard Disk Configuration (THIS CAUSED CRASHES BEFORE)

#### Disk Option

Choose:
```
Create a new virtual hard disk
```

#### Disk Type

Select:
```
VDI (VirtualBox Disk Image)
```

#### Allocation Method

Select:
```
Dynamically allocated
```

**Why:**
- Uses space only when needed
- Avoids large upfront disk usage

#### Disk Size (VERY IMPORTANT)

Set:
```
100–120 GB
```

Minimum:
```
80 GB (NOT recommended)
```

**Why this matters:**
- We previously encountered:
```
  VERR_DISK_FULL
  BLKCACHE_IOERR
```

- Disk size below 80 GB caused system crashes during `apt upgrade`

**Click Finish**

---

### 2.9 DO NOT START THE VM YET

At this point:
- ✔ The VM exists
- ✔ Ubuntu is NOT installed yet
- ✔ We must configure graphics settings first

**Starting the VM now caused:**
- Display freezes
- Installer crashes

---

## Phase 3 — Display & Graphics Configuration (CRITICAL)

**Goal of this phase:** Configure graphics before the first boot to avoid:
- Black screen after install
- Freezes during `apt upgrade`
- Laggy UI
- Random VM crashes

---

### 3.1 Why This Phase Is Critical

In our earlier attempts, most failures were caused by:
- Wrong Graphics Controller
- Low Video Memory
- Enabling 3D Acceleration too early

**Ubuntu 24.04 + GNOME is graphics-sensitive, especially inside a VM.**

---

### 3.2 Open VM Settings (DO NOT START THE VM)

1. In **VirtualBox Manager**
2. Select the VM:
```
   Ubuntu-ROS2-Jazzy
```
3. Click **Settings**

---

### 3.3 Display → Screen (Main Configuration)

Navigate to:
```
Settings → Display → Screen
```

You will now see multiple graphics-related options.

---

### 3.4 Graphics Controller (MOST IMPORTANT OPTION)

Set:
```
Graphics Controller: VMSVGA
```

#### DO NOT use:
- **VBoxVGA** ❌ (deprecated, causes black screens)
- **VBoxSVGA** ❌ (unstable with Ubuntu 24.04)

#### Why VMSVGA?
- Best compatibility with modern Linux kernels
- Stable with GNOME
- Fewer freezes during updates

**This single option fixed multiple crashes we experienced earlier.**

---

### 3.5 Video Memory (MUST BE MAXED)

Set:
```
Video Memory: 128 MB
```

#### Rules:
- Always use the maximum value
- Never leave it at default (16–32 MB)

#### Why:
Low video memory caused:
- UI freezing
- Installer lag
- Black screens after login

---

### 3.6 Number of Monitors

Set:
```
1 Monitor
```

**Do NOT increase this.** Multiple monitors increase GPU load unnecessarily.

---

### 3.7 Scale Factor (Leave Default)

Leave:
```
Scale Factor: 100%
```

We handle scaling inside the VM later, not here.

---

### 3.8 3D Acceleration (IMPORTANT DECISION)

Set:
```
❌ Disable 3D Acceleration
```

#### Why we disable it initially:
- Caused VM crashes during:
  - Ubuntu installation
  - `apt upgrade`
- VirtualBox 3D acceleration is unstable on many Windows GPUs

#### ⚠️ Rule:
We only enable 3D acceleration later if the system is stable.

---

### 3.9 Recording Tab (Optional Check)

Navigate to:
```
Display → Recording
```

Ensure:
```
❌ Recording is disabled
```

Recording increases GPU usage and can cause lag.

---

### 3.10 Apply Settings

Click:
```
OK
```

**Do NOT start the VM yet if:**
- Any setting above is incorrect

---

## Phase 4 — Installing Ubuntu 24.04 (Manual Installation, Screen by Screen)

**Goal of this phase:** Install Ubuntu 24.04 manually and visibly, so the student:
- Understands every installer choice
- Avoids broken installs
- Ends with a clean, stable system

---

### 4.1 Start the Virtual Machine (First Boot)

1. Open **VirtualBox Manager**
2. Select:
```
   Ubuntu-ROS2-Jazzy
```
3. Click **Start**

#### What should happen:
- VM boots from the Ubuntu ISO
- Ubuntu splash screen appears

#### If the VM does not boot:
- **STOP**
- Do NOT retry
- Re-check Phase 2 & Phase 3

---

### 4.2 Ubuntu Welcome Screen

You will see two options:
- Try Ubuntu
- Install Ubuntu

**Action:**
```
Select → Install Ubuntu
```

#### Why:
- We are installing permanently to the virtual disk
- "Try Ubuntu" runs a temporary live session only

**Click Continue**

---

### 4.3 Keyboard Layout

Screen shows:
- Keyboard layout selection

**Choose:**
- Language: **English**
- Layout: **English (US)**

**Click Continue**

📌 **Reason:**
- Consistent key bindings
- Avoids issues with terminal commands later

---

### 4.4 Updates & Other Software (VERY IMPORTANT)

This screen caused confusion before, so we lock it clearly.

#### Choose:
```
✔ Normal installation
```

This includes:
- Web browser
- System utilities
- GNOME desktop tools

#### Updates Section

Enable:
```
✔ Download updates while installing Ubuntu
```

**Why:**
- Reduces number of updates needed after installation
- Saves time later

#### Third-Party Software

Enable:
```
✔ Install third-party software for graphics and Wi-Fi hardware
```

**Why:**
- Improves display and media support
- Safe inside a VM
- Prevents missing codec issues

**Click Continue**

---

### 4.5 Installation Type — Disk Partitioning (CRITICAL)

This screen is where many students panic.

You will see:
```
Erase disk and install Ubuntu
```

**Action:**
```
Select → Erase disk and install Ubuntu
```

#### ⚠️ VERY IMPORTANT CLARIFICATION:
- This erases ONLY the virtual disk
- It does NOT touch Windows
- It does NOT affect the host computer

#### Why we choose this:
- Automatic partitioning is safer
- Manual partitioning introduces errors
- This avoids filesystem and boot issues

**Click Install Now**

#### Confirmation Dialog

Ubuntu will warn that changes will be written to disk.

**Action:**
```
Click → Continue
```

---

### 4.6 Time Zone Selection

- Select your city or region
- Map will auto-adjust

**Click Continue**

---

### 4.7 User Account Creation

Fill in the following:
- **Your name:** Student's name
- **Computer name:** Leave default or simple name
- **Username:** Lowercase, no spaces
- **Password:** Must be remembered

#### Login Option:
```
✔ Require my password to log in
```

**Why:**
- Better security
- Avoids auto-login bugs

**Click Continue**

---

### 4.8 Installation Process

Ubuntu now installs the system.

#### What happens:
- Files are copied
- Packages are installed
- Updates are applied

⏳ **This takes 10–20 minutes depending on hardware.**

#### Do NOT:
- Close the VM
- Pause the VM
- Resize aggressively

---

### 4.9 Installation Complete → Restart

When prompted:
```
Installation complete
Restart Now
```

**Click Restart Now**

---

### 4.10 "Remove Installation Medium" Message

You may see:
```
Please remove the installation medium, then press ENTER
```

#### Important:
- VirtualBox automatically removes the ISO
- Do NOT manually remove anything

**Action:**
```
Press ENTER
```

The VM will reboot into Ubuntu.

---

## Phase 5 — First Login, System Update & Freeze Handling (CRITICAL)

**Goal of this phase:** Make the system stable after first login, apply updates safely, and teach students exactly what to do if the system hangs—بدون هلع ولا إعادة تثبيت (without panic or reinstallation).

---

### 5.1 First Login (What to Expect)

After reboot, you will see the Ubuntu login screen.

#### What the student should do

1. Select the created user
2. Enter the password
3. Log in

#### What is normal at this stage

- Slower response than expected
- Fans spin up
- Desktop takes extra seconds to load

📌 **Reason:** Ubuntu 24.04 (GNOME) is initializing caches and services for the first time.

**Do NOT restart unless completely frozen for more than 5 minutes.**

---

### 5.2 Initial Desktop Check (Before Any Updates)

Before opening Terminal, verify:

- ✔ Desktop loads correctly
- ✔ Mouse and keyboard respond
- ✔ System Settings opens
- ✔ No black screen

#### If you see:
- Minor lag → **normal**
- Short freezes (1–2 seconds) → **normal**

#### If the screen is completely black:
- **STOP**
- Re-check Phase 3 (graphics settings)

---

### 5.3 Open Terminal

Open Terminal using:
- `Ctrl + Alt + T`, or
- **Applications → Terminal**

All system updates will be done only from Terminal.

---

### 5.4 First System Update (Safe Order)

#### Step 1 — Update Package Lists

Run:
```bash
sudo apt update
```

**What this does:**
- Refreshes package repositories
- Does NOT install anything yet

**Expected behavior:**
- Takes 10–60 seconds
- Shows list of repositories
- Ends without errors

**If errors appear:**
- Check internet connection
- Do NOT continue yet

#### Step 2 — Upgrade Packages (This Is Where Issues Happened)

Run:
```bash
sudo apt upgrade -y
```

**This step:**
- Installs kernel updates
- Updates GNOME components
- Updates system libraries

⏳ **This may take several minutes.**

---

### 5.5 What Is NORMAL During `apt upgrade`

Students often panic here—so we define normal behavior clearly.

#### Normal:
- Terminal pauses for 10–30 seconds
- CPU usage spikes
- Disk activity increases
- Fans spin up

**Do NOT close the VM during this.**

---

### 5.6 What To Do If the System Appears Frozen

#### Case 1 — Terminal paused but VM still responsive

✔ **Wait** - This is normal.

#### Case 2 — VM UI lags but mouse still moves

✔ **Wait at least 3–5 minutes**

This happened in our earlier tests and resolved itself.

#### Case 3 — VM completely unresponsive for >5 minutes

This is when students must act correctly.

**Correct action:**

1. From VirtualBox menu:
```
   Machine → Restart
```

2. Do NOT use **Power Off** unless restart fails

**Why:**
- Restart is a clean reset
- Power Off risks filesystem corruption

---

### 5.7 After Restart — Fixing Interrupted Updates (VERY IMPORTANT)

If the update was interrupted, Ubuntu may show warnings like:
```
dpkg was interrupted
```

#### Correct fix (DO NOT repeat `apt upgrade` yet):

Run:
```bash
sudo dpkg --configure -a
```

**What this does:**
- Completes unfinished package installations
- Fixes broken states safely

Wait until it finishes.

Then run:
```bash
sudo apt update
sudo apt upgrade -y
```

This second run usually completes cleanly.

---

### 5.8 Why We Had Crashes Before (Important Explanation for Students)

Previously, crashes occurred because:
- Disk size was too small
- Video memory was low
- Wrong graphics controller was used
- Students force-closed the VM during upgrades

#### Now that we have:
- ✔ ≥100 GB disk
- ✔ VMSVGA graphics
- ✔ 128 MB video memory

**These issues no longer occur.**

---

### 5.9 Reboot After Updates (MANDATORY)

After `apt upgrade` finishes successfully:
```bash
sudo reboot
```

#### Why:
- Kernel updates require reboot
- Ensures system stability
- Prevents hidden issues later

---

## Phase 6 — Installing VirtualBox Guest Additions

---

### 6.1 What Are Guest Additions? (Explain to Students)

VirtualBox Guest Additions are drivers installed inside Ubuntu that allow:
- Dynamic screen resizing
- Proper graphics acceleration (safe mode)
- Clipboard sharing
- Mouse integration
- Better overall performance

#### Without Guest Additions:
- Screen stays small
- Scaling is blurry
- Copy/paste does not work
- VM feels "broken"

---

### 6.2 Insert Guest Additions CD (From VirtualBox Menu)

#### Action (VERY IMPORTANT ORDER)

1. Make sure the VM is running
2. From the VirtualBox menu bar (top of the VM window):
```
   Devices → Insert Guest Additions CD Image
```

#### What happens:
- A virtual CD is mounted inside Ubuntu
- Ubuntu may show a popup

#### If Ubuntu asks:
"Do you want to run the software?"

**❌ Do NOT click Run yet**

We install prerequisites first.

---

### 6.3 Install Required Build Tools (MANDATORY)

Open Terminal and run:
```bash
sudo apt install -y build-essential dkms linux-headers-$(uname -r)
```

#### Why this is required:
- Guest Additions include kernel modules
- These modules must be compiled for the running kernel
- Missing headers = installation failure

**Wait until this command finishes completely.**

---

### 6.4 Run Guest Additions Installer

#### Option A — Using the Popup (If Appears)

If Ubuntu shows:
"VBox_GAs_… contains software. Do you want to run it?"

Then:
1. Click **Run**
2. Enter your password
3. Wait for the installation to finish

#### Option B — Manual Installation (Preferred for Teaching)

If no popup appears, do it manually:
```bash
sudo mount /dev/cdrom /mnt
sudo /mnt/VBoxLinuxAdditions.run
```

During installation you will see:
- Kernel module compilation
- Messages scrolling in the terminal

**This is normal.**

Wait until you see no errors.

---

### 6.5 Reboot the VM (MANDATORY)

After installation finishes:
```bash
sudo reboot
```

**Do NOT skip this step.**

---

### 6.6 Verify Guest Additions Are Working

After reboot, verify the following:

#### Auto Resize Display

From VM menu:
```
View → Auto-resize Guest Display
```

Resize the VM window.

- ✔ Screen should resize automatically
- ✔ No black borders
- ✔ No blur

#### Full Screen Mode
```
View → Full Screen Mode
Host + F
```

(Default Host key = Right Ctrl)

- ✔ Screen fits perfectly
- ✔ No distortion

---

### 6.7 Display Scaling (Correct Way)

Inside Ubuntu:
1. Open **Settings**
2. Go to **Displays**
3. Set:
```
   Scale: 125% or 150%
```
   (Choose what feels comfortable)

- **❌ Do NOT use VirtualBox scaling as primary method**
- **✔ Always scale from Ubuntu settings**

---

### 6.8 Enable Clipboard Sharing (CRITICAL)

#### VM Settings (VM must be OFF for this)

1. Power off the VM
2. VirtualBox Manager → Select VM
3. Click **Settings → General → Advanced**

Set:
```
Shared Clipboard: Bidirectional
Drag'n'Drop: Bidirectional
```

Click **OK**

Start the VM again.

#### Verify Clipboard

Test:
- Copy text from Windows
- Paste into Ubuntu Terminal
- Copy from Ubuntu → paste into Windows

**✔ Should work both ways**

#### If it does not:
- Reboot once
- Do NOT reinstall Guest Additions

---

### 6.9 Shared Folder (Optional but Recommended)

This allows file transfer without copy–paste.

#### Setup

1. Power off the VM
2. **VM Settings → Shared Folders**
3. Add new folder:
   - **Folder Path:** any Windows folder
   - **Folder Name:** `shared`
   - ✔ **Auto-mount**
   - ✔ **Make Permanent**

Start the VM.

#### Access in Ubuntu
```bash
ls /media/
```

You should see a folder like:
```
sf_shared
```

#### If permission is denied:
```bash
sudo usermod -aG vboxsf $USER
sudo reboot
```

---

### 6.10 Common Problems & Fixes (Based on What We Saw)

#### Screen Still Small
- Guest Additions not installed correctly
- Re-run installer
- Reboot

#### Copy/Paste Not Working
- Clipboard not set to Bidirectional
- VM not rebooted after settings change

#### VM Feels Laggy After Guest Additions
- This is normal for first boot
- Improves after 1–2 minutes

---

## Phase 7 — Installing Essential Developer Tools

---

### 7.1 Why We Install These Tools Now

At this point:
- Ubuntu is stable
- Display and input work correctly
- Updates are complete

Installing developer tools before ROS avoids:
- Dependency conflicts
- Broken builds
- Missing compilers later

---

### 7.2 Install Visual Studio Code (MANDATORY)

#### Why VS Code?
- Industry standard
- Lightweight
- Excellent Linux support
- Required later for debugging and extensions

#### Method A — Ubuntu Software (Recommended for Students)

1. Open **Ubuntu Software**
2. Search for:
```
   Visual Studio Code
```
3. Select **Visual Studio Code (Microsoft)**
4. Click **Install**
5. Wait until installation finishes

✔ This installs the official Microsoft build

#### Method B — Terminal (Alternative)
```bash
sudo snap install code --classic
```

Either method is acceptable, but Method A is preferred for beginners.

#### Verify Installation

Run:
```bash
code
```

VS Code should open without errors.

---

### 7.3 Install Git (MANDATORY)

#### Why Git?
- Version control
- Assignment submission
- Collaboration
- Required for almost all modern development workflows

#### Install Git
```bash
sudo apt install -y git
```

#### Verify:
```bash
git --version
```

You should see a version number.

---

### 7.4 Install Build Essentials (MANDATORY)

#### Why this is required

Many projects require:
- `gcc`
- `g++`
- `make`
- `cmake`

Without these:
- Code will not compile
- ROS packages will fail later

#### Install Build Tools
```bash
sudo apt install -y build-essential cmake
```

#### Verify:
```bash
gcc --version
cmake --version
```

---

### 7.5 Install Common Utilities (Recommended)

These tools simplify daily work:
```bash
sudo apt install -y curl wget unzip htop
```

#### What they are used for:
- `curl` / `wget`: downloading files
- `unzip`: extracting archives
- `htop`: monitoring CPU & memory usage

---

### 7.6 Terminal Basics Check (Quick Sanity Test)

Make sure the student can run:
```bash
pwd
ls
cd ~
mkdir test_folder
cd test_folder
```

**If these work → environment is healthy.**

---

### 7.7 VS Code First-Time Setup (Minimal)

Inside VS Code:

1. Open **VS Code**
2. Go to **Extensions**
3. Install:
   - **Python**
   - **C/C++**
   - **GitHub Pull Requests** (optional)

⚠️ **Do NOT overload students with extensions yet.**

---

### 7.8 Reboot After Tool Installation (Recommended)
```bash
sudo reboot
```

#### Why:
- Ensures paths are updated
- Clears background package services
- Prevents weird first-launch issues

---

## Phase 8 — Final System Validation & Baseline Snapshot

---

### 8.1 Final System Validation (Must Pass All)

Open Terminal and verify each item.

#### OS & Kernel
```bash
lsb_release -a
uname -r
```

- ✔ Ubuntu 24.04.x LTS
- ✔ Kernel shows a recent 6.x version

#### Network & Updates
```bash
ping -c 3 google.com
sudo apt update
```

- ✔ Internet works
- ✔ No repository errors

#### Developer Tools
```bash
code --version
git --version
gcc --version
cmake --version
```

- ✔ All commands return versions (no "command not found")

#### Display & UX

- Resize VM window → screen auto-resizes ✔
- Full screen (Host+F) works ✔
- Clipboard copy/paste both directions ✔

---

### 8.2 Clean Up Before Snapshot (Important)

We snapshot only after cleanup to avoid carrying junk forward.
```bash
sudo apt autoremove -y
sudo apt clean
```

#### Optional (check disk usage):
```bash
df -h /
```

✔ Plenty of free space remains

---

### 8.3 Create the Baseline Snapshot (MANDATORY)

**This is your panic button for the entire course.**

#### Steps

1. Power off the VM (**Machine → Power Off**)
2. In **VirtualBox Manager**, select the VM
3. Go to **Snapshots**
4. Click **Take**
5. Name:
```
   BASELINE - Ubuntu 24.04 (Post-Setup)
```

6. Description (recommended):
```
   Clean install, updated system, Guest Additions, VS Code, Git, build tools.
```

✔ **Snapshot created successfully**

---

**Installation Complete!**

Your Ubuntu 24.04 VM is now fully configured and ready for ROS 2 Jazzy installation and development work. You can always restore to this snapshot if anything goes wrong in future phases.

---

## Summary Checklist

Before proceeding to ROS 2 installation, confirm:

- ✔ Phase 0: Virtualization enabled, Windows conflicts disabled
- ✔ Phase 1: VirtualBox and Extension Pack installed
- ✔ Phase 2: VM created with correct settings
- ✔ Phase 3: Graphics configured (VMSVGA, 128MB)
- ✔ Phase 4: Ubuntu installed successfully
- ✔ Phase 5: System updated and stable
- ✔ Phase 6: Guest Additions working
- ✔ Phase 7: Developer tools installed
- ✔ Phase 8: Baseline snapshot created

**You are now ready for Phase 9 - ROS 2 Jazzy Installation!**