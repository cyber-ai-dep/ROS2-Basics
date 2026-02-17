# Ubuntu 24.04 Installation on VirtualBox

---

## Recommended Device Specifications

| | |
|---|---|
| **Minimum RAM** | 8 GB |
| **Recommended RAM** | 16 GB or more |
| **Minimum CPU** | 2 physical cores |
| **Disk Space** | At least 60 GB free on your computer |

---

## Part 1: Install VirtualBox

---

### Step 1 — Download Ubuntu 24.04

Go to the official Ubuntu website and download the Ubuntu 24.04 ISO file.
Visit: https://ubuntu.com/download/desktop
The file will be named something like `ubuntu-24.04-desktop-amd64.iso`.
![Step 1](VM1.jpeg)

---

### Step 2 — Open the VirtualBox Website

Go to the official VirtualBox website to download the installer.
Visit: https://www.virtualbox.org/wiki/Downloads
Choose the version that matches your operating system (Windows, macOS, or Linux).

![Step 2](VM2.jpeg)

---

### Step 3 — Download VirtualBox

Click the download link for your operating system.
The file will download to your computer. Wait for it to finish before continuing.

![Step 3](VM3.jpeg)

---

### Step 4 — Run the VirtualBox Installer

Open the downloaded installer file.
Follow the on-screen steps and keep all the default options selected.
Click **Next** until the installation begins.

![Step 4](VM4.jpeg)

---

### Step 5 — Confirm the Installation Warning

During installation, you may see a warning about your network connection being temporarily interrupted.
This is normal. Click **Yes** to continue.

![Step 5](VM5.jpeg)

---

### Step 6 — Complete the Installation

Click **Install** to start installing VirtualBox on your computer.
Wait for the progress bar to finish. This may take a few minutes.

![Step 6](VM6.jpeg)

---

### Step 7 — Finish and Launch VirtualBox

Once installation is done, click **Finish**.
VirtualBox will open automatically.

![Step 7](VM7.jpeg)

---

## Part 2: Install Ubuntu

---

### Step 8 — Create a New Virtual Machine

In VirtualBox, click the **New** button to create a new virtual machine.
A setup window will appear.

![Step 8](VM8.jpeg)

---

### Step 9 — Name Your Virtual Machine and Attach the ISO

Give your virtual machine a name (for example: `Ubuntu-24.04`).
Under **ISO Image**, click the dropdown and select your downloaded Ubuntu ISO file.
Make sure to check the box **Skip Unattended Installation**.

![Step 9](VM9.jpeg)

---

### Step 10 — Set the RAM and CPU

Set the **Memory (RAM)** based on your computer:
- If you have 8 GB RAM → set 4096 MB
- If you have 16 GB RAM → set 6144 MB

Set the **CPU cores** to 2 or more.
Stay within the green zone shown on the slider.

![Step 10](VM10.jpeg)

---

### Step 11 — Create a Virtual Hard Disk

Select **Create a Virtual Hard Disk Now**.
Set the disk size to **50 GB**. This gives enough space for Ubuntu and your files.
Click **Next** to continue.

![Step 11](VM11.jpeg)

---

### Step 12 — Review and Finish VM Creation

You will see a summary of your virtual machine settings.
Check that everything looks correct, then click **Finish**.

![Step 12](VM12.jpeg)

---

### Step 13 — Open VM Settings

Your new virtual machine now appears in VirtualBox.
Before starting it, click **Settings** (the gear icon) to adjust a few important options.

![Step 13](VM13.jpeg)

---

### Step 14 — System Settings: Motherboard Tab

Go to **Settings → System → Motherboard**.
Set the **Boot Order** so that **Optical** is first and **Hard Disk** is second.
Make sure **Enable EFI** is **unchecked** (turned OFF).

> ⚠️ Leaving EFI enabled is one of the most common causes of installation failure. Keep it OFF.

![Step 14](VM14.jpeg)

---

### Step 15 — System Settings: Processor Tab

Go to **Settings → System → Processor**.
Set the number of processors to **2 or more** (stay in the green zone).
Make sure **Enable PAE/NX** is checked.

![Step 15](VM15.jpeg)

---

### Step 16 — Display Settings

Go to **Settings → Display → Screen**.
Set **Video Memory** to **128 MB** (drag the slider all the way to the right).
Set **Graphics Controller** to **VMSVGA**.
Make sure **Enable 3D Acceleration** is **unchecked** (turned OFF).

> ⚠️ These display settings are critical. Wrong settings can cause crashes or a black screen.

![Step 16](VM16.jpeg)

---

### Step 17 — Storage Settings: Attach the ISO

Go to **Settings → Storage**.
Click on the **Empty** optical drive under Controller: IDE.
On the right side, click the **CD icon** and choose your Ubuntu ISO file.
This tells the VM to boot from the Ubuntu installer.

![Step 17](VM17.jpeg)

---

### Step 18 — Network Settings

Go to **Settings → Network**.
Make sure **Adapter 1** is enabled and set to **NAT**.
This gives your virtual machine internet access. Click **OK** to save all settings.

![Step 18](VM18.jpeg)

---

### Step 19 — Verify Your VM Settings (Important!)

Before starting the VM, take a moment to double-check your settings. These two screenshots show the **correct** configuration you should have.

**VM19-1** shows the correct **System** settings:
- EFI is **disabled**
- Boot order is correct (Optical first, then Hard Disk)

![Step 19-1](VM19-1.jpeg)

**VM19-2** shows the correct **Display** settings:
- Video Memory is set to **128 MB**
- Graphics Controller is set to **VMSVGA**
- 3D Acceleration is **disabled**

![Step 19-2](VM19-2.jpeg)

> ✅ If your settings match these screenshots, you are ready to start the installation.

---

### Step 20 — Start the Virtual Machine

Click the **Start** button (the green arrow) to power on your virtual machine.
A new window will open showing the Ubuntu installer loading.

![Step 20](VM20.jpeg)

---

### Step 21 — Select "Try or Install Ubuntu"

You will see the Ubuntu boot menu.
Select **Try or Install Ubuntu** and press **Enter**.
Wait a moment for the installer to load.

![Step 21](VM21.jpeg)

---

### Step 22 — Choose Your Language

The Ubuntu installer will open.
Select your language (English is recommended) and click **Next**.

![Step 22](VM22.jpeg)

---

### Step 23 — Accessibility Options

This screen shows accessibility options.
You do not need to change anything here. Click **Next** to continue.

![Step 23](VM23.jpeg)

---

### Step 24 — Select Your Keyboard Layout

Choose your keyboard layout from the list.
If you are unsure, leave the default selection. Click **Next**.

![Step 24](VM24.jpeg)

---

### Step 25 — Connect to the Internet

If prompted to connect to the internet, select your network or choose **Use wired connection**.
An internet connection allows Ubuntu to download updates during installation.

![Step 25](VM25.jpeg)

---

### Step 26 — Choose Installation Type

Select **Install Ubuntu** (not "Try Ubuntu").
Click **Next** to continue.

![Step 26](VM26.jpeg)

---

### Step 27 — Select Installation Options

Choose **Interactive installation** to go through the setup manually.
This gives you full control over the settings.

![Step 27](VM27.jpeg)

---

### Step 28 — Choose What to Install

Select **Default selection** (recommended for beginners).
This installs a standard set of apps and tools.
Click **Next**.

![Step 28](VM28.jpeg)

---

### Step 29 — Install Third-Party Software

Check the boxes to install third-party software and download updates.
This improves hardware compatibility. Click **Next**.

![Step 29](VM29.jpeg)

---

### Step 30 — Disk Setup: Erase and Install

Select **Erase disk and install Ubuntu**.
Do not worry — this only affects the virtual disk, not your real computer.
Click **Next**.

![Step 30](VM30.jpeg)

---

### Step 31 — Create Your User Account

Fill in the following:
- **Your name**: Your full name
- **Computer name**: Something simple like `ubuntu-vm`
- **Username**: A short lowercase name (example: `student`)
- **Password**: Choose a password you will remember

Write your password down somewhere safe. Click **Next**.

![Step 31](VM31.jpeg)

---

### Step 32 — Choose Your Timezone

Select your timezone from the map or dropdown.
Click **Next** to continue.

![Step 32](VM32.jpeg)

---

### Step 33 — Review and Begin Installation

You will see a summary of all your choices.
If everything looks correct, click **Install** to begin.
The installation will take around 10–20 minutes. Do not close the window.

![Step 33](VM33.jpeg)

---

### Step 34 — Installation Complete: Restart

When the installation finishes, you will see a message saying it is complete.
Click **Restart Now**.
When prompted to "remove the installation medium", go to **Devices → Optical Drives → Remove Disk from Virtual Drive**, then press **Enter** in the VM window.

![Step 34](VM34.jpeg)

---

### Step 35 — Ubuntu is Ready!

Your virtual machine will restart and boot into Ubuntu 24.04.
Log in with the username and password you created.
Welcome to your new Ubuntu environment! 🎉

![Step 35](VM35.jpeg)

---
## Additional Resources

### Official Documentation
- **Ubuntu**: https://help.ubuntu.com
- **VirtualBox**: https://www.virtualbox.org/manual/
- **ROS 2 Jazzy**: https://docs.ros.org/en/jazzy/

### Community Support
- **ROS Discourse**: https://discourse.ros.org
- **Ubuntu Forums**: https://ubuntuforums.org
- **Stack Overflow**: Tag your questions with `ros2`, `ubuntu`, or `virtualbox`

### Learning Resources
- **ROS 2 Tutorials**: https://docs.ros.org/en/jazzy/Tutorials.html
- **Linux Command Line Basics**: https://ubuntu.com/tutorials/command-line-for-beginners
- **Git Tutorial**: https://git-scm.com/book/en/v2

---

**Document Version**: 1.0  
**Last Updated**: February 2026  
**Target**: Ubuntu 24.04 LTS, ROS 2 Jazzy Jalisco, VirtualBox 7.0+  
**Maintained By**: Cyber Robot Team
