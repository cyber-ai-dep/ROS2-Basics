# Ubuntu 24.04 Dual Boot Guide (Windows + Ubuntu)

## 1. Device Specifications (Recommended Minimum)

| Component | Practical Minimum |
|-----------|-------------------|
| CPU | Intel i5 6th Gen or older Ryzen 5 |
| RAM | 8 GB |
| Storage | SSD |
| Free Space (on SSD) | 60–80 GB |
| GPU | Intel HD / UHD |

ROS2 + RViz will run properly. Gazebo works for light simulations only.

## 2. Requirements

* 8 GB USB flash drive
* Rufus: **https://rufus.ie**
* Ubuntu 24.04 ISO: **https://ubuntu.com/download/desktop**

## 3. Create Bootable USB

1. Download Ubuntu ISO. from link https://ubuntu.com/download/desktop

![Download Ubuntu ISO](../../../assets/images/fundamentals/Download%20Ubuntu%20ISO.png)

2. Install and open Rufus. from link https://rufus.ie

![Install Rufus](../../../assets/images/fundamentals/Install%20Rufus.png)

3. Insert USB flash drive.

![Install Rufus1](../../../assets/images/fundamentals/Install%20Rufus1.png)

4. Select Ubuntu ISO.

![Install Rufus2](../../../assets/images/fundamentals/Install%20Rufus2.png)

![Install Rufus3](../../../assets/images/fundamentals/Install%20Rufus3.png)

5. Click Start.

![Install Rufus4](../../../assets/images/fundamentals/Install%20Rufus4.png)

![Install Rufus5](../../../assets/images/fundamentals/Install%20Rufus5.png)


**Note:** After this step, the USB cannot be used for storage until formatted again.

## 4. Create Free Space in Windows


1. Right-click Windows icon

![Create Free Space in Windows](../../../assets/images/fundamentals/Create%20Free%20Space%20in%20Windows.png)

2. Select Disk Management
3. Right-click on C: (SSD)
4. Select Shrink Volume

![Create Free Space in Windows1](../../../assets/images/fundamentals/Create%20Free%20Space%20in%20Windows1.png)

5. Enter size in MB


**Recommended:** 100000 MB (~100 GB)

![Create Free Space in Windows2](../../../assets/images/fundamentals/Create%20Free%20Space%20in%20Windows2.png)

You should see Unallocated Space after shrinking.

![Create Free Space in Windows3](../../../assets/images/fundamentals/Create%20Free%20Space%20in%20Windows3.png)

## 5. Disable Secure Boot

**Check Status**
1. Search for System Information
2. Confirm Secure Boot State = Off

![Create Free Space in Windows4](../../../assets/images/fundamentals/Create%20Free%20Space%20in%20Windows4.png)

![Create Free Space in Windows5](../../../assets/images/fundamentals/Create%20Free%20Space%20in%20Windows5.png)

**If Secure Boot is On**

Restart the device and follow:
1. Troubleshoot → Advanced Options → UEFI Firmware Settings

![Create Free Space in Windows6](../../../assets/gifs/fundamentals/Create%20Free%20Space%20in%20Windows6.gif)

Then:
1. Boot Menu → Secure Boot → Disable
2. Exit → Save Changes & Reset

![Create Free Space in Windows7](../../../assets/images/fundamentals/Create%20Free%20Space%20in%20Windows7.png)

![Create Free Space in Windows8](../../../assets/images/fundamentals/Create%20Free%20Space%20in%20Windows8.png)

![Create Free Space in Windows9](../../../assets/images/fundamentals/Create%20Free%20Space%20in%20Windows9.png)

Return to Windows and confirm Secure Boot is Off.

## 6. Enter Boot Menu (Common Devices)

| Brand | Boot Key |
|-------|----------|
| Lenovo | F12 |
| HP | F9 |
| Dell | F12 |
| ASUS | F8 |
| Acer | F12 |
| MSI | F11 |
| Toshiba | F12 |

Restart and repeatedly press the corresponding key.


## 7. Install Ubuntu

1. Select USB from Boot Menu.
2. Choose Install Ubuntu.

![Enter Boot Menu](../../../assets/images/fundamentals/Enter%20Boot%20Menu.png.png)

![Install Ubuntu](../../../assets/images/fundamentals/Install%20Ubuntu.png)

![Install Ubuntu2](../../../assets/images/fundamentals/Install%20Ubuntu2.png)

![Install Ubuntu3](../../../assets/images/fundamentals/Install%20Ubuntu3.png)

![Install Ubuntu4](../../../assets/images/fundamentals/Install%20Ubuntu4.png)

![Install Ubuntu5](../../../assets/images/fundamentals/Install%20Ubuntu5.png)

![Install Ubuntu6](../../../assets/images/fundamentals/Install%20Ubuntu6.png)

![Install Ubuntu7](../../../assets/images/fundamentals/Install%20Ubuntu7.png)



3. Select Something Else (Manual Partitioning).
4. Use only the Unallocated Space.

## 8. Required Partitions

You need three partitions:

| Mount Point | Type | File System | Size |
|-------------|------|-------------|------|
| swap | Logical | swap | RAM × 2 (Max 8GB) |
| / (root) | Primary | ext4 | 60–70% of free space |
| /home | Logical | ext4 | Remaining space |


![Install Ubuntu8](../../../assets/images/fundamentals/Install%20Ubuntu8.png)

![Install Ubuntu9](../../../assets/images/fundamentals/Install%20Ubuntu9.png)

![Install Ubuntu10](../../../assets/images/fundamentals/Install%20Ubuntu10.png)


**Example (100GB Free Space)**

| Partition | Size |
|-----------|------|
| swap (RAM = 8GB) | 8GB |
| / | 60–70GB |
| /home | 22–32GB |

Bootloader installation device: main disk (usually /dev/sda).

**Do not modify existing Windows partitions.**

![Install Ubuntu11](../../../assets/images/fundamentals/Install%20Ubuntu11.png)

## 9. After Installation

After reboot and removing flash drive, GRUB menu will appear:
* Ubuntu
* Windows Boot Manager


You can choose the desired OS at each startup.