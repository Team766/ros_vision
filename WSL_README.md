# WSL Setup Guide for ros_vision

This guide provides instructions for setting up USB camera access in Windows Subsystem for Linux (WSL) for the ros_vision project.

## Overview

WSL doesn't natively support USB devices. To use USB cameras with this vision system in WSL, you need to install and configure `usbipd-win` to forward USB devices from Windows to your WSL distribution.

## Prerequisites

- Windows 10 version 2004 and higher (Build 19041 and higher) or Windows 11
- WSL2 installed and configured
- Administrator privileges on Windows

## Installation

### Step 1: Install usbipd-win on Windows

1. Close any instances of Ubuntu terminal that you have open.
2. Open Windows Powershell in administrator mode.
3. Install usbipd with `winget install usbipd`
4. Upgrade the wsl version with `wsl --update`
5. Restart the Windows Powershell for the changes to take effect.
6. Start the Ubuntu terminal.
7. Plug in a USB camera to the computer.


## Camera Setup

### Step 3: List Available USB Devices

In Windows Command Prompt or PowerShell (run as Administrator):

```cmd
usbipd list
```

This will show all connected USB devices. Look for your camera device (typically shows as a USB camera, webcam, or video device).

Example output:
```
PS C:\Windows\system32> usbipd list
Connected:
BUSID  VID:PID    DEVICE                                                        STATE
1-1    1bcf:2284  Full HD webcam, USB microphone                                Shared
2-1    0c45:6366  Arducam OV9281 USB Camera                                     Shared
2-3    1038:1122  USB Input Device                                              Not shared
2-4    27c6:6094  Goodix MOC Fingerprint                                        Not shared
2-5    0489:e10a  Qualcomm FastConnect 7800 Dual Bluetooth Adapter              Not shared
4-1    5986:1193  FHD Camera, FHD IR Camera, Camera DFU Device                  Not shared

Persisted:
GUID                                  DEVICE
```

The state of Shared or Not Shared indicates whether wsl can access the camera or not.

### Step 4: Bind the Camera Device

Bind your camera device using its BUSID.  In the example above the Arducam OV9281 is at bus id 2-1:

```cmd
usbipd bind --wsl --busid 2-1
```

**Note**: You only need to bind a device once. The binding persists across reboots.

Type ```cmd
usbpid list
```

and confirm that the centry for the desired camera shows as Shared.

### Step 5: Attach the Camera to WSL

Attach the camera to your WSL distribution:

```cmd
usbipd attach --wsl --busid 2-1
```
You should see something like this:

```cmd
PS C:\Windows\system32> usbipd attach --wsl --busid 2-1
usbipd: info: Using WSL distribution 'Ubuntu-22.04' to attach; the device will be available in all WSL 2 distributions.
usbipd: info: Detected networking mode 'nat'.
usbipd: info: Using IP address 172.20.192.1 to reach the host.```

**Note**: You need to attach the device every time you restart WSL or disconnect/reconnect the camera.

### Step 6: Verify Camera Access in WSL

In your WSL terminal, verify the camera is accessible:

```bash
lsusb
```

You should see something like this:

```
cpadwick@MSI:~/code/ros_vision$ lsusb
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 008: ID 0c45:6366 Microdia Webcam Vitade AF
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```

Also check for video devices:

```bash
cpadwick@MSI:~/code/ros_vision$ ls /dev/video*
/dev/video0  /dev/video1
```

You should see video device files like `/dev/video0`, `/dev/video1`, etc.

## Usage Workflow

### Daily Usage

1. **Connect your camera** to your Windows machine
2. **Attach to WSL** (in Windows Command Prompt as Administrator):
   ```cmd
   usbipd attach --wsl --busid <your-camera-busid>
   ```
3. **Run your vision system** in WSL as normal

### When Finished

To detach the camera from WSL (optional):

```cmd
usbipd detach --wsl --busid <your-camera-busid>
```

## Troubleshooting

### Camera Not Detected in WSL

1. **Check if device is attached**:
   ```cmd
   usbipd wsl list
   ```
   Look for "Attached" status next to your camera.

2. **Verify USB/IP kernel module is loaded** in WSL:
   ```bash
   lsmod | grep usbip
   ```

3. **Check WSL kernel version**:
   ```bash
   uname -r
   ```
   Ensure you're running WSL2 with a recent kernel (5.10.16+ recommended).

### Permission Issues

If you get permission errors accessing `/dev/video*` devices:

```bash
sudo chmod 666 /dev/video*
```

Or add your user to the video group:

```bash
sudo usermod -a -G video $USER
```

Then restart your WSL session.

### Camera Busy/In Use Error

If you get "device busy" errors:

1. **Close any Windows applications** using the camera (like Camera app, Skype, etc.)
2. **Detach and reattach** the device:
   ```cmd
   usbipd detach --wsl --busid <your-camera-busid>
   usbipd attach --wsl --busid <your-camera-busid>
   ```

### Multiple Cameras

For multiple cameras, repeat the bind and attach process for each camera's BUSID:

```cmd
usbipd bind --wsl --busid 1-1
usbipd bind --wsl --busid 1-2
usbipd attach --wsl --busid 1-1
usbipd attach --wsl --busid 1-2
```

## Notes

The streaming bandwidth for usb devices in WSL is lower than using native Linux.  As such, for the Arducam streaming at 1280x800 causes some frames to be corrupted.  I have found that changing the resolution to 800x600 in system_config.json fixes those errors - but be warned though - the calibration files likely were generated at a higher resolution so streaming at a lower res and using these cal coefficients will cause apriltag positioning issues.

## Additional Resources

- [usbipd-win GitHub Repository](https://github.com/dorssel/usbipd-win)
- [Microsoft WSL USB Documentation](https://docs.microsoft.com/en-us/windows/wsl/connect-usb)
- [Main ros_vision README](./README.md)

## Notes

- USB device attachment doesn't persist across WSL restarts
- You must run usbipd commands as Administrator in Windows
- Some high-bandwidth devices may have reduced performance over USB/IP
- Consider using a powered USB hub for multiple cameras to ensure stable power delivery
