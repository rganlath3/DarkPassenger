# Dark Passenger ROS2 Setup and Configuration

[![Hardware License: CERN-OHL-S-2.0](https://img.shields.io/badge/Hardware%20License-CERN--OHL--S--2.0-lightgrey.svg)](https://ohwr.org/cern_ohl_s_v2.txt)
[![Software License: GPL v3](https://img.shields.io/badge/Software%20License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Status: In Development](https://img.shields.io/badge/Status-In%20Development-yellow.svg)]()

#### Install ROS2 Jazzy Jalisco on Ubuntu 24.04 environment
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
Verify ROS2 was installed correctly using cpp talker and py listener.

#### Configure your ROS2 environment
Source your ROS2 install in your shell startup script.
```echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc```

Verify the environment is configured properly.
``` printenv | grep -i ROS```

You should get ROS_VERSION=2, ROS_PYTHON_VERSION=3, and ROS_DISTRO=jazzy


#### Build your package
Navigate to the workspace. Ex. "github/DarkPassenger/ROS2/dp_ws"
```colcon build --symlink-install```