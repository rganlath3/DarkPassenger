# Dark Passenger ROS2 Setup and Configuration

[![Hardware License: CERN-OHL-S-2.0](https://img.shields.io/badge/Hardware%20License-CERN--OHL--S--2.0-lightgrey.svg)](https://ohwr.org/cern_ohl_s_v2.txt)
[![Software License: GPL v3](https://img.shields.io/badge/Software%20License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Status: In Development](https://img.shields.io/badge/Status-In%20Development-yellow.svg)]()

### Generic Software Tools to Install


### Generic Software Tools to Install with Ubuntu 24.04
sudo apt update
sudo apt upgrade
sudo apt install git terminator chromium-browser gedit python3-pip
sudo snap install --classic code

#### VSCode Extensions
Remote Development by Microsoft
CMake Tools by Microsoft
C/C++ by Microsoft
C/C++ Extension Pack by Microsoft
Python by Microsoft
Python Debugger by Microsoft
Pylance by Microsoft
ROS by Microsoft
URDF by smilerobotics
URDF Visualizer by morningfrog
XML Tools by Josh Johnson

### Cloning this repo
I have a ROS2 workspace built into the repo so all you need to do is clone this repo into a directory of your choice. I am using a folder called "github" in my root directory.
cd ~/github/

git clone https://github.com/rganlath3/DarkPassenger.git

### SSH Setup
sudo apt install openssh-server

### Installing ROS2 Jazzy Jalisco
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
Verify ROS2 was installed correctly using cpp talker and py listener.

#### Configure your ROS2 environment
Source your ROS2 install in your shell startup script.
```echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc```

Verify the environment is configured properly.
``` printenv | grep -i ROS```

You should get ROS_VERSION=2, ROS_PYTHON_VERSION=3, and ROS_DISTRO=jazzy

Navigate to the workspace and run this command to find the packages
```source install/setup.bash```

Add the path of the ROS2 workspace to the startup script as well. (gedit ~/.bashrc)
```source /opt/ros/jazzy/setup.bash```
```source ~/github/DarkPassenger/ROS2/dp_ws/install/setup.bash```
```cd ~/github/DarkPassenger/ROS2/dp_ws```



#### Install ROS2 Tools and Gazebo Harmonic Simulator
```sudo apt-get install ros-${ROS_DISTRO}-ros-gz```
```sudo apt install ros-jazzy-twist-mux```
```sudo apt install ros-jazzy-twist-stamper```
```sudo apt install ros-jazzy-joy*```
```sudo apt install ros-jazzy-joint-state-publisher```

#### Installing ROS2 Control
```sudo apt install ros-jazzy-ros2-control```
```sudo apt install ros-jazzy-ros2-controllers```
```sudo apt install ros-jazzy-gz-ros2-control```

Test your gazebo install
```ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf```


In Gazebo Classic, topics were easily shared between Gazebo and ROS2 interface. Now they are separate and we need to use a bridge to link the two topic names together. Often times, the easiest solution to a problem is that a topic is only in one environment and isn't bridged. We can check this by using the topic viewer in Gazebo and comparing it to the ROS2 topic list command output. We define the data we are bridging using the gz_bridge.yaml file and specify it in our launch command. Image topics use their own launch command.

Gazebo Harmonic differs from the "Classic" Gazebo meaning that there's a few changes to make to packages that are used to interfacing with classic gazebo.
1. Modify package.xml and CMakeLists.txt files replace "gazebo, "gazebo_ros_pkgs", etc with packages from "ros_gz"
2. Edit launch files that start Gazebo
3. Update the world SDFormat File
4. Edit launch files that spawn models.
5. Edit modelSDFormat files.
6. Bridge ROS topics.


### Arduino Setup
Install Arduino 1.8.19 IDE

Add permissions to use serial ports.
```sudo usermod -a -G dialout $USER```

Note: You will need to reboot your computer for the changes to take effect. Install the Arduino IDE on your computer. This tutorial used 1.8.19. Verify the code works using the Arduino IDE serial monitor. If it works, record the device's serial port using the following command:

```ls /dev/tty*```
Note: The serial port will be /dev/ttyUSB* or /dev/ttyACM*. 


Verify installation
```python3```
```import serial```
```import time```
```esp32=serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)```
```print(esp32.readline())```

Now we can write a Python node that communicates with your ESP32 board through the serial port.

#### Interfacing with ESP32 over Serial
I created a custom python ROS2 package for interfacing with both the base and turret (future implementation) microcontrollers.
The base_bridge.py connects the ESP32 with 2x topics: serial_tx and serial_rx. Other nodes interface with these topics to generate commands or use the input datastream.
The base_rx_topic_router processes the serial_rx topic and formats and packages data out to relevant topics.

### RPLidar Setup
Install the RPLIDAR ROS Package
```sudo apt install ros-jazzy-rplidar-ros```

Add USB Reading Permissions:
```sudo usermod -a -G dialout $USER```

Check List of USB Devices
```ls -1 /dev |grep ttyUSB```

Run the Lidar Node:
```ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0 -p frame_id:=laser_frame -p angle_compensate:=true -p scan_mode:=Standard```

To Stop The motor:
```ros2 service call /stop_motor std_srvs/srv/Empty {}```

To Start the motor:
```ros2 service call /start_motor std_srvs/srv/Empty {}```


Here is a good launch file test script:
```ros2 launch rplidar_ros view_rplidar.launch.py```

If a closed script is still hogging the USB port, you can run this command:
```killall rplidar_composition```


### Camera (Software)
The camera is outputting it's uncompressed image out to the topic: /camera/image_raw

In order to compress the images (eats up processing power but greatly reduces bandwidth for video stream), we need to install a ROS transport plugin.
```sudo apt install ros-jazzy-image-transport-plugins```
```sudo apt install ros-jazzy-rqt-image-view```

To visually see ros image topics, use ```ros2 run rqt_image_view  rqt_image_view```

To convert between compressed and uncompressed feeds, we can use:
ros2 run image_transport list_transports

Ex. Compressed to Raw
ros2 run image_transport republish compressed raw --ros-args -r in/compressed:=/camera/image_raw/compressed -r out:=/camera/image_raw/raw/uncompressed
Note: This doesn't seem to be working as the output is not being configured correctly. I couldnt find any similar errors online.


### ZED Depth Camera
I am using an RGBD camera in Gazebo (URDF plugin) and passing the image and depth_image info using gz_bridge. For the depth point data, I am....

The ZED camera requires CUDA meaning it needs an NVIDIA graphics card to run. The development machine or a JETSON must be used. 
https://github.com/stereolabs/zed-ros2-wrapper