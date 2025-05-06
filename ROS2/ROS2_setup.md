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

Navigate to the workspace and run this command to find the packages
```source install/setup.bash```

Add the path of the ROS2 workspace to the startup script as well.
```source /opt/ros/jazzy/setup.bash```
```source ~/github/DarkPassenger/ROS2/dp_ws/install/setup.bash```

#### Install Gazebo Harmonic Simulator
```sudo apt-get install ros-${ROS_DISTRO}-ros-gz```
```sudo apt install ros-jazzy-gz-ros2-control```
```sudo apt install ros-jazzy-twist-mux```
```sudo apt install ros-jazzy-twist-stamper```
```sudo apt install ros-jazzy-ros2-control```
```sudo apt install ros-jazzy-ros2-controllers```
```sudo apt install ros-jazzy-joy*```
```sudo apt install ros-jazzy-joint-state-publisher```



Test your install
```ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf```

Gazebo Harmonic differs from the "Classic" Gazebo meaning that there's a few changes to make to packages that are used to interfacing with classic gazebo.
1. Modify package.xml and CMakeLists.txt files replace "gazebo, "gazebo_ros_pkgs", etc with packages from "ros_gz"
2. Edit launch files that start Gazebo
3. Update the world SDFormat File
4. Edit launch files that spawn models.
5. Edit modelSDFormat files.
6. Bridge ROS topics.

#### Build your package
Navigate to the workspace. Ex. "github/DarkPassenger/ROS2/dp_ws"
```colcon build --symlink-install```

#### Useful Commands (to organize later)
To launch the main launch script for use with rviz.
```ros2 launch dexter rsp.launch.py ```

To launch rviz2 with configuration files
```rviz2 -d src/dexter/config/view_bot.rviz```

To launch the main launch script for use with gazebo.
```ros2 launch dexter rsp.launch.py use_sim_time:=true```


To control wheels, you need to be running joint state publisher.
```ros2 run joint_state_publisher_gui joint_state_publisher_gui```



#### Testing Importing the robot into Gazebo.
Create an empty world gazebo sim
```ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf```

Create our robot instance
```ros2 launch dexter rsp.launch.py use_sim_time:=true```

Import our robot into the gazebo world simulation.
```ros2 run ros_gz_sim create -topic robot_description```

Note: this is a migrated command from the classic gazebo command ```ros2 run gazebo_ros spawn_entity.py -topic robot_description```


#### Using a launch file to start Gazebo, Robot State Publisher, and Spawn our Robot from URDF file.
```ros2 launch dexter launch_sim.launch.py```



#### Controlling your Robot's velocity commands via keyboard for Gazebo and RVIZ
Using 3x terminals:
Term1: ```ros2 launch dexter launch_sim.launch.py```
Term2: ```ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_cont/cmd_vel_unstamped```
We have to remap the cmd_vel topic due to Jazzy-Harmonic changes.
Term3: ```rviz2```


#### Connecting to the RPLidar (Physical Hardware)
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


#### Camera (Software)
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


#### Depth Camera
I am using an RGBD camera in Gazebo (URDF plugin) and passing the image and depth_image info using gz_bridge. For the depth point data, I am....



#### ROS2 Jazzy and Gazebo Harmonic
In Gazebo Classic, topics were easily shared between Gazebo and ROS2 interface. Now they are separate and we need to use a bridge to link the two topic names together. Often times, the easiest solution to a problem is that a topic is only in one environment and isn't bridged. We can check this by using the topic viewer in Gazebo and comparing it to the ROS2 topic list command output. We define the data we are bridging using the gz_bridge.yaml file and specify it in our launch command. Image topics use their own launch command.




#### Arduino Serial Setup
Install Arduino 1.8.19 IDE

Add permissions to use serial ports.
```sudo usermod -a -G dialout $USER```

Note: You will need to log out and log back in for the changes to take effect. Install the Arduino IDE on your computer. This tutorial used 1.8.19. Verify the code works using the Arduino IDE serial monitor. If it works, record the device's serial port using the following command:

```ls /dev/tty*```
Note: The serial port will be /dev/ttyUSB* or /dev/ttyACM*. 

Install the pyserial package on your computer using the following command:
```pip install pyserial```

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