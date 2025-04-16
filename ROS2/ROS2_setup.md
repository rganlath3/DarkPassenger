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
```ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_cont/cmd_vel_unstamped```
We have to remap the cmd_vel topic due to Jazzy-Harmonic changes.