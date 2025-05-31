# Dark Passenger ROS2 Setup and Configuration

[![Hardware License: CERN-OHL-S-2.0](https://img.shields.io/badge/Hardware%20License-CERN--OHL--S--2.0-lightgrey.svg)](https://ohwr.org/cern_ohl_s_v2.txt)
[![Software License: GPL v3](https://img.shields.io/badge/Software%20License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Status: In Development](https://img.shields.io/badge/Status-In%20Development-yellow.svg)]()

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


## Making python files executable
In terminal, navigate to where your script is located. Then use the following command.
```chmod +x nameOfPythonScript.py```

## Remapping Teleop Twist Keyboard ROS2 Node
### For ROS2 DiffDrive Plugin
```ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped```


### For ROS2 ROS2_Control Plugin
```ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel -p stamped:=true```





