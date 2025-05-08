# Helpful ROS2 CMDs and Notes

Unlike ROS1, ROS2 does not need a roscore to be running. All nodes act independent. (Think of each node running its own roscore).
ROS2 runs CPP under the cover
ROS2 uses DDS for middleware (data distro service) handles all communication between pkgs and nodes.
ros2 can handle many many many programming languages using DDS.
Ex. Python, CPP, NodeJS, Java, etc.


The 2 most important communication features in ROS2 are Topics and Services.
Topics are used for data streams, and Services for a client/server interaction.

## TMUX (Terminal Alternative)
For a shortcut of all hotkeys for TMUX:
```
$ curl cheat.sh/tmux
```

## NODES OVERVIEW
Nodes are subprograms responsible for only one thing.
Ex. Inside the camera pkg there will be nodes. One node handles camera drivers like framegrabbing. One node handles image processing like recognition.
If there is another package for motion planning then DDS allows for communications between nodes and between pkgs.
DDS uses topics, services, and actions to transfer data between nodes/pkgs.

To run a node:
```
$ ros2 run <package name> <node name>
```


## ROSOUT
rosout records all logs from apps. So anytime you print statements a copy will go to rosout.

## RUNNING NODES
Gives you a list of all running nodes.
```
$ ros2 node list
```

Gives you all subs pubs and service servers for a node. Note the node must be running.
```
$ ros2 node info <name of node>
```

## RUNNING MULTIPLE INSTANCES OF THE SAME NODE
If we want to launch the same node multiple times (ex. node for temp sensor but you have 5 sensors) we need to make sure each node has a different name. If they are the same name, ROS won't complain but it can have unintended side effects. Especially for communications. 
to mitigate this, we rename nodes at launch. 
```
$ --ros-args --remap __node:=<name> 
```

## COLCON BUILD
Anytime you make changes to your python or cpp code, you will need to rebuild your workspace. 
This is the equivalent of ROS1's catkin_make command. This updates ros2 to reflect any file changes. Perform this command in the ros2_ws directory.
```
$ cd shagohod/ROS2/ros2_ws
$ colcon build
```
if you are consistently running and editing a package, consider using symbolic link to link your nodes to scripts directly instead of building them.
```
$ cd shagohod/ROS2/ros2_ws
$ colcon build --packages-select <package_name> --symlink-install
```


## RUNNING TOPICS
Gives you an echo of whatever the topic is publishing.
```
$ ros2 topic echo <name of ros topic>
```
Gives you the rate at which content is published to a topic 
```
$ ros2 topic hz <name of ros topic>
```
You can rename topics just like renaming nodes. Ex. ```--ros-args -r <original_topic>:=<new_topic_name>```

For example:
```
$ ros2 run <package name> <node name> --ros-args -r __node:=my_station -r robot_news:=my_news
```

Topic: Composed of two things: Name (ex. /number_count) and msg definition (ex: example_interfaces/msg/Int64)


## GRAPHICAL TOOL
rqt is a collection of ros2 plugins.
launches rqt (visual representation reference)
``` 
$ rqt
```
or
```
$ rqt_graph
```

## SERVICES
Unlike ROS1, services in ROS2 can be synchrononous or asynchronous (covers role of ROS1 actions)
Displays all available services
```
ros2 service list
```

You can rename services just like renaming nodes. Ex. ```--ros-args -r <original_service>:=<new_service_name>```

Service: Composed of two things: Name (ex. /reset_number_count) and srv definition (ex: example_interfaces/srv/SetBool)
Service definition is composed of request and response parts.


## SERVICE CALL
Just like with ROS1, we can use the command line to perform service calls.
```
$ ros2 service call /<service name> <service message type> "{message}"
```
For example:
```
$ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 7}"

```


## ROS PARAMETERS
Settings for nodes, value of parameters are set at runtime. Specific to a node and will die if node dies. So two nodes can have the same paramter name but the values of that parameter are unique. Can be booleans, ints, doubles, strings, lists, etc.
To list all available parameters, use:
```
$ ros2 param list
```
To start a node and set a parameter follow this example
```
$ ros2 run my_py_pkg number_publisher --ros-args -p number_to_publish:=3
```
You need to have this in your code first after ```super().__init__("number_publisher")```
```
self.declare_parameter("number_to_publish", <default value>) 
```
You can use the parameter value in your code by using the following line:
```
self.number = self.get_parameter("number_to_publish").value
```


## Making python files executable
In terminal, navigate to where your script is located. Then use the following command.
```chmod +x nameOfPythonScript.py```