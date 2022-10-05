# UMA Open Source Rover ROS2
Open hardware and software rover developed by Malaga University

*Authors:* C.J. Pérez del Pulgar, Ricardo Sánchez Ibáñez, Patricio López Lupiañez, Francisco de Asís Delgado Rivero and Laura Marta Mantoani

*Contact info:* carlosperez@uma.es

*Affiliation:* University of Malaga, [Space Robotics Lab](https://www.uma.es/robotics-and-mechatronics/info/107542/robotica-espacial/)


## Overview
The OpenUmaRov is an open source mobile robotic platform that allows anyone to build it with a maximum cost of 700€, taking into account that the Reach M+ GPS can be replaced with whatever GPS that works with NMEA sentences: just connecting it to the Raspberry Pi through USB port, thanks to the ROS node the GPS can publish NMEA sentences in different topics. This means that, replacing the Reach M+ with a cheaper one, the rover can be built with a maximum cost of 500 €. 

The platform is based on commercial components that can be acquired by Internet. This project involves students and researchers to improve the development of the platform. Moreover, it can be used for teaching and research activities that involves mechanical, engineering, software, electronics and robotics.

## ROS2 

An adaptation of the original code to ROS2 Humble has been developed. This enables the use of the Rover with the Middleware ROS2. 

### Guide

First you need to setup your package and your workspace.

First create Python ROS2 package:
```
cd ~/rov_ws/src        
ros2 pkg create --build-type ament_python --node-name ROS2UMARov rov_node_pkg
edit this file into ~/rov_ws/src/rov_node_pkg/rov_node_pkg/ROS2UMARov.py
add entry point to ~/rov_ws/src/rov_node_pkg/setup.py
     'console_scripts': [
            'simple_node = rov_node_pkg.ROS2UMARov:main',
```

Then make sure to source the package and build the workspace.
```
 source install/setup.bash
 colcon build
```

### Troubleshooting

Issues might appear while setting up your workspace.

- setuptools 58.2 for colcon build sterr output error due to deprecated version.

https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/

```pip install setuptools==58.2.0```

Install serial
Guide: https://create.arduino.cc/projecthub/ansh2919/serial-communication-between-python-and-arduino-e7cce0

```pip install pyserial```

TInfo about the Twist format for the msg variable which holds the data received from the topic.

```http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html```

### Tests with ros2 pub

Now that you have the message structure, you can publish data onto a topic directly from the command line using:

```ros2 topic pub <topic_name> <msg_type> <args>```

The ```'<args>'``` argument is the actual data you will pass to the topic, in the structure you just discovered in the previous section.

Its important to note that this argument needs to be input in YAML syntax. Input the full command like so:

```ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"```

```--once``` is an optional argument meaning “publish one message then exit”.

### Running the node

To run the node once sourced and built.

```ros2 run rov_node_pkg ROS2UMARov```

Check that /cmd_vel is a topic.

```ros2 topic list```

Make a small test publishing a certain v and w.

```ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"```
