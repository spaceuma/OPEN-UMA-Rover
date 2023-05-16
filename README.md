# UMA Open Source Rover ROS2
Open hardware and software rover developed by Malaga University

*Authors:* C.J. Pérez del Pulgar, Ricardo Sánchez Ibáñez, Patricio López Lupiañez, Francisco de Asís Delgado Rivero, Laura Marta Mantoani and Danil Rivero Pavlenko

*Contact info:* carlosperez@uma.es

*Affiliation:* University of Malaga, [Space Robotics Lab](https://www.uma.es/robotics-and-mechatronics/info/107542/robotica-espacial/)


## Overview
The OpenUmaRov is an open source mobile robotic platform that allows anyone to build it with a maximum cost of 700€, taking into account that the Reach M+ GPS can be replaced with whatever GPS that works with NMEA sentences: just connecting it to the Raspberry Pi through USB port, thanks to the ROS node the GPS can publish NMEA sentences in different topics. This means that, replacing the Reach M+ with a cheaper one, the rover can be built with a maximum cost of 500 €. 

The platform is based on commercial components that can be acquired by Internet. This project involves students and researchers to improve the development of the platform. Moreover, it can be used for teaching and research activities that involves mechanical, engineering, software, electronics and robotics.

## ROS2 

An adaptation of the original code to ROS2 Humble has been developed. This enables the use of the Rover with the Middleware ROS2. 

![rover_ros2](https://user-images.githubusercontent.com/94115082/211777911-90832245-dfb3-46b3-bb0b-76830e6d5024.png)

### Guide

First you need to setup your package and your workspace. Use the package in this repository inside the workspace.

Then make sure to source the package and build the workspace.
```
 source install/setup.bash
 colcon build
```

### Troubleshooting

Issues might appear while setting up your workspace.

- setuptools 58.2 for colcon build sterr output error due to deprecated version.

https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/

```
pip install setuptools==58.2.0
```

Install serial
Guide: https://create.arduino.cc/projecthub/ansh2919/serial-communication-between-python-and-arduino-e7cce0

```
pip install pyserial
```

Info. about the Twist format for the msg variable which holds the data received from the topic.

http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html

### Running the node

To run the node once sourced and built.

```
ros2 run rov_node_pkg ROS2UMARov
```

## Rover URDF

In order to test the Rover in a simulated environment, a URDF file of the OpenUMARov has been developed.

![alt text](https://github.com/spaceuma/OPEN-UMA-Rover/blob/ROS2/rover_urdf.png?raw=true)

## Rover SDF

To implement Rover simulations on Gazebo, it is necessary to develop the SDF Model of it. 

![alt text](https://github.com/spaceuma/OPEN-UMA-Rover/blob/ROS2/SDF_Model.png?raw=true)

This model uses the following plugins.

- GPS Plugin to simulate such in the simulation environment.
- IMU to simulate the orientation of the Rover.
- Differential drive Model to implement the movement of the Rover 
- Joint State Publisher to know wheels state.

An alternative for NAV2 has been implemented as en extended model.

## Joystick Control

A good way to easily control the Rover is by using the Teleoperation Joystick Control.

By default, it uses the ```ps3_config.yaml``` file.

To test this package a F-710 Game Pad with a USB with Bluetooth connection was used, even though the ```.config``` file is for the PS3 Joystick. If you wish to use a Bluetooh joystick control, you have to edit the directory of the connection to rather the Bluetooth direction, using a USB connection is recommended nonetheless.

To use it with the PC the following changes should be done:

- Add permissions to read/write Serial: 

```
sudo adduser $USER $(stat --format="%G" /dev/ttyACM0)
```

- Build the package with the following command:

```
colcon build --allow-overriding teleop_twist_joy
```

PS3 is default, to run for another config (e.g. xbox) use this:

```
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

Note: this launch file also launches the joy node so do not run it separately.

Only if required:

- Uninstall the default joystick teleop. from ROS2 Lib. Head to opt/ros/lib and remove the file teleop_twist_joy. 
- The last step is due to ROS2 using the config. of the default driver and its parameters instead of your package.
- Launch the driver as normally.
