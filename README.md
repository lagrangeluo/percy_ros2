# ROS2 Packages for Percy Mobile Robot

## Packages

This repository contains minimal packages to control the percy robot using ROS. 

* percy_base: a ROS wrapper around [ugv_sdk](https://github.com/agilexrobotics/ugv_sdk) to monitor and control the percy robot
* percy_msgs: percy related message definitions

## Supported Hardware

* Percy
* Percy-mini

## Basic usage of the ROS packages

1. Clone the packages into your colcon workspace and compile

    (the following instructions assume your catkin workspace is at: ~/ros2_ws/src)

    ```
    $ mkdir -p ~/ros2_ws/src
    $ cd ~/ros2_ws/src
    $ git clone https://github.com/agilexrobotics/ugv_sdk.git
    $ git clone https://github.com/agilexrobotics/percy_ros2.git
    $ cd ..
    $ colcon build
    ```
2. Setup CAN-To-USB adapter

* Enable gs_usb kernel module(If you have already added this module, you do not need to add it)
    ```
    $ sudo modprobe gs_usb
    ```
    
* first time use percy-ros2 package
   ```
   $ cd ~/your_ws/src/ugv_sdk/scripts/
   $ bash setup_can2usb.bash
   ```
   
* if not the first time use percy-ros2 package(Run this command every time you turn off the power) 
   ```
   $ cd ~/ros2_ws/src/ugv_sdk/scripts/
   $ bash bringup_can2usb_500k.bash
   ```
   
* Testing command
    ```
    # receiving data from can0
    $ candump can0
    ```
3. Launch ROS nodes

* Start the base node for the Percy robot

    ```
    $ ros2 launch percy_base percy_base.launch.py
    ```
    or
     ```
    $ ros2 launch percy_base percy_mini_base.launch.py
     ```

* Then you can send command to the robot
    ```
    $ ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
    x: 0.0
    y: 0.0
    z: 0.0
    angular:
    x: 0.0
    y: 0.0
    z: 0.0" 
    
    ```
    **SAFETY PRECAUSION**: 

Always have your remote controller ready to take over the control whenever necessary. 

## CAN test

- enable the virtual can net

  ``` bash
  sudo modprobe vcan
  sudo ip link add dev vcan0 type vcan
  sudo ip link set up vcan0
  ```

- then launch the fake robot

  ```bash
  ros2 launch percy_base percy_fake_base.launch.py
  ```

- listen to the can net in terminal

  ```bash
  candump vcan0
  ```

- pub some messages

  - the light control message:

  ```bash
  ros2 topic pub -1 /light_control percy_msgs/msg/PercyLightCmd "{cmd_ctrl_allowed: true, front_illumination_mode: 1, back_illumination_mode: 1}"
  ```
  - the light rgb message:

  ```bash
  ros2 topic pub -1 /light_rgb_cmd percy_msgs/msg/PercyRgbCmd "{front_left: {r_value: 1, g_value: 2, b_value: 3}, front_right: {r_value: 4, g_value: 5, b_value: 6}, back_left: {r_value: 7, g_value: 8, b_value: 9}, back_right: {r_value: 10, g_value: 11, b_value: 12}}"
  ```
  - the break mode control message:

  ```bash
  ros2 topic pub -1 /percy_break_mode percy_msgs/msg/PercyBreakCtl "{break_mode: 1}"
  ```

  - the power rail command:

  ```bash
  ros2 topic pub -1 /percy_power_rail_cmd percy_msgs/msg/PercyPowerRailCtl "{external_48v: true, external_5v: true, fan_12v: true, jetson1_12v: true, jetson2_12v: true, ultrasonic_sensor_12v: true, camera_12v: true, router_5g_12v: true, switchboard_12v: true, usb_hub_12v: true, sick_system_24v: true}"
  ```

  
