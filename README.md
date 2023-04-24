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

this part is used for agilex's developers to test the CAN protocol

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
  
- test the feedback messages

  - send the system state feedback message

  ```bash
   cansend vcan0 211#0000000000000000
   cansend vcan0 211#0101323232320001
  ```
  
  ```bash
  ros2 topic echo /percy_status
  vehicle_state: 1
  control_mode: 1
  error_code: 1
  fan_1_percentage: 50
  fan_2_percentage: 50
  fan_3_percentage: 50
  fan_4_percentage: 50
  ```
  
  - send the motion feedback
  
  ```bash
   cansend vcan0 221#03E803E800000000
  ```
  
  ```bash
  ros2 topic echo /percy_status
  linear_velocity: 1.0
  angular_velocity: 1.0
  ```
  
  - send the light state feedback
  
  ```bash
   cansend vcan0 231#0000000000000000
   cansend vcan0 231#0101000100000000
  ```
  ```bash
  ros2 topic echo /percy_status
  light_control_state: 1
  front_light_mode: 1
  back_light_mode: 1
  ```
  
  - send the light rgb feedback
  
  ```bash
   cansend vcan0 232#0001010101010100
   cansend vcan0 233#0001010101010100
  ```
  
  ```bash
  ros2 topic echo /percy_status
  front_left:
    r_value: 1
    g_value: 1
    b_value: 1
  front_right:
    r_value: 1
    g_value: 1
    b_value: 1
    ...(the same with back_left and back_right)
  ```
  
  - send motor state feedback
  
  ```bash
  cansend vcan0 251#000C0064000004d2
  cansend vcan0 261#00F0000000000100
  ```
  
  - send rc feedback
  
  ```bash
  cansend vcan0 241#AA32323232320000
  ```
  
  ```bash
  ros2 topic echo /percy_status
  rc_state:
    swa: 0
    swb: 0
    swc: 0
    swd: 0
    stick_right_v: 50
    stick_right_h: 50
    stick_left_v: 50
    stick_left_h: 50
    var_a: 0
  ```
  
  - send BMS feedback
  
  ```bash
  cansend vcan0 361#323204D204D2012C
  ```
  
  ```bash
  ros2 topic echo /percy_status
  bms_state:
    soc: 50
    soh: 50
    battery_voltage: 123.4000015258789
    battery_current: 123.4000015258789
    battery_temp: 30
  ```
  
  - send power button feedback and mechanical calibration feedback
  
  ```bash
  cansend vcan0 371#0200000000000000
  cansend vcan0 381#000004D2000004D2
  ```
  
  ```bash
  ros2 topic echo /percy_status
  powerbutton: 2
  wheel_circumference: 1234
  wheel_track: 1234
  ```
  
  

