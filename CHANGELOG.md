# Changelog for percy ros2 

## TODO
- add specific error info instead of error code;
- replace enum strings instead of number of rc info;
- use two thread to deal with callbacks with diffrent frequency and priority
- some instructions have return message,if the instructions have failed then we should log warn info on the screen

## Questions
- how to use restore factory setting
- 0x0 upgrade software and 0x4 firmware version configuration is more suitable for Xingyu Ma to complete

## 1.1 (2023-4-24)
------------------
- test system feedback,light rgb feedback,motion feedback and motor feedback 
- test all the sensor feedback funtion
- motor feedback:when send motor driver motion feedback message for motor1 in virtual can net,the topic info will not change until the same feedback message is send for motor2.
- Contributors: Junyu Luo

## 1.1 (2023-4-20)
------------------
- add light rgb command ros msg files
- test light rgb command message using virtual can network.
- test break control command message
- test power rail control function
- Contributors: Junyu Luo

## 1.1 (2023-4-18)
------------------
- test motion command and light mode control command using virtual can network,this two functionality is ok
- Contributors: Junyu Luo