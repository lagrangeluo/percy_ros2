# Changelog for percy ros2 

## TODO
- add specific error info instead of error code;
- replace enum strings instead of number of rc info;

## 1.1 (2023-4-24)
------------------
- test system feedback,light rgb feedback,motion feedback and motor feedback 
- test all the sensor feedback funtion
- motor feedback:when send motor driver motion feedback message for motor1 in virtual can net,the topic info will not change until the same feedback message is send for motor2.

## 1.1 (2023-4-20)
------------------
- add light rgb command ros msg files
- test light rgb command message using virtual can network.
- test break control command message
- test power rail control function

## 1.1 (2023-4-18)
------------------
- test motion command and light mode control command using virtual can network,this two functionality is ok
- Contributors: Junyu Luo