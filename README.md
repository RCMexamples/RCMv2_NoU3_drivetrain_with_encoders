# RCMv2
https://github.com/rcmgames/RCMv2
### open in VSCode (using [PlatformIO](https://platformio.org/platformio-ide)), or use the Arduino IDE.

## This is an example program for how to use the RCMv2 software and an Alfredo Systems NoU3 to control a robot that has two motors with encoders on the drivetrain. 

### Use the arcadedrive.txt file with RCMDS-new

https://github.com/rcmgames/RCMDS-new

### Or, use the following instructions to control the robot with ROS2

install ros2 and micro ros

run ifconfig on the ros system and put the ip address in set_microros_wifi_transports() and the network ssid and password in WiFi.begin() in ROSWifiSettings()

upload the code with #define RCM_COMM_METHOD RCM_COMM_ROS uncommented

in terminal on ros system run: sudo docker run -it --rm --net=host microros/micro-ros-agent:iron udp4 --port 8888

turn on robot, and check that things print in the ros terminal

in new terminal on ros system run: ros2 topic pub /rcm/enabled example_interfaces/Bool 'data: True'

in new terminal on ros system run: ros2 topic echo /rcm/battery

in new terminal on ros system run: ros2 run teleop_twist_keyboard teleop_twist_keyboard


## find more information about the Robot Control Module hardware here: [https://github.com/rcmgames](https://github.com/rcmgames?view_as=public)

## compatible with
* [Alfredo Systems NoU3](https://www.alfredosys.com/products/alfredo-nou3/)

### Libraries used:
* [JMotor](https://github.com/joshua-8/JMotor) library for motor control
* [micro-ROS](https://micro.ros.org/) as optional method of communication
* [ESP32_easy_wifi_data](https://github.com/joshua-8/ESP32_easy_wifi_data) as optional method of communication
