//   This program is template code for programming small esp32 powered wifi controlled robots.
//   https://github.com/rcmgames/RCMv2
//   for information see this page: https://github.com/RCMgames

/**
UNCOMMENT ONE OF THE FOLLOWING LINES DEPENDING ON WHAT HARDWARE YOU ARE USING
Remember to also choose the "environment" for your microcontroller in PlatformIO
*/
// #define RCM_HARDWARE_VERSION RCM_ORIGINAL // versions 1, 2, 3, and 3.1 of the original RCM hardware // https://github.com/RCMgames/RCM_hardware_documentation_and_user_guide
// #define RCM_HARDWARE_VERSION RCM_4_V1 // version 1 of the RCM 4 // https://github.com/RCMgames/RCM-Hardware-V4
// #define RCM_HARDWARE_VERSION RCM_BYTE_V2 // version 2 of the RCM BYTE // https://github.com/RCMgames/RCM-Hardware-BYTE
// #define RCM_HARDWARE_VERSION RCM_NIBBLE_V1 // version 1 of the RCM Nibble // https://github.com/RCMgames/RCM-Hardware-Nibble
// #define RCM_HARDWARE_VERSION RCM_D1_V1 // version 1 of the RCM D1 // https://github.com/RCMgames/RCM-Hardware-D1
// #define RCM_HARDWARE_VERSION ALFREDO_NOU2_NO_VOLTAGE_MONITOR // voltageComp will always report 10 volts https://www.alfredosys.com/products/alfredo-nou2/
// #define RCM_HARDWARE_VERSION ALFREDO_NOU2_WITH_VOLTAGE_MONITOR // modified to add resistors VIN-30k-D36-10k-GND https://www.alfredosys.com/products/alfredo-nou2/
#define RCM_HARDWARE_VERSION ALFREDO_NOU3 // https://www.alfredosys.com/products/alfredo-nou3/

/**
uncomment one of the following lines depending on which communication method you want to use
*/
#define RCM_COMM_METHOD RCM_COMM_EWD // use the normal communication method for RCM robots
// #define RCM_COMM_METHOD RCM_COMM_ROS // use the ROS communication method

#include "rcm.h" //defines pins

// set up motors and anything else you need here
// See this page for how to set up servos and motors for each type of RCM board:
// https://github.com/RCMgames/useful-code/tree/main/boards
// See this page for information about how to set up a robot's drivetrain using the JMotor library
// https://github.com/joshua-8/JMotor/wiki/How-to-set-up-a-drivetrain

JMotorDriverPCA9685HBridge leftMotorDriver = JMotorDriverPCA9685HBridge(motor3port, true); // reverse motor
JMotorDriverPCA9685HBridge rightMotorDriver = JMotorDriverPCA9685HBridge(motor2port);
JEncoderQuadratureAttachInterrupt leftEncoder = JEncoderQuadratureAttachInterrupt(encoder3port, .000088, true); // meters, reverse
JEncoderQuadratureAttachInterrupt rightEncoder = JEncoderQuadratureAttachInterrupt(encoder2port, .000088); // meters
JMotorCompStandardConfig ttmotorConfig = JMotorCompStandardConfig(1.232, .1, 2.31, .7, 5.39, 1.3, 100);
JMotorCompStandard leftMotorCompensator = JMotorCompStandard(voltageComp, ttmotorConfig);
JMotorCompStandard rightMotorCompensator = JMotorCompStandard(voltageComp, ttmotorConfig);
JControlLoopBasic leftControlLoop = JControlLoopBasic(30, 1000);
JControlLoopBasic rightControlLoop = JControlLoopBasic(30, 1000);
JMotorControllerClosed leftMotor = JMotorControllerClosed(leftMotorDriver, leftMotorCompensator, leftEncoder, leftControlLoop, INFINITY, INFINITY, .025);
JMotorControllerClosed rightMotor = JMotorControllerClosed(rightMotorDriver, rightMotorCompensator, rightEncoder, rightControlLoop, INFINITY, INFINITY, .025);
JDrivetrainTwoSide drivetrain = JDrivetrainTwoSide(leftMotor, rightMotor, 0.17);
JTwoDTransform driveControl = JTwoDTransform({ 0, 0, 0 });

void Enabled()
{
    // code to run while enabled, put your main code here
    drivetrain.setVel(JDeadzoneRemover::calculate(driveControl, { 0, 0, 0 }, drivetrain.getMaxVel(), { 0.01, 0.01, 0.01 }));
}

void Enable()
{
    // turn on outputs
    drivetrain.enable();
}

void Disable()
{
    // turn off outputs
    drivetrain.disable();
}

jENCODER_MAKE_ISRS_MACRO(leftEncoder);
jENCODER_MAKE_ISRS_MACRO(rightEncoder);

void PowerOn()
{
    // runs once on robot startup, set pin modes and use begin() if applicable here
    leftEncoder.setUpInterrupts(leftEncoder_jENCODER_ISR_A, leftEncoder_jENCODER_ISR_B);
    rightEncoder.setUpInterrupts(rightEncoder_jENCODER_ISR_A, rightEncoder_jENCODER_ISR_B);
}

void Always()
{
    // always runs if void loop is running, JMotor run() functions should be put here
    // (but only the "top level", for example if you call drivetrainController.run() you shouldn't also call leftMotorController.run())
    drivetrain.run();
    delay(1);
}

#if RCM_COMM_METHOD == RCM_COMM_EWD
void WifiDataToParse()
{
    enabled = EWD::recvBl();
    // add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)
    // receive values for all the variables (you can put this in WifiDataToParse())
    float turn = -EWD::recvFl();
    float forward = EWD::recvFl();
    driveControl = JTwoDTransform({ forward, 0, turn });
}
void WifiDataToSend()
{
    EWD::sendFl(voltageComp.getSupplyVoltage());
    // add data to send here: (EWD::sendBl(), EWD::sendBy(), EWD::sendIn(), EWD::sendFl())(boolean, byte, int, float)
    EWD::sendFl(leftEncoder.getVel());
    EWD::sendFl(rightEncoder.getVel());
}

void configWifi()
{
    EWD::mode = EWD::Mode::connectToNetwork;
    EWD::routerName = "router";
    EWD::routerPassword = "password";
    EWD::routerPort = 25210;

    // EWD::mode = EWD::Mode::createAP;
    // EWD::APName = "rcm0";
    // EWD::APPassword = "rcmPassword";
    // EWD::APPort = 25210;
}
#elif RCM_COMM_METHOD == RCM_COMM_ROS ////////////// ignore everything below this line unless you're using ROS mode /////////////////////////////////////////////
void ROSWifiSettings()
{
    // on a computer run: sudo docker run -it --rm --net=host microros/micro-ros-agent:iron udp4 --port 8888
    WiFi.disconnect(true, true);
    delay(100);
    WiFi.begin("router", "password");
    WiFi.setTxPower(WIFI_POWER_8_5dBm); // fix for wifi on nou3 thanks @torchtopher from mini FRC
    set_microros_wifi_transports(nullptr, nullptr, "10.0.0.171", 8888); // doesn't complete until it connects to the wifi network
    nodeName = "nou3_robot";
    // numSubscribers = 10; //change max number of subscribers
}

#include <example_interfaces/msg/bool.h>
#include <std_msgs/msg/byte.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
// and lots of other message types are available (see file available_ros2_types)
#include <geometry_msgs/msg/twist.h>

// declare publishers
declarePub(battery, std_msgs__msg__Float32);

// declare subscribers and write callback functions
declareSubAndCallback(cmd_vel, geometry_msgs__msg__Twist);
driveControl.x = cmd_velMsg->linear.x;
driveControl.y = cmd_velMsg->linear.y;
driveControl.theta = cmd_velMsg->angular.z;
} // end of callback

void ROSbegin()
{
    // create publishers
    createPublisher(battery, std_msgs__msg__Float32, "/rcm/battery");
    batteryMsg.data = 0;

    // add subscribers
    addSub(cmd_vel, geometry_msgs__msg__Twist, "/cmd_vel");
}

void ROSrun()
{
    rosSpin(1);
    // you can add more publishers here
    batteryMsg.data = voltageComp.getSupplyVoltage();
    publish(battery);
}
#endif

#include "rcmutil.h"
