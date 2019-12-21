//
// SonarSRF
// Arduino Library for controlling SRF sonar sensors
// http://www.arduino.cc/playground/Main/SonarSrf08
//
// MIT License
// Copyright(c) 2009 Zach Foresta
// Copyright(c) 2012 Leo Colombaro
// Copyright(c) 2012 Philipp A. Mohrenweiser
//
#include <Wire.h>

#include <SonarSRF08.hpp> //sonar

// following are IMU part
#include <LIS3MDL.hpp> //magnetometer
#include <LSM6.hpp>    //accelerometer

// ros includes
#include <ros.h>
#include <std_msgs/Int32.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt16.h>

void distance(String reference, int sensorReading);
void isConnected(String reference, int sensorSoft);

LIS3MDL mag;
LSM6 imu;

#define MAIN_08_ADDRESS (0xF8 >> 1) // address for Sonar but only one Sonar is work as well
SonarSRF08 MainSonar;

// Setup Analogue Gain
// http://www.robot-electronics.co.uk/htm/srf08tech.html section "Analogue Gain"
#define GAIN_REGISTER 0x09

// Setup Range Location
// http://www.robot-electronics.co.uk/htm/srf08tech.html section "Changing the Range"
#define LOCATION_REGISTER 0x8C

char unit = 'c'; // 'i' for inches, 'c' for centimeters, 'm' for micro-seconds
char report[80];
char info[80];

ros::NodeHandle nh;

// sonar node
std_msgs::Int32 sonar_msg;
ros::Publisher sonar_pub("sonar", &sonar_msg);

// magnetometer node
std_msgs::UInt16MultiArray mag_msg;
ros::Publisher mag_pub("magnetometer", &mag_msg);

// IMU accelerometer node
std_msgs::UInt16MultiArray accel_msg;
ros::Publisher accel_pub("IMU_accelerometer", &accel_msg);

// IMU gyro node
std_msgs::UInt16MultiArray gyro_msg;
ros::Publisher gyro_pub("IMU_gyro", &gyro_msg);

void setup() {
  // init node handle
  nh.initNode();

  // serial port initializer
  //Serial.begin(9600);

  // initializing the main sonar and checking if it's connected
  MainSonar.connect(MAIN_08_ADDRESS, GAIN_REGISTER, LOCATION_REGISTER);

  // check if connected and advertise node
  if (MainSonar.getSoft() >= 0) {
    nh.advertise(sonar_pub);
  }

  Wire.begin();

  // MAG init
  if (mag.init()) {
    mag.enableDefault();

    // init array message
    mag_msg.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension));
    mag_msg.layout.dim[0].label = "magnetometer";
    mag_msg.layout.dim[0].size = 3; // x, y, z values
    mag_msg.layout.dim[0].stride = 1;
    mag_msg.layout.data_offset = 0;
    mag_msg.data = (unsigned int*)malloc(sizeof(int)*8);
    mag_msg.data_length = 3;

    // advertise node
    nh.advertise(mag_pub);
  } else {
    // Serial.println("Failed to detect and initialize magnetometer!");
  }

  // IMU init
  if (imu.init()) {
    imu.enableDefault();

    // init array message (accel)
    accel_msg.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension));
    accel_msg.layout.dim[0].label = "IMU_accelerometer";
    accel_msg.layout.dim[0].size = 3; // x, y, z values
    accel_msg.layout.dim[0].stride = 1;
    accel_msg.layout.data_offset = 0;
    accel_msg.data = (unsigned int*)malloc(sizeof(int)*8);
    accel_msg.data_length = 3;

    // init array message (gyro)
    gyro_msg.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension));
    gyro_msg.layout.dim[0].label = "IMU_gyro";
    gyro_msg.layout.dim[0].size = 3; // x, y, z values
    gyro_msg.layout.dim[0].stride = 1;
    gyro_msg.layout.data_offset = 0;
    gyro_msg.data = (unsigned int*)malloc(sizeof(int)*8);
    gyro_msg.data_length = 3;

    // advertise node
    nh.advertise(gyro_pub);
    nh.advertise(accel_pub);
  } else {
    // Serial.println("Failed to detect and initialize IMU!");
  }
}

void loop() {
  // read sonar data
  sonar_msg.data = MainSonar.getRange(unit);
  sonar_pub.publish(&sonar_msg);

  // read magnetometer data
  mag.read();
  mag_msg.data[0] = mag.m.x;
  mag_msg.data[1] = mag.m.y;
  mag_msg.data[2] = mag.m.z;
  mag_pub.publish(&mag_msg);
  delay(500);

  // read IMU data and split in accel and gyro nodes
  imu.read();
  accel_msg.data[0] = imu.a.x;
  accel_msg.data[1] = imu.a.y;
  accel_msg.data[2] = imu.a.z;
  accel_pub.publish(&accel_msg);

  gyro_msg.data[0] = imu.g.x;
  gyro_msg.data[1] = imu.g.y;
  gyro_msg.data[2] = imu.g.z;
  gyro_pub.publish(&gyro_msg);

  delay(100);
  nh.spinOnce();
}
