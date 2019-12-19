#include <srf08_lib.cpp>
#include <ros.h>
#include <std_msgs/UInt16.h>

void setup();
void loop();

// Set up the ros node and publisher
ros::NodeHandle nh;

std_msgs::UInt16 sonar_msg;
ros::Publisher pub_sonar("sonar", &sonar_msg);

// #define MAIN_08_ADDRESS (0xF8 >> 1)
#define MAIN_08_ADDRESS 0x70

// Setup Analogue Gain
// http://www.robot-electronics.co.uk/htm/srf08tech.html section "Analogue Gain"
#define GAIN_REGISTER 0x09

// Setup Range Location
// http://www.robot-electronics.co.uk/htm/srf08tech.html section "Changing the Range"
#define LOCATION_REGISTER 0x8C

SonarSRF08 MainSonar(MAIN_08_ADDRESS, GAIN_REGISTER, LOCATION_REGISTER);

uint16_t sensorReading = 0;

char unit = 'c'; // 'i' for inches, 'c' for centimeters, 'm' for micro-seconds


void setup() {
  // init ros node
  nh.initNode();
  nh.advertise(pub_sonar);

  MainSonar.begin();
}

void loop() {
  sensorReading = MainSonar.readRange(unit);
  sonar_msg.data = sensorReading;
  pub_sonar.publish(&sonar_msg);
  delay(1000);

  nh.spinOnce();
}
