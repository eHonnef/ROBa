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

// Sensor connections:
// * SDA - Analog pin 4
// * SCL - Analog pin 5

#ifndef SonarSRF08_h
#define SonarSRF08_h

#include <Arduino.h>
#include <Wire.h>

#define INCHES 0x50
#define CENTIMETERS 0x51
#define MICROSECONDS 0x52
#define COMMAND_REGISTER (byte)0x00
#define RESULT_REGISTER 0x02

class SonarSRF08 {
public:
  void connect(int address, int gainRegister = NULL, int rangeLocation = NULL) {
    _address = address;
    _gainRegister = gainRegister;
    _rangeLocation = rangeLocation;
    // start I2C bus
    Wire.begin();
  }

  // Sets Units for display / storage
  void startRanging(char unit) {
    switch (unit) {
    case 'i':
      sendCommand(INCHES);
      break;
    case 'c':
      sendCommand(CENTIMETERS);
      break;
    case 'm':
      sendCommand(MICROSECONDS);
      break;
    default:
      Serial.println("Invalid units entered... using micro-seconds");
      sendCommand(MICROSECONDS);
    }
  }

  // Read data from register return result
  int getRange(char unit = 'c', bool andStart = true) {
    int result = 0; // the result is two bytes long
    if (andStart) {
      startRanging(unit);
      waitForCompletion();
    }
    sendCommand(NULL, RESULT_REGISTER);
    Wire.requestFrom(_address, 2);
    // wait for two bytes to return
    while (Wire.available() < 2)
      ; // wait for result
    // read the two bytes, and combine them into one int
    byte highByte = Wire.read(); // Stores high byte from ranging
    byte lowByte = Wire.read();  // Stored low byte from ranging
    result = (highByte << 8) + lowByte;
    // return the result:
    return result;
  }

  // Get software revision
  int getSoft() {
    sendCommand();
    Wire.requestFrom(_address, 1); // Request 1 byte
    while (Wire.available() < 0)
      ;                         // While byte available
    int software = Wire.read(); // Get byte
    return software;
  }

  void changeAddress(int newAddress) {
    // The address given in Arduino 7bit has to be converted back into SRF 8bit
    // newAddress << 1 can be set to any of E0, E2, E4, E6, E8, EA, EC, EE
    // F0, F2, F4, F6, F8, FA, FC, FE
    sendCommand(0xA0);
    sendCommand(0xAA);
    sendCommand(0xA5);
    sendCommand(newAddress << 1);
  }

protected:
  void waitForCompletion() {
    while (getSoft() == -1) {
      delay(1);
    }
  }

  // Communicates with Sonar to send commands
  void sendCommand(int command = NULL, int addressRegister = COMMAND_REGISTER) {
    // start I2C transmission
    Wire.beginTransmission(_address);
    // send command
    Wire.write(addressRegister); // SRF Location 0
    if (command != NULL) {
      Wire.write(command); // SRF Command
      if (_gainRegister && _rangeLocation) {
        Wire.write(_gainRegister);  // SRF Location 1
        Wire.write(_rangeLocation); // SRF Location 2
      }
    }
    // end I2C transmission
    Wire.endTransmission();
  }

  int _address;
  int _gainRegister;
  int _rangeLocation;
};

#endif
