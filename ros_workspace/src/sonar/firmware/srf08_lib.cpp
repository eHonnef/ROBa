//
// SonarSRF
// Arduino Library for controlling SRF sonar sensors
// http://www.arduino.cc/playground/Main/SonarSrf08
//
// MIT License
// Copyright(c) 2009 Zach Foresta
// Copyright(c) 2012 Philipp A. Mohrenweiser
// Copyright(c) 2012-2016 Leo Colombaro
//

// Sensor connections:
// * SDA - Analog pin 4
// * SCL - Analog pin 5

#ifndef SONARSRF08_H
#define SONARSRF08_H

#include <Arduino.h>
#include <Wire.h>

// Read
#define SOFTWARE_REVISION 0x00
#define LIGHT_SENSOR 0x01
// Write
#define COMMAND_REGISTER 0x00
#define MAX_GAIN_REGISTER 0x01
#define RANGE_REGISTER 0x02
// Units
#define INCHES 0x50
#define CENTIMETERS 0x51
#define MICROSECONDS 0x52

class SonarSRF08 {
private:
  uint8_t _address;
  uint8_t _gainRegister;
  uint8_t _rangeLocation;

public:
  SonarSRF08(int address, int gainRegister = 0, int rangeLocation = 0) {
    _address = (uint8_t)(address);
    _gainRegister = (uint8_t)(gainRegister);
    _rangeLocation = (uint8_t)(rangeLocation);
  }

  void begin(void) { Wire.begin(); }

  void writeAddress(unsigned int newAddress) {
    write(0xA0);
    write(0xAA);
    write(0xA5);
    write(newAddress << 1);
  }

  uint16_t readRange(char unit = 'c', bool andStart = true) {
    if (andStart) {
      writeUnit(unit);
      waitForCompletion();
    }

    return (uint16_t)(read(RANGE_REGISTER, 2));
  }

  int8_t readVersion(void) { return (int8_t)(read(SOFTWARE_REVISION, 1)); }

  uint8_t readLuminosity(void) {
    writeUnit('i');
    delay(70);

    return (uint8_t)(read(LIGHT_SENSOR, 1));
  }

  void write(unsigned int command, unsigned int addressRegister = COMMAND_REGISTER) {
    Wire.beginTransmission(_address); // Start I2C transmission
    Wire.write((uint8_t)(addressRegister));
    if (command != 0) {
      Wire.write((uint8_t)(command)); // Send command
      if (_gainRegister && _rangeLocation) {
        Wire.write(_gainRegister);  // SRF Location 1
        Wire.write(_rangeLocation); // SRF Location 2
      }
    }
    Wire.endTransmission(); // End I2C transmission
  }

  void writeUnit(char unit) {
    switch (unit) {
    case 'i':
      write(INCHES);
      break;
    case 'c':
      write(CENTIMETERS);
      break;
    case 'm':
      write(MICROSECONDS);
      break;
    default:
      Serial.println("Invalid units entered... using micro-seconds");
      write(MICROSECONDS);
    }
  }

  int read(unsigned int command, unsigned int length) {
    write(0x00, command);
    Wire.requestFrom(_address, (uint8_t)(length)); // Request length bytes
    while (Wire.available() < length)
      ;      // Wait for result while bytes available
    int res; // Read the bytes, and combine them into one int
    for (; length > 0; length--) {
      res += Wire.read() << (8 * (length - 1));
    }

    return res;
  }

  void waitForCompletion(void) {
    while (readVersion() == -1) {
      delay(1);
    }
  }
};

#endif // SONARSRF_H
