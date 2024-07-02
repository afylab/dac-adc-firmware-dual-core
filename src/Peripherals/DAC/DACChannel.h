#pragma once

#include <Arduino.h>
#include <Peripherals/PeripheralCommsController.h>

class DACChannel {
 private:
  float gain_error;
  float offset_error;
  int cs_pin;
  int ldac;
  float voltage;
  float voltage_upper_bound;
  float voltage_lower_bound;
  float full_scale = 10.0;
  PeripheralCommsController &commsController;

 public:
  DACChannel(PeripheralCommsController &commsController, int cs_pin, int ldac)
      : commsController(commsController) {
    this->cs_pin = cs_pin;
    this->ldac = ldac;
    offset_error = 0.0;
    gain_error = 1.0;
    voltage = 0.0;
    voltage_upper_bound = full_scale * gain_error + offset_error;
    voltage_lower_bound = -full_scale * gain_error + offset_error;
  }

  // initialize is the command INITIALIZE, setup is called in main::setup
  void initialize() {
    digitalWrite(cs_pin, LOW);
    commsController.sendBytesInTransaction(32, 0, 2);  // Write to control register; Reserved byte; Unclamp DAC from GND
    digitalWrite(cs_pin, HIGH);
    setVoltage(0.0);
  }

  // initialize is the command INITIALIZE, setup is called in main::setup
  void setup() {
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
    pinMode(ldac, OUTPUT);
    digitalWrite(ldac, HIGH);
  }

  float setVoltage(float voltage) {
    byte b1;
    byte b2;
    byte b3;

    voltageToDecimal(voltage / gain_error - offset_error, &b1, &b2, &b3);

    digitalWrite(cs_pin, LOW);
    commsController.sendBytesInTransaction(b1, b2, b3);  // send command byte to DAC; MS data bits, DAC2; LS 8 data bits, DAC2

    digitalWrite(cs_pin, HIGH);

    digitalWrite(ldac, LOW);
    digitalWrite(ldac, HIGH);

    float v = gain_error * (threeByteToVoltage(b1, b2, b3) + offset_error);
    this->voltage = v;
    return v;
  }

  float setVoltageBuffer(float voltage) {
    byte b1;
    byte b2;
    byte b3;

    voltageToDecimal(voltage / gain_error - offset_error, &b1, &b2, &b3);

    digitalWrite(cs_pin, LOW);
    commsController.sendBytesInTransaction(b1, b2, b3);  // send command byte to DAC; MS data bits, DAC2; LS 8 data bits, DAC2
    digitalWrite(cs_pin, HIGH);

    return threeByteToVoltage(b1, b2, b3);
  }

  float getVoltage() { return this->voltage; }

  void setCalibration(float offset, float gain) {
    this->offset_error = offset;
    this->gain_error = gain;
  }

  void setFullScale(float full_scale) {
    full_scale = full_scale;
    voltage_upper_bound = full_scale * gain_error + offset_error;
    voltage_lower_bound = -full_scale * gain_error + offset_error;
  }

  float getLowerBound() { return voltage_lower_bound; }

  float getUpperBound() { return voltage_upper_bound; }

  float getOffsetError() { return offset_error; }
  float getGainError() { return gain_error; }

  float sendCode(int decimal) {
    byte b1;
    byte b2;
    byte b3;

    intToThreeBytes(decimal, &b1, &b2, &b3);

    digitalWrite(cs_pin, LOW);
    commsController.sendBytesInTransaction(b1, b2, b3);  // send command byte to DAC; MS data bits, DAC2; LS 8 data bits, DAC2
    digitalWrite(cs_pin, HIGH);

    digitalWrite(ldac, LOW);

    digitalWrite(ldac, HIGH);

    float v = gain_error * (threeByteToVoltage(b1, b2, b3) + offset_error);

    voltage = v;

    return v;
  }

 private:
  void voltageToDecimal(float voltage, byte *DB1, byte *DB2, byte *DB3) {
    int decimal;
    if (voltage >= 0) {
      decimal = voltage * 524287 / full_scale;
    } else {
      decimal = voltage * 524288 / full_scale + 1048576;
    }
    intToThreeBytes(decimal, DB1, DB2, DB3);
  }

  void intToThreeBytes(int decimal, byte *DB1, byte *DB2, byte *DB3) {
    *DB1 = (byte)((decimal >> 16) | 16);
    *DB2 = (byte)((decimal >> 8) & 255);
    *DB3 = (byte)(decimal & 255);
  }

  // This gives a 16 bit integer (between +/- 2^16)
  int threeByteToInt( byte DB1, byte DB2, byte DB3) {
    return ((int)(((((DB1 & 15) << 8) | DB2) << 8) | DB3));
  }

  float threeByteToVoltage(byte DB1, byte DB2, byte DB3) {
    int decimal;
    float voltage;

    decimal = threeByteToInt(DB1, DB2, DB3);

    if (decimal <= 524287) {
      voltage = decimal * full_scale / 524287;
    } else {
      voltage = -(1048576 - decimal) * full_scale / 524288;
    }
    return voltage;
  }
};