#pragma once

#include <Arduino.h>
#include <Peripherals/PeripheralCommsController.h>

// ADC symbols
// All tables & pages reference AD7734 Data Sheet Rev B (4 Channels, AD7732 is 2
// Channels) Communications Register, Table 11 Summery

// Read/Write bit (R, W), Table 11
#define READ 1 << 6
#define WRITE 0 << 6

// ADC register addresses, Table 11
#define ADDR_COM 0x0
#define ADDR_IO 0x1
#define ADDR_REVISION 0x2
#define ADDR_TEST 0x3
#define ADDR_ADCSTATUS 0x4
#define ADDR_CHECKSUM 0x5
#define ADDR_ADCZEROSCALECAL 0x6
#define ADDR_ADCFULLSCALE 0x7

#define DUMP_MODE 1 << 3

// Address macro functions, returns address for desired register of selected
// channel (0-3), Table 11
#define ADDR_CHANNELDATA(adc_channel) (0x8 + adc_channel)
#define ADDR_CHANNELZEROSCALECAL(adc_channel) (0x10 + adc_channel)
#define ADDR_CHANNELFULLSCALECAL(adc_channel) (0x18 + adc_channel)
#define ADDR_CHANNELSTATUS(adc_channel) (0x20 + adc_channel)
#define ADDR_CHANNELSETUP(adc_channel) (0x28 + adc_channel)
#define ADDR_CHANNELCONVERSIONTIME(adc_channel) (0x30 + adc_channel)
#define ADDR_MODE(adc_channel) (0x38 + adc_channel)

// Operational Mode Register, Table 12
// mode bits (MD2, MD1, MD0 bits)
#define IDLE_MODE 0 << 5
#define CONT_CONV_MODE 1 << 5
#define SINGLE_CONV_MODE 2 << 5
#define PWR_DOWN_MODE 3 << 5
#define ZERO_SCALE_SELF_CAL_MODE 4 << 5
#define CH_ZERO_SCALE_SYS_CAL_MODE 6 << 5
#define CH_FULL_SCALE_SYS_CAL_MODE 7 << 5
#define CH_EN_CONT_CONV 1 << 3

// resolution for 16 bit mode operation, the ADC supports 16 or 24 bit
// resolution.
#define ADCRES16 65535.0
// full scale range, can take 4 different values
#define FSR 20.0
#define ADC2DOUBLE(vin) (FSR * ((double)vin - (ADCRES16 / 2.0)) / ADCRES16)

class ADCBoard {
 private:
  int cs_pin;
  int data_ready_pin;
  int reset_pin;
  PeripheralCommsController &commsController;

  float map2(float x, long in_min, long in_max, float out_min,
             float out_max)  // float
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  int twoByteToInt(byte DB1,
                   byte DB2)  // This gives a 16 bit integer (between +/- 2^16)
  {
    return ((int)((DB1 << 8) | DB2));
  }

  void intToTwoByte(int s, byte *DB1, byte *DB2) {
    *DB1 = ((byte)((s >> 8) & 0xFF));
    *DB2 = ((byte)(s & 0xFF));
  }

  void waitDataReady() {
    int count = 0;
    while (digitalRead(data_ready_pin) == HIGH && count < 2000) {
      count = count + 1;
      delay(1);
    }
  }

 public:
  ADCBoard(PeripheralCommsController &commsController, int cs_pin,
           int data_ready_pin, int reset_pin)
      : cs_pin(cs_pin),
        data_ready_pin(data_ready_pin),
        reset_pin(reset_pin),
        commsController(commsController) {}

  void setup() {
    pinMode(reset_pin, OUTPUT);
    pinMode(data_ready_pin, INPUT);
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);

    // Resets ADC on startup.
    digitalWrite(reset_pin, HIGH);
    digitalWrite(reset_pin, LOW);
    delay(5);
    digitalWrite(reset_pin, HIGH);
  }

  void initialize() {}

  float readVoltage(int channel_index) {
    int statusbyte = 0;
    byte o2;
    byte o3;
    int ovr;
    commsController.beginTransaction();
    digitalWrite(cs_pin, LOW);
    commsController.transfer(0x38 + channel_index);  // Indicates comm register to access
                                     // mode register with channel

    commsController.transfer(0x48);  // Indicates mode register to start
                     // single convertion in dump mode

    waitDataReady();  // Waits until convertion finishes

    commsController.transfer(0x48 + channel_index);  // Indcates comm register to read
                                     // data channel data register

    statusbyte = commsController.receiveByte();  // Reads Channel 'ch' status

    o2 = commsController.receiveByte();  // Reads first byte

    o3 = commsController.receiveByte();  // Reads second byte
    digitalWrite(cs_pin, HIGH);
    commsController.endTransaction();

    ovr = statusbyte & 1;
    switch (ovr) {
      case 0:
        int decimal;
        decimal = twoByteToInt(o2, o3);
        float voltage;
        voltage = map2(decimal, 0, 65536, -10.0, 10.0);
        return voltage;
        break;

      case 1:
        return 0.0;
        break;
    }

    return 0.0;
  }

  float readVoltageNew(int channel_index) {
    startSingleConversion(channel_index);
    waitDataReady();
    uint16_t data = getConversionData(channel_index);
    return convertToVoltage(data);
  }


  // return ADC status register, pg. 16
  uint8_t getADCStatus() {
    uint8_t data_array;

    data_array = READ | ADDR_ADCSTATUS;

    commsController.beginTransaction();
    digitalWrite(cs_pin, LOW);
    commsController.transfer(data_array);
    byte b = commsController.receiveByte();
    digitalWrite(cs_pin, HIGH);
    commsController.endTransaction();
    return b;
  }

  void channelSetup(int adc_channel, uint8_t flags) {

    commsController.beginTransaction();
    digitalWrite(cs_pin, LOW);
    commsController.transfer(WRITE | ADDR_CHANNELSETUP(adc_channel));
    commsController.transfer(flags);
    digitalWrite(cs_pin, HIGH);
    commsController.endTransaction();

  }

  // tells the ADC to start a single conversion on the passed channel
  void startSingleConversion(int adc_channel) {



    commsController.beginTransaction();
    digitalWrite(cs_pin, LOW);
    // setup communication register for writing operation to the mode register
    commsController.transfer(WRITE | ADDR_MODE(adc_channel));
    // setup mode register
    commsController.transfer(SINGLE_CONV_MODE);
    digitalWrite(cs_pin, HIGH);
    commsController.endTransaction();

    // data is ready when _rdy goes low
  }

  // tells the ADC to start a continous conversion on the passed channel
  void startContinuousConversion(int adc_channel) {
    uint8_t data_array[4];

    // address the channel setup register and write to it
    data_array[0] = WRITE | ADDR_CHANNELSETUP(adc_channel);
    data_array[1] = CH_EN_CONT_CONV;

    // address the channel mode register and write to it
    data_array[2] = WRITE | ADDR_MODE(adc_channel);
    data_array[3] = CONT_CONV_MODE;

    // send off command
    commsController.beginTransaction();
    digitalWrite(cs_pin, LOW);
    commsController.transfer(data_array, 4);
    digitalWrite(cs_pin, HIGH);
    commsController.endTransaction();

    // data is ready when _rdy goes low
  }

  void setConversionTime(int adc_channel, int chop, int fw) {
    byte chop_byte = chop == 1 ? 0x80 : 0x00;
    byte send = chop_byte | static_cast<byte>(fw);
    digitalWrite(cs_pin, LOW);
    commsController.beginTransaction();
    commsController.transfer(WRITE | ADDR_CHANNELCONVERSIONTIME(adc_channel));
    commsController.transfer(send);
    commsController.endTransaction();
    digitalWrite(cs_pin, HIGH);
    Serial.println(send);
  }

  uint16_t getConversionData(int adc_channel) {
    byte data_array, upper, lower;

    // setup communication register for reading channel data
    data_array = READ | ADDR_CHANNELDATA(adc_channel);

    // write to the communication register
    commsController.beginTransaction();
    digitalWrite(cs_pin, LOW);
    commsController.transfer(data_array);
    // read upper and lower bytes of channel data register (16 bit mode)
    upper = commsController.receiveByte();
    lower = commsController.receiveByte();
    digitalWrite(cs_pin, HIGH);
    commsController.endTransaction();

    uint16_t result = upper << 8 | lower;

    return result;
  }

  float convertToVoltage(uint16_t data) {
    return ADC2DOUBLE(data);
  }
};