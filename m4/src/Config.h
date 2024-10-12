#pragma once
#include <Arduino.h>
#include <SPI.h>

///////////////////////////////////////////////////////////////
//  Arduino pin initialization - used to appropriately map   //
//  physical GPIO to software data names                     //
///////////////////////////////////////////////////////////////

// const int NUM_ADC_BOARDS = 2;
// const int NUM_DAC_CHANNELS = 4;
// const int adc_cs_pins[NUM_ADC_BOARDS] = {48, 42}; // SYNC for both 24-bit ADCs
// const int dac_cs_pins[NUM_DAC_CHANNELS] = {24, 26, 38, 36}; // CS for 4x 20-bit DAC channels
// const int ldac = 22; // LDAC pin shared across all AD5791 -- used to synchronize DAC voltage output
// const int reset[NUM_ADC_BOARDS] = {46, 44}; // reset pins on ADC
// const int drdy[NUM_ADC_BOARDS] = {50, 40}; // data_ready pin for both ADCs -- used as input to indicate ADC conversion has completed
// const int led = 7; // indicator LED
// const int data = 6; // data indicator LED
// const int err = 11; // error indicator LED


const int NUM_ADC_BOARDS = 4;
const int NUM_DAC_CHANNELS = 16;
const int adc_cs_pins[NUM_ADC_BOARDS] = {39,40,41,42};
const int dac_cs_pins[NUM_DAC_CHANNELS] = {23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38};
const int ldac = 22;
const int reset[NUM_ADC_BOARDS] = {43,44,45,46};
const int drdy[NUM_ADC_BOARDS] = {47,48,49,50};
const int adc_sync = 51;
const int led = 7; // indicator LED
const int data = 6; // data indicator LED
const int err = 11; // error indicator LED



const int SPI_SPEED = 4000000; // SPI speed for both ADC and DAC

const static SPISettings DAC_SPI_SETTINGS(35000000, MSBFIRST, SPI_MODE1);
const static SPISettings ADC_SPI_SETTINGS(8000000, MSBFIRST, SPI_MODE3);

#define NUM_CHANNELS_PER_DAC_BOARD 4
#define NUM_CHANNELS_PER_ADC_BOARD 4
