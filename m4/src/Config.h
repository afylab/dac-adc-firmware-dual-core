#pragma once
#include <Arduino.h>
#include <SPI.h>

///////////////////////////////////////////////////////////////
//  Arduino pin initialization - used to appropriately map   //
//  physical GPIO to software data names                     //
///////////////////////////////////////////////////////////////

const int adc_cs_pins[2] = {48, 42}; // SYNC for both 16-bit ADCs
const int dac_cs_pins[4] = {24, 26, 38, 36}; // CS for 4x 20-bit DAC channels
const int ldac = 22; // LDAC pin shared across all AD5791 -- used to synchronize DAC voltage output
const int reset[2] = {46, 44}; // reset pins on ADC
const int drdy[2] = {50, 40}; // data_ready pin for both ADCs -- used as input to indicate ADC conversion has completed
const int led = 7; // indicator LED
const int data = 6; // data indicator LED
const int err = 11; // error indicator LED


const static SPISettings DAC_SPI_SETTINGS(4000000, MSBFIRST, SPI_MODE3);
const static SPISettings ADC_SPI_SETTINGS(4000000, MSBFIRST, SPI_MODE3);

#define NUM_CHANNELS_PER_DAC_BOARD 4
#define NUM_CHANNELS_PER_ADC_BOARD 4
