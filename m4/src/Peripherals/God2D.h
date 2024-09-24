#pragma once

#include <Arduino.h>
#include <Peripherals/ADC/ADCController.h>
#include <Peripherals/DAC/DACController.h>

#include "Config.h"
#include "Utils/TimingUtil.h"
#include "Utils/shared_memory.h"
#include "unordered_set"

class God2D
{
public:
  static void setup() { initializeRegistry(); }

  static void initializeRegistry()
  {
    registerMemberFunctionVector(timeSeriesBufferRamp2D, "2D_TIME_SERIES_BUFFER_RAMP");
    registerMemberFunctionVector(dacLedBufferRamp2D, "2D_DAC_LED_BUFFER_RAMP");
  }

  // timeSeriesBufferRamp2D:
  // Arguments (in order):
  // numDacChannels, numAdcChannels, numStepsFast, numStepsSlow,
  // dacInterval_us, adcInterval_us, retrace (0.0f = false, 1.0f = true),
  // numFastDacChannels, [fastDacChannelID, fastDacV0, fastDacVf] *
  // numFastDacChannels, numSlowDacChannels, [slowDacChannelID, slowDacV0,
  // slowDacVf] * numSlowDacChannels, [adcChannelID] * numAdcChannels
  static OperationResult timeSeriesBufferRamp2D(
      const std::vector<float> &args)
  {
    // Minimum required arguments:
    // 7 initial params + at least 1 fast DAC channel + 1 slow DAC channel + ADC
    // channels
    if (args.size() < 7 + 3 + 3 + 1)
    {
      return OperationResult::Failure(
          "Not enough arguments provided for 2D ramp");
    }

    size_t currentIndex = 0;

    // Parse initial parameters
    int numDacChannels = static_cast<int>(args[currentIndex++]);
    int numAdcChannels = static_cast<int>(args[currentIndex++]);
    int numStepsFast = static_cast<int>(args[currentIndex++]);
    int numStepsSlow = static_cast<int>(args[currentIndex++]);
    uint32_t dac_interval_us = static_cast<uint32_t>(args[currentIndex++]);
    uint32_t adc_interval_us = static_cast<uint32_t>(args[currentIndex++]);
    bool retrace =
        static_cast<bool>(args[currentIndex++]); // 0.0f = false, 1.0f = true

    // Parse Fast DAC Channels
    if (currentIndex >= args.size())
    {
      return OperationResult::Failure(
          "Unexpected end of arguments while parsing fast DAC channels");
    }
    int numFastDacChannels = static_cast<int>(args[currentIndex++]);
    if (args.size() < currentIndex + numFastDacChannels * 3)
    {
      return OperationResult::Failure(
          "Not enough arguments for fast DAC channels");
    }

    int *fastDacChannels = new int[numFastDacChannels];
    float *fastDacV0s = new float[numFastDacChannels];
    float *fastDacVfs = new float[numFastDacChannels];

    for (int i = 0; i < numFastDacChannels; ++i)
    {
      fastDacChannels[i] = static_cast<int>(args[currentIndex++]);
      fastDacV0s[i] = args[currentIndex++];
      fastDacVfs[i] = args[currentIndex++];
    }

    // Parse Slow DAC Channels
    if (currentIndex >= args.size())
    {
      // Clean up allocated memory before returning
      delete[] fastDacChannels;
      delete[] fastDacV0s;
      delete[] fastDacVfs;
      return OperationResult::Failure(
          "Unexpected end of arguments while parsing slow DAC channels");
    }
    int numSlowDacChannels = static_cast<int>(args[currentIndex++]);
    if (args.size() < currentIndex + numSlowDacChannels * 3)
    {
      // Clean up allocated memory before returning
      delete[] fastDacChannels;
      delete[] fastDacV0s;
      delete[] fastDacVfs;
      return OperationResult::Failure(
          "Not enough arguments for slow DAC channels");
    }

    int *slowDacChannels = new int[numSlowDacChannels];
    float *slowDacV0s = new float[numSlowDacChannels];
    float *slowDacVfs = new float[numSlowDacChannels];

    for (int i = 0; i < numSlowDacChannels; ++i)
    {
      slowDacChannels[i] = static_cast<int>(args[currentIndex++]);
      slowDacV0s[i] = args[currentIndex++];
      slowDacVfs[i] = args[currentIndex++];
    }

    // Parse ADC Channels
    if (args.size() < currentIndex + numAdcChannels)
    {
      // Clean up allocated memory before returning
      delete[] fastDacChannels;
      delete[] fastDacV0s;
      delete[] fastDacVfs;
      delete[] slowDacChannels;
      delete[] slowDacV0s;
      delete[] slowDacVfs;
      return OperationResult::Failure("Not enough arguments for ADC channels");
    }

    int *adcChannels = new int[numAdcChannels];
    for (int i = 0; i < numAdcChannels; ++i)
    {
      adcChannels[i] = static_cast<int>(args[currentIndex++]);
    }

    // Validate total number of DAC channels
    if (numFastDacChannels + numSlowDacChannels != numDacChannels)
    {
      // Clean up allocated memory before returning
      delete[] fastDacChannels;
      delete[] fastDacV0s;
      delete[] fastDacVfs;
      delete[] slowDacChannels;
      delete[] slowDacV0s;
      delete[] slowDacVfs;
      delete[] adcChannels;
      return OperationResult::Failure(
          "Sum of fast and slow DAC channels does not match numDacChannels");
    }

    // Allocate memory for slow DAC voltage setpoints
    float **slowVoltSetpoints = new float *[numSlowDacChannels];
    for (int i = 0; i < numSlowDacChannels; ++i)
    {
      slowVoltSetpoints[i] = new float[numStepsSlow];
      for (int j = 0; j < numStepsSlow; ++j)
      {
        slowVoltSetpoints[i][j] =
            slowDacV0s[i] +
            (slowDacVfs[i] - slowDacV0s[i]) * j / (numStepsSlow - 1);
      }
    }

    // Start continuous ADC conversions
    for (int i = 0; i < numAdcChannels; i++)
    {
      ADCController::startContinuousConversion(adcChannels[i]);
    }

    // Iterate over slow steps with optional retrace
    for (int slowStep = 0; slowStep < numStepsSlow && !getStopFlag();
         ++slowStep)
    {
      // Set slow DAC channels to the current slow step voltages
      DACChannel::commsController.beginTransaction();
      for (int i = 0; i < numSlowDacChannels; ++i)
      {
        DACController::setVoltageNoTransaction(slowDacChannels[i],
                                               slowVoltSetpoints[i][slowStep]);
      }
      DACController::toggleLdac();
      DACChannel::commsController.endTransaction();

      // Determine ramp direction based on retrace flag
      bool isReverse = false;
      if (retrace)
      {
        isReverse = (slowStep % 2 != 0); // Reverse on odd slow steps
      }

      // Prepare ramp voltages
      float *currentV0s = fastDacV0s;
      float *currentVfs = fastDacVfs;
      if (isReverse)
      {
        // Swap V0 and Vf for reverse ramp
        currentV0s = new float[numFastDacChannels];
        currentVfs = new float[numFastDacChannels];
        for (int i = 0; i < numFastDacChannels; ++i)
        {
          currentV0s[i] = fastDacVfs[i];
          currentVfs[i] = fastDacV0s[i];
        }
      }

      // Call the base ramp function for fast channels
      OperationResult rampResult = timeSeriesBufferRampBaseNoConversionSetup(
          numFastDacChannels, numAdcChannels, numStepsFast, dac_interval_us,
          adc_interval_us, fastDacChannels, currentV0s, currentVfs,
          adcChannels);

      // If reverse ramp was performed, clean up the temporary arrays
      if (isReverse)
      {
        delete[] currentV0s;
        delete[] currentVfs;
      }

      if (!rampResult.isSuccess())
      {
        // Clean up allocated memory before returning
        for (int i = 0; i < numSlowDacChannels; ++i)
        {
          delete[] slowVoltSetpoints[i];
        }
        delete[] slowVoltSetpoints;
        delete[] fastDacChannels;
        delete[] fastDacV0s;
        delete[] fastDacVfs;
        delete[] slowDacChannels;
        delete[] slowDacV0s;
        delete[] slowDacVfs;
        delete[] adcChannels;
        return rampResult; // Return the failure reason
      }
    }

    // Set ADC channels to idle mode
    for (int i = 0; i < numAdcChannels; i++)
    {
      ADCController::idleMode(adcChannels[i]);
    }

    // Clean up allocated memory
    for (int i = 0; i < numSlowDacChannels; i++)
    {
      delete[] slowVoltSetpoints[i];
    }
    delete[] slowVoltSetpoints;
    delete[] fastDacChannels;
    delete[] fastDacV0s;
    delete[] fastDacVfs;
    delete[] slowDacChannels;
    delete[] slowDacV0s;
    delete[] slowDacVfs;
    delete[] adcChannels;

    if (getStopFlag())
    {
      setStopFlag(false);
      return OperationResult::Failure("2D RAMPING_STOPPED");
    }

    return OperationResult::Success();
  }

  static OperationResult timeSeriesBufferRampBaseNoConversionSetup(
      int numDacChannels, int numAdcChannels, int numSteps,
      uint32_t dac_interval_us, uint32_t adc_interval_us, int *dacChannels,
      float *dacV0s, float *dacVfs, int *adcChannels)
  {
    if (adc_interval_us < 1 || dac_interval_us < 1)
    {
      return OperationResult::Failure("Invalid interval");
    }
    if (numSteps < 1)
    {
      return OperationResult::Failure("Invalid number of steps");
    }
    if (numDacChannels < 1 || numAdcChannels < 1)
    {
      return OperationResult::Failure("Invalid number of channels");
    }
    // uint32_t adc_comms_period_us = (1.0/SPI_SPEED)*1e6*8*4; // 8 bits per
    // byte, 4 bytes per ADC conversion if (adc_interval_us <
    // adc_comms_period_us*numAdcChannels) {
    //   return OperationResult::Failure("ADC interval too short");
    // }
    // uint32_t dac_comms_period_us = (1.0/SPI_SPEED)*1e6*8*3; // 8 bits per
    // byte, 3 bytes per DAC update if (dac_interval_us <
    // dac_comms_period_us*numDacChannels) {
    //   return OperationResult::Failure("DAC interval too short");
    // }

    int steps = 0;
    int x = 0;

    const int saved_data_size = numSteps * dac_interval_us / adc_interval_us;

    float **voltSetpoints = new float *[numDacChannels];

    for (int i = 0; i < numDacChannels; i++)
    {
      voltSetpoints[i] = new float[numSteps];
      for (int j = 0; j < numSteps; j++)
      {
        voltSetpoints[i][j] =
            dacV0s[i] + (dacVfs[i] - dacV0s[i]) * j / (numSteps - 1);
      }
    }

    TimingUtil::setupTimersTimeSeries(dac_interval_us, adc_interval_us);

    while (x < saved_data_size && !getStopFlag())
    {
      if (TimingUtil::adcFlag)
      {
        ADCBoard::commsController.beginTransaction();
        if (steps <= 1)
        {
          for (int i = 0; i < numAdcChannels; i++)
          {
            ADCController::getVoltageDataNoTransaction(adcChannels[i]);
          }
        }
        else
        {
          VoltagePacket *packets = new VoltagePacket[numAdcChannels];
          for (int i = 0; i < numAdcChannels; i++)
          {
            float v =
                ADCController::getVoltageDataNoTransaction(adcChannels[i]);
            packets[i] = {static_cast<uint8_t>(adcChannels[i]),
                          static_cast<uint32_t>(x), v};
          }
          m4SendVoltage(packets, numAdcChannels);
          delete[] packets;
          x++;
        }
        ADCBoard::commsController.endTransaction();
        TimingUtil::adcFlag = false;
      }
      if (TimingUtil::dacFlag && steps < numSteps + 1)
      {
        DACChannel::commsController.beginTransaction();
        if (steps == 0)
        {
          for (int i = 0; i < numDacChannels; i++)
          {
            DACController::setVoltageNoTransaction(dacChannels[i],
                                                   voltSetpoints[i][0]);
          }
        }
        else
        {
          for (int i = 0; i < numDacChannels; i++)
          {
            DACController::setVoltageNoTransaction(dacChannels[i],
                                                   voltSetpoints[i][steps - 1]);
          }
        }
        DACController::toggleLdac();
        DACChannel::commsController.endTransaction();
        steps++;
        TimingUtil::dacFlag = false;
      }
    }

    TimingUtil::disableDacInterrupt();
    TimingUtil::disableAdcInterrupt();

    for (int i = 0; i < numDacChannels; i++)
    {
      delete[] voltSetpoints[i];
    }
    delete[] voltSetpoints;

    if (getStopFlag())
    {
      setStopFlag(false);
      return OperationResult::Failure("RAMPING_STOPPED");
    }

    return OperationResult::Success();
  }

  // dacLedBufferRamp2D:
  // Arguments (in order):
  // numDacChannels, numAdcChannels, numStepsFast, numStepsSlow,
  // dacInterval_us, dacSettlingTime_us, retrace (0.0f = false, 1.0f = true), numAdcAverages
  // numFastDacChannels, [fastDacChannelID, fastDacV0, fastDacVf] *
  // numFastDacChannels, numSlowDacChannels, [slowDacChannelID, slowDacV0,
  // slowDacVf] * numSlowDacChannels, [adcChannelID] * numAdcChannels
  static OperationResult dacLedBufferRamp2D(
      const std::vector<float> &args)
  {
    // Minimum required arguments:
    // 7 initial params + at least 1 fast DAC channel + 1 slow DAC channel + ADC
    // channels
    if (args.size() < 7 + 3 + 3 + 1)
    {
      return OperationResult::Failure(
          "Not enough arguments provided for 2D ramp");
    }

    size_t currentIndex = 0;

    // Parse initial parameters
    int numDacChannels = static_cast<int>(args[currentIndex++]);
    int numAdcChannels = static_cast<int>(args[currentIndex++]);
    int numStepsFast = static_cast<int>(args[currentIndex++]);
    int numStepsSlow = static_cast<int>(args[currentIndex++]);
    uint32_t dac_interval_us = static_cast<uint32_t>(args[currentIndex++]);
    uint32_t dac_settling_time_us = static_cast<uint32_t>(args[currentIndex++]);
    bool retrace =
        static_cast<bool>(args[currentIndex++]); // 0.0f = false, 1.0f = true
    int numAdcAverages = static_cast<int>(args[currentIndex++]);

    // Parse Fast DAC Channels
    if (currentIndex >= args.size())
    {
      return OperationResult::Failure(
          "Unexpected end of arguments while parsing fast DAC channels");
    }
    int numFastDacChannels = static_cast<int>(args[currentIndex++]);
    if (args.size() < currentIndex + numFastDacChannels * 3)
    {
      return OperationResult::Failure(
          "Not enough arguments for fast DAC channels");
    }

    int *fastDacChannels = new int[numFastDacChannels];
    float *fastDacV0s = new float[numFastDacChannels];
    float *fastDacVfs = new float[numFastDacChannels];

    for (int i = 0; i < numFastDacChannels; ++i)
    {
      fastDacChannels[i] = static_cast<int>(args[currentIndex++]);
      fastDacV0s[i] = args[currentIndex++];
      fastDacVfs[i] = args[currentIndex++];
    }

    // Parse Slow DAC Channels
    if (currentIndex >= args.size())
    {
      // Clean up allocated memory before returning
      delete[] fastDacChannels;
      delete[] fastDacV0s;
      delete[] fastDacVfs;
      return OperationResult::Failure(
          "Unexpected end of arguments while parsing slow DAC channels");
    }
    int numSlowDacChannels = static_cast<int>(args[currentIndex++]);
    if (args.size() < currentIndex + numSlowDacChannels * 3)
    {
      // Clean up allocated memory before returning
      delete[] fastDacChannels;
      delete[] fastDacV0s;
      delete[] fastDacVfs;
      return OperationResult::Failure(
          "Not enough arguments for slow DAC channels");
    }

    int *slowDacChannels = new int[numSlowDacChannels];
    float *slowDacV0s = new float[numSlowDacChannels];
    float *slowDacVfs = new float[numSlowDacChannels];

    for (int i = 0; i < numSlowDacChannels; ++i)
    {
      slowDacChannels[i] = static_cast<int>(args[currentIndex++]);
      slowDacV0s[i] = args[currentIndex++];
      slowDacVfs[i] = args[currentIndex++];
    }

    // Parse ADC Channels
    if (args.size() < currentIndex + numAdcChannels)
    {
      // Clean up allocated memory before returning
      delete[] fastDacChannels;
      delete[] fastDacV0s;
      delete[] fastDacVfs;
      delete[] slowDacChannels;
      delete[] slowDacV0s;
      delete[] slowDacVfs;
      return OperationResult::Failure("Not enough arguments for ADC channels");
    }

    int *adcChannels = new int[numAdcChannels];
    for (int i = 0; i < numAdcChannels; ++i)
    {
      adcChannels[i] = static_cast<int>(args[currentIndex++]);
    }

    // Validate total number of DAC channels
    if (numFastDacChannels + numSlowDacChannels != numDacChannels)
    {
      // Clean up allocated memory before returning
      delete[] fastDacChannels;
      delete[] fastDacV0s;
      delete[] fastDacVfs;
      delete[] slowDacChannels;
      delete[] slowDacV0s;
      delete[] slowDacVfs;
      delete[] adcChannels;
      return OperationResult::Failure(
          "Sum of fast and slow DAC channels does not match numDacChannels");
    }

    // Allocate memory for slow DAC voltage setpoints
    float **slowVoltSetpoints = new float *[numSlowDacChannels];
    for (int i = 0; i < numSlowDacChannels; ++i)
    {
      slowVoltSetpoints[i] = new float[numStepsSlow];
      for (int j = 0; j < numStepsSlow; ++j)
      {
        slowVoltSetpoints[i][j] =
            slowDacV0s[i] +
            (slowDacVfs[i] - slowDacV0s[i]) * j / (numStepsSlow - 1);
      }
    }

    // Start continuous ADC conversions
    for (int i = 0; i < numAdcChannels; i++)
    {
      ADCController::startContinuousConversion(adcChannels[i]);
    }

    // Iterate over slow steps with optional retrace
    for (int slowStep = 0; slowStep < numStepsSlow && !getStopFlag();
         ++slowStep)
    {
      // Set slow DAC channels to the current slow step voltages
      DACChannel::commsController.beginTransaction();
      for (int i = 0; i < numSlowDacChannels; ++i)
      {
        DACController::setVoltageNoTransaction(slowDacChannels[i],
                                               slowVoltSetpoints[i][slowStep]);
      }
      DACController::toggleLdac();
      DACChannel::commsController.endTransaction();

      // Determine ramp direction based on retrace flag
      bool isReverse = false;
      if (retrace)
      {
        isReverse = (slowStep % 2 != 0); // Reverse on odd slow steps
      }

      // Prepare ramp voltages
      float *currentV0s = fastDacV0s;
      float *currentVfs = fastDacVfs;
      if (isReverse)
      {
        // Swap V0 and Vf for reverse ramp
        currentV0s = new float[numFastDacChannels];
        currentVfs = new float[numFastDacChannels];
        for (int i = 0; i < numFastDacChannels; ++i)
        {
          currentV0s[i] = fastDacVfs[i];
          currentVfs[i] = fastDacV0s[i];
        }
      }

      // Call the base ramp function for fast channels
      OperationResult rampResult = dacLedBufferRampBaseNoConversionSetup(
          numFastDacChannels, numAdcChannels, numStepsFast, numAdcAverages, dac_interval_us,
          dac_settling_time_us, fastDacChannels, currentV0s, currentVfs,
          adcChannels);

      // If reverse ramp was performed, clean up the temporary arrays
      if (isReverse)
      {
        delete[] currentV0s;
        delete[] currentVfs;
      }

      if (!rampResult.isSuccess())
      {
        // Clean up allocated memory before returning
        for (int i = 0; i < numSlowDacChannels; ++i)
        {
          delete[] slowVoltSetpoints[i];
        }
        delete[] slowVoltSetpoints;
        delete[] fastDacChannels;
        delete[] fastDacV0s;
        delete[] fastDacVfs;
        delete[] slowDacChannels;
        delete[] slowDacV0s;
        delete[] slowDacVfs;
        delete[] adcChannels;
        return rampResult; // Return the failure reason
      }
    }

    // Set ADC channels to idle mode
    for (int i = 0; i < numAdcChannels; i++)
    {
      ADCController::idleMode(adcChannels[i]);
    }

    // Clean up allocated memory
    for (int i = 0; i < numSlowDacChannels; i++)
    {
      delete[] slowVoltSetpoints[i];
    }
    delete[] slowVoltSetpoints;
    delete[] fastDacChannels;
    delete[] fastDacV0s;
    delete[] fastDacVfs;
    delete[] slowDacChannels;
    delete[] slowDacV0s;
    delete[] slowDacVfs;
    delete[] adcChannels;

    if (getStopFlag())
    {
      setStopFlag(false);
      return OperationResult::Failure("2D RAMPING_STOPPED");
    }

    return OperationResult::Success();
  }

  static OperationResult dacLedBufferRampBaseNoConversionSetup(
      int numDacChannels, int numAdcChannels, int numSteps, int numAdcAverages,
      uint32_t dac_interval_us, uint32_t dac_settling_time_us, int *dacChannels,
      float *dacV0s, float *dacVfs, int *adcChannels)
  {
    if (dac_settling_time_us < 1 || dac_interval_us < 1 ||
        dac_settling_time_us >= dac_interval_us)
    {
      return OperationResult::Failure("Invalid interval or settling time");
    }
    if (numAdcAverages < 1)
    {
      return OperationResult::Failure("Invalid number of ADC averages");
    }
    if (numSteps < 1)
    {
      return OperationResult::Failure("Invalid number of steps");
    }
    if (numDacChannels < 1 || numAdcChannels < 1)
    {
      return OperationResult::Failure("Invalid number of channels");
    }
    for (int i = 0; i < numAdcChannels; i++)
    {
      if (dac_settling_time_us <
          ADCController::getConversionTimeFloat(adcChannels[i]))
      {
        return OperationResult::Failure(
            "DAC settling time too short for ADC conversion time");
      }
    }
    // uint32_t adc_comms_period_us = (1.0/SPI_SPEED)*1e6*8*4; // 8 bits per
    // byte, 4 bytes per ADC conversion if
    // (numAdcChannels*numAdcAverages*adc_comms_period_us > dac_interval_us -
    // dac_settling_time_us) {
    //   return OperationResult::Failure("Buffer Ramp limited by ADC SPI
    //   comms");
    // }

    int steps = 0;
    int x = 0;

    float **voltSetpoints = new float *[numDacChannels];

    for (int i = 0; i < numDacChannels; i++)
    {
      voltSetpoints[i] = new float[numSteps];
      for (int j = 0; j < numSteps; j++)
      {
        voltSetpoints[i][j] =
            dacV0s[i] + (dacVfs[i] - dacV0s[i]) * j / (numSteps - 1);
      }
    }

    // Set up timers with the same period but phase shifted
    TimingUtil::setupTimersDacLed(dac_interval_us, dac_settling_time_us);

    while (x < numSteps && !getStopFlag())
    {
      if (TimingUtil::adcFlag)
      {
        ADCBoard::commsController.beginTransaction();
        if (steps <= 1)
        {
          for (int i = 0; i < numAdcChannels; i++)
          {
            for (int j = 0; j < numAdcAverages; j++)
            {
              ADCController::getVoltageDataNoTransaction(adcChannels[i]);
            }
          }
        }
        else
        {
          VoltagePacket *packets = new VoltagePacket[numAdcChannels];
          for (int i = 0; i < numAdcChannels; i++)
          {
            float total = 0.0;
            for (int j = 0; j < numAdcAverages; j++)
            {
              total +=
                  ADCController::getVoltageDataNoTransaction(adcChannels[i]);
            }
            float v = total / numAdcAverages;
            packets[i] = {static_cast<uint8_t>(adcChannels[i]),
                          static_cast<uint32_t>(x), v};
          }
          m4SendVoltage(packets, numAdcChannels);
          delete[] packets;
          x++;
        }
        ADCBoard::commsController.endTransaction();
        TimingUtil::adcFlag = false;
      }
      if (TimingUtil::dacFlag && steps < numSteps + 1)
      {
        DACChannel::commsController.beginTransaction();
        if (steps == 0)
        {
          for (int i = 0; i < numDacChannels; i++)
          {
            DACController::setVoltageNoTransaction(dacChannels[i],
                                                   voltSetpoints[i][0]);
          }
        }
        else
        {
          for (int i = 0; i < numDacChannels; i++)
          {
            DACController::setVoltageNoTransaction(dacChannels[i],
                                                   voltSetpoints[i][steps - 1]);
          }
        }
        DACController::toggleLdac();
        DACChannel::commsController.endTransaction();
        steps++;
        TimingUtil::dacFlag = false;
      }
    }

    TimingUtil::disableDacInterrupt();
    TimingUtil::disableAdcInterrupt();

    for (int i = 0; i < numDacChannels; i++)
    {
      delete[] voltSetpoints[i];
    }
    delete[] voltSetpoints;

    if (getStopFlag())
    {
      setStopFlag(false);
      return OperationResult::Failure("RAMPING_STOPPED");
    }

    return OperationResult::Success();
  }
};