#pragma once

#include "stm32h7xx.h"

struct TimingUtil {
  inline static volatile bool adcFlag = false;
  inline static volatile bool dacFlag = false;

  inline static void resetTimers() {
    // Disable all interrupts
    __disable_irq();

    // Reset TIM1 and TIM8
    __HAL_RCC_TIM1_FORCE_RESET();
    __HAL_RCC_TIM8_FORCE_RESET();
    __HAL_RCC_TIM1_RELEASE_RESET();
    __HAL_RCC_TIM8_RELEASE_RESET();

    // Disable TIM1 and TIM8 clocks
    __HAL_RCC_TIM1_CLK_DISABLE();
    __HAL_RCC_TIM8_CLK_DISABLE();

    // Disable and clear all relevant interrupts
    NVIC_DisableIRQ(TIM1_UP_IRQn);
    NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
    NVIC_DisableIRQ(TIM8_CC_IRQn);
    NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
    NVIC_ClearPendingIRQ(TIM8_UP_TIM13_IRQn);
    NVIC_ClearPendingIRQ(TIM8_CC_IRQn);

    // Reset flags
    adcFlag = false;
    dacFlag = false;

    // Re-enable interrupts
    __enable_irq();
    delayMicroseconds(5);
  }

  inline static void setupTimerOnlyDac(uint32_t period_us) {
    resetTimers();

    // Enable TIM1 clock
    __HAL_RCC_TIM1_CLK_ENABLE();

    uint32_t timerClock = HAL_RCC_GetPCLK2Freq();

    // Configure TIM1
    TIM1->PSC = (2 * timerClock / 1000000) - 1;  // For 1us resolution
    TIM1->ARR = period_us - 1;
    TIM1->CR1 = TIM_CR1_ARPE;
    TIM1->DIER |= TIM_DIER_UIE;

    // Enable interrupts
    NVIC_SetPriority(TIM1_UP_IRQn, 2);
    NVIC_EnableIRQ(TIM1_UP_IRQn);

    // Start timer
    TIM1->CR1 |= TIM_CR1_CEN;
  }

  inline static void setupTimersTimeSeries(uint32_t dac_period_us,
                                           uint32_t adc_period_us) {
    resetTimers();

    // Enable TIM1 and TIM8 clocks
   __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    uint32_t timerClock = HAL_RCC_GetPCLK2Freq();

    // Configure TIM1
    TIM1->PSC = (2 * timerClock / 1000000) - 1;  // For 1us resolution
    TIM1->ARR = dac_period_us - 1;
    TIM1->CR1 = TIM_CR1_ARPE;
    TIM1->DIER |= TIM_DIER_UIE;

    // Configure TIM8
    TIM8->PSC = (2 * timerClock / 1000000) - 1;  // For 1us resolution
    TIM8->ARR = adc_period_us - 1;
    TIM8->CR1 = TIM_CR1_ARPE;
    TIM8->DIER |= TIM_DIER_UIE;

    // Enable interrupts
    NVIC_SetPriority(TIM1_UP_IRQn, 2);
    NVIC_EnableIRQ(TIM1_UP_IRQn);
    NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 3);
    NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

    // Start timers
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM8->CR1 |= TIM_CR1_CEN;
  }

  inline static void setupTimersDacLed(uint32_t period_us,
                                       uint32_t phase_shift_us) {
    resetTimers();

    // Enable TIM1 and TIM8 clocks
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    uint32_t timerClock = HAL_RCC_GetPCLK2Freq();

    // Configure TIM1 (Master timer - DAC trigger)
    TIM1->PSC = (2 * timerClock / 1000000) - 1;  // For 1us resolution
    TIM1->ARR = period_us - 1;
    TIM1->CR1 = TIM_CR1_ARPE;
    TIM1->CR2 |= TIM_CR2_MMS_1;  // Master Mode Selection: Update event as
                                 // trigger output (TRGO)
    TIM1->DIER |= TIM_DIER_UIE;

    // Configure TIM8 (Slave timer - ADC trigger)
    TIM8->PSC = (2 * timerClock / 1000000) - 1;  // For 1us resolution
    TIM8->ARR = period_us - 1;
    TIM8->CR1 = TIM_CR1_ARPE;
    TIM8->SMCR |=
        TIM_SMCR_TS_0 | TIM_SMCR_TS_2;  // Trigger selection: ITR0 (TIM1)
    TIM8->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1;  // Slave mode: Trigger mode
    TIM8->DIER |= TIM_DIER_UIE;

    // Set up phase shift
    if (phase_shift_us > 0 && phase_shift_us < period_us) {
      TIM8->CCR1 = phase_shift_us + 2;
      TIM8->CCMR1 |= TIM_CCMR1_OC1M_1 |
                     TIM_CCMR1_OC1M_2;  // Output Compare mode: PWM mode 1
      TIM8->CCER |= TIM_CCER_CC1E;      // Enable Capture/Compare 1 output
      TIM8->DIER |= TIM_DIER_CC1IE;     // Enable Capture/Compare 1 interrupt
    }

    // Enable interrupts
    NVIC_SetPriority(TIM1_UP_IRQn, 2);
    NVIC_EnableIRQ(TIM1_UP_IRQn);
    NVIC_SetPriority(TIM8_CC_IRQn, 3);
    NVIC_EnableIRQ(TIM8_CC_IRQn);

    // Start timers
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM8->CR1 |= TIM_CR1_CEN;
  }

  inline static void disableDacInterrupt() {
    TIM1->DIER &= ~TIM_DIER_UIE;
    NVIC_DisableIRQ(TIM1_UP_IRQn);
  }

  inline static void disableAdcInterrupt() {
    TIM8->DIER &= ~TIM_DIER_UIE;
    NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
    NVIC_DisableIRQ(TIM8_CC_IRQn);
  }
};

extern "C" void TIM1_UP_IRQHandler(void) {
  if (TIM1->SR & TIM_SR_UIF) {
    TIM1->SR &= ~TIM_SR_UIF;
    TimingUtil::dacFlag = true;
  }
}

extern "C" void TIM8_UP_TIM13_IRQHandler(void) {
  if (TIM8->SR & TIM_SR_UIF) {
    TIM8->SR &= ~TIM_SR_UIF;
    ADCController::toggleSync();
  }
}

extern "C" void TIM8_CC_IRQHandler(void) {
  if (TIM8->SR & TIM_SR_CC1IF) {
    TIM8->SR &= ~TIM_SR_CC1IF;
    ADCController::toggleSync();
  }
}