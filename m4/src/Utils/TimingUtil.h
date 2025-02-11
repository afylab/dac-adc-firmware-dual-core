#pragma once

#include "stm32h7xx.h"

struct TimingUtil {
  inline static volatile bool adcFlag = false;
  inline static volatile bool dacFlag = false;

  inline static void resetTimers() {
    // Disable all interrupts
    __disable_irq();

    // Reset TIM5
    __HAL_RCC_TIM5_FORCE_RESET();
    __HAL_RCC_TIM5_RELEASE_RESET();

    // Disable TIM5 clock
    __HAL_RCC_TIM5_CLK_DISABLE();

    // Disable and clear TIM5 interrupt
    NVIC_DisableIRQ(TIM5_IRQn);
    NVIC_ClearPendingIRQ(TIM5_IRQn);

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

    TIM1->EGR |= 0x01;
    TIM1->SR &= ~TIM_SR_UIF;
    TIM1->EGR |= 0x02;
    TIM1->CCR1 &= ~TIM_SR_CC1IF;

    // Configure TIM8
    TIM8->PSC = (2 * timerClock / 1000000) - 1;  // For 1us resolution
    TIM8->ARR = adc_period_us - 1;
    TIM8->CR1 = TIM_CR1_ARPE;
    TIM8->DIER |= TIM_DIER_UIE;

    TIM8->EGR |= 0x01;
    TIM8->SR &= ~TIM_SR_UIF;
    TIM8->EGR |= 0x02;
    TIM8->CCR1 &= ~TIM_SR_CC1IF;

    // Enable interrupts
    NVIC_SetPriority(TIM1_UP_IRQn, 2);
    NVIC_EnableIRQ(TIM1_UP_IRQn);
    NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 3);
    NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

    // Start timers
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM8->CR1 |= TIM_CR1_CEN;
  }


  inline static void setupTimersDacLed(uint32_t period_us, uint32_t phase_shift_us) {
    // Reset timers and clear prior configuration
    resetTimers();
    
    // Enable clock for TIM5
    __HAL_RCC_TIM5_CLK_ENABLE();

    // Configure TIM5 basic settings
    uint32_t timerClock = HAL_RCC_GetPCLK2Freq();
    TIM5->PSC = (2 * timerClock / 1000000) - 1;  // 1 µs resolution
    TIM5->ARR = period_us - 1;
    TIM5->CR1 = TIM_CR1_ARPE;
    TIM5->CNT = 0;  // Start at 0

    // Configure CC1 for DAC triggering (at start of period)
    TIM5->CCR1 = 0;  // Trigger DAC at beginning
    TIM5->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM5->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;  // PWM mode 1
    TIM5->CCER |= TIM_CCER_CC1E;  // Enable CC1 output

    // Configure CC2 for ADC triggering (with phase shift)
    TIM5->CCR2 = (phase_shift_us > 0 && phase_shift_us < period_us) ? 
                  phase_shift_us : 0;
    TIM5->CCMR1 &= ~TIM_CCMR1_OC2M;
    TIM5->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;  // PWM mode 1
    TIM5->CCER |= TIM_CCER_CC2E;  // Enable CC2 output

    // Clear any pending flags
    TIM5->SR = 0;

    // Enable interrupts for both channels
    TIM5->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE;

    // Configure NVIC
    NVIC_SetPriority(TIM5_IRQn, 2);
    NVIC_EnableIRQ(TIM5_IRQn);

    // Start timer
    TIM5->CR1 |= TIM_CR1_CEN;
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

extern "C" void TIM5_IRQHandler(void) {
  if (TIM5->SR & TIM_SR_CC1IF) {
      TIM5->SR &= ~TIM_SR_CC1IF;  // Clear CC1 flag
      TimingUtil::dacFlag = true;
  }
  if (TIM5->SR & TIM_SR_CC2IF) {
      TIM5->SR &= ~TIM_SR_CC2IF;  // Clear CC2 flag
      TimingUtil::adcFlag = true;
  }
}