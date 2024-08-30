#include <Arduino.h>

// #include "RPC.h"

#include "Utils/shared_memory.h"

#define return_if_not_ok(x)    \
  do {                         \
    int ret = x;               \
    if (ret != HAL_OK) return; \
  } while (0);

void enableM4() {
  HAL_MPU_Disable();

  // Disable caching for the shared memory region.
  MPU_Region_InitTypeDef MPU_InitStruct;
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = D3_SRAM_BASE;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER15;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  // Enable the MPU
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
  // If CM4 is already booted, disable auto-boot and reset.
  FLASH_OBProgramInitTypeDef OBInit;

  OBInit.Banks = FLASH_BANK_1;
  HAL_FLASHEx_OBGetConfig(&OBInit);
  if (OBInit.USERConfig & FLASH_OPTSR_BCM4) {
    OBInit.OptionType = OPTIONBYTE_USER;
    OBInit.USERType = OB_USER_BCM4;
    OBInit.USERConfig = 0;
    return_if_not_ok(HAL_FLASH_OB_Unlock());
    return_if_not_ok(HAL_FLASH_Unlock());
    return_if_not_ok(HAL_FLASHEx_OBProgram(&OBInit));
    return_if_not_ok(HAL_FLASH_OB_Launch());
    return_if_not_ok(HAL_FLASH_OB_Lock());
    return_if_not_ok(HAL_FLASH_Lock());
    printf("CM4 autoboot disabled\n");
    NVIC_SystemReset();
    return;
  }

  // Classic boot, just set the address and we are ready to go
  LL_SYSCFG_SetCM4BootAddress0(CM4_BINARY_START >> 16);
  LL_RCC_ForceCM4Boot();
}

void setup() {
  enableM4();

  if (!initSharedMemory()) {
    while (1) {
      Serial.println("Failed to initialize shared memory");
      delay(1000);
    }
  }
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    m7SendData(command.c_str());
  }
  if (m7CheckForNewData()) {
    char response[MESSAGE_SIZE];
    m7GetData(response);
    Serial.println(response);
  }
  if (m7CheckForNewFloats()) {
    float floats[MAX_FLOATS];
    size_t count = m7GetFloats(floats);  // Retrieve the count of floats
    for (size_t i = 0; i < count; i++) {
      Serial.println(floats[i], 8);
    }
  }
}