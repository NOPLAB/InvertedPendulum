#include "interrupt.hpp"
#include "app.hpp"
#include "stm32f3xx_hal_adc.h"

inline void HAL_ADC_IRQHandler(ADC_HandleTypeDef *hadc) {
  if (hadc == nullptr) {
    return; // Null check for safety
  }

  App::getInstance().state->interruptHandler->handleAdcInterrupts();
}
