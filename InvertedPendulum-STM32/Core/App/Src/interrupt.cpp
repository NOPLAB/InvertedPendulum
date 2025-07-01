#include "interrupt.hpp"
#include "app.hpp"
#include "stm32f3xx_hal_adc.h"

inline void HAL_ADC_IRQHandler(ADC_HandleTypeDef *hadc) {
  App::getInstance().getInterruptHandler()->handleAdcInterrupts(hadc);
}

inline void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim) {
  App::getInstance().getInterruptHandler()->handleTimerInterrupts(htim);
}
