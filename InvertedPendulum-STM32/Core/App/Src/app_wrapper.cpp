#include "app_wrapper.hpp"
#include "app.hpp"

void AppRun(void) { App::getInstance().run(); }

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  App::getInstance().getInterruptHandler()->handleAdcDMAInterrupts(hadc);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  App::getInstance().getInterruptHandler()->handleTimerInterrupts(htim);
}
