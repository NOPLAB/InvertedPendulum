#include "app_wrapper.hpp"
#include "app.hpp"
#include "main.h"

App *AppInstance = App::getInstanceRef();

void AppRun(void) { App::getInstance().run(); }

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (!App::getInstance().initialized)
    return;

  App::getInstance().getInterruptHandler()->handleAdcInterrupts(hadc);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (!App::getInstance().initialized)
    return;

  App::getInstance().getInterruptHandler()->handleTimerInterrupts(htim);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (!App::getInstance().initialized)
    return;

  if (GPIO_Pin == QEI_L_A_Pin || GPIO_Pin == QEI_L_B_Pin) {
    QEI_Encode(&App::getInstance().encoderLeft);
  }
  if (GPIO_Pin == QEI_R_A_Pin || GPIO_Pin == QEI_R_B_Pin) {
    QEI_Encode(&App::getInstance().encoderRight);
  }
}
