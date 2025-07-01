#include "app_wrapper.hpp"
#include "app.hpp"

void AppRun(void) { App::getInstance().run(); }

void App_DMA1_Channel1_IRQHandler() {
  App::getInstance().getInterruptHandler()->handleAdcDMAInterrupts();
}

void App_TIM6_DAC1_IRQHandler() {
  App::getInstance().getInterruptHandler()->handleTimerInterrupts();
}