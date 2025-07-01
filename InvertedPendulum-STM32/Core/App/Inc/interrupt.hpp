#ifndef ADC__HPP_
#define ADC__HPP_

#include "stm32f3xx_hal.h"

class IAdcInterruptHandler {
public:
  virtual ADC_HandleTypeDef *adcHandlerType() const = 0;
  virtual void handleAdcInterrupt() = 0;
};

class InterruptHandler {
private:
  IAdcInterruptHandler **adcHandlers;
  int adcHandlersNum = 0;

public:
  InterruptHandler(IAdcInterruptHandler *handlers[], int handlerNum) {
    this->adcHandlers = handlers;
    this->adcHandlersNum = handlerNum;
  }

  void handleAdcInterrupts() {
    for (int i = 0; i < adcHandlersNum; i++) {
      ADC_HandleTypeDef *hadc = adcHandlers[i]->adcHandlerType();
      if (hadc != nullptr && hadc->Instance != nullptr) {
        HAL_ADC_IRQHandler(hadc);
        adcHandlers[i]->handleAdcInterrupt();
      }
    }
  }
};

#endif /* ADC__HPP_ */