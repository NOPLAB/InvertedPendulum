#ifndef ADC__HPP_
#define ADC__HPP_

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_adc.h"

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

  void handleAdcInterrupts(ADC_HandleTypeDef *hadc = nullptr) {
    for (int i = 0; i < adcHandlersNum; i++) {
      ADC_HandleTypeDef *handler = adcHandlers[i]->adcHandlerType();
      if (handler == hadc) {
        adcHandlers[i]->handleAdcInterrupt();
      }
    }
  }
};

#endif /* ADC__HPP_ */