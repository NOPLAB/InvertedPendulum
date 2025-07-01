#ifndef ADC__HPP_
#define ADC__HPP_

#include "stm32f3xx_hal.h"

class IAdcInterruptHandler {
public:
  virtual ADC_HandleTypeDef *adcHandlerType() const = 0;
  virtual void handleAdcInterrupt() = 0;
};

class ITimerInterruptHandler {
public:
  virtual TIM_HandleTypeDef *timerHandlerType() const = 0;
  virtual void handleTimerInterrupt() = 0;
};

class InterruptHandler {
private:
  IAdcInterruptHandler **adcHandlers = nullptr;
  int adcHandlersNum = 0;

  ITimerInterruptHandler **timerHandlers = nullptr;
  int timerHandlersNum = 0;

public:
  InterruptHandler() {}

  void registerAdc(IAdcInterruptHandler *handlers[], int handlerNum) {
    this->adcHandlers = handlers;
    this->adcHandlersNum = handlerNum;
  }

  void handleAdcDMAInterrupts(ADC_HandleTypeDef *hadc) {
    for (int i = 0; i < adcHandlersNum; i++) {
      ADC_HandleTypeDef *handler = adcHandlers[i]->adcHandlerType();
      if (handler == hadc) {
        adcHandlers[i]->handleAdcInterrupt();
      }
    }
  }

  void registerTimer(ITimerInterruptHandler *handlers[], int handlerNum) {
    this->timerHandlers = handlers;
    this->timerHandlersNum = handlerNum;
  }

  void handleTimerInterrupts(TIM_HandleTypeDef *htim) {
    for (int i = 0; i < timerHandlersNum; i++) {
      TIM_HandleTypeDef *handler = timerHandlers[i]->timerHandlerType();
      if (handler == htim) {
        timerHandlers[i]->handleTimerInterrupt();
      }
    }
  }
};

#endif /* ADC__HPP_ */