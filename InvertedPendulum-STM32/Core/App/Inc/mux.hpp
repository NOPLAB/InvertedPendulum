#ifndef MUX__HPP_
#define MUX__HPP_

#include "adc.h"
#include "interrupt.hpp"

#define MUX_CHANNELS 16

struct MuxCorrectedValues {
  float value[MUX_CHANNELS];
};

class Mux : public IAdcInterruptHandler {
private:
  uint32_t adcValues[MUX_CHANNELS] = {0};
  MuxCorrectedValues correctedValues;

public:
  Mux();
  ~Mux();

public: // IAdcHandler
  ADC_HandleTypeDef *adcHandlerType() const override { return &hadc1; }

  void handleAdcInterrupt() override { this->switchMuxChannel(); }

public:
  void getCorrectedValues(MuxCorrectedValues *values);

private:
  uint8_t selectedChannel = 0;

  void switchMuxChannel() {
    this->selectedChannel++;
    if (this->selectedChannel >= MUX_CHANNELS) {
      this->selectedChannel = 0;
    }

    HAL_GPIO_WritePin(Mux_A_GPIO_Port, Mux_A_Pin,
                      (this->selectedChannel & 0x01) ? GPIO_PIN_SET
                                                     : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Mux_B_GPIO_Port, Mux_B_Pin,
                      (this->selectedChannel & 0x02) ? GPIO_PIN_SET
                                                     : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Mux_C_GPIO_Port, Mux_C_Pin,
                      (this->selectedChannel & 0x04) ? GPIO_PIN_SET
                                                     : GPIO_PIN_RESET);
  }
};

#endif /* MUX__HPP_ */