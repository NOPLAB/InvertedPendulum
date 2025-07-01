#ifndef MUX__HPP_
#define MUX__HPP_

#include "adc.h"
#include "interrupt.hpp"

#define ADC_CHANNELS 3
#define MUX_CHANNELS 8

struct AdcCorrectedValues {
  float p_1;
  float p_2;
  float mux_value[MUX_CHANNELS];
};

class Adc : public IAdcInterruptHandler {
private:
  uint8_t selectedChannel = 0;
  uint16_t rawAdcValues[ADC_CHANNELS] = {0};
  uint16_t muxAdcValues[MUX_CHANNELS] = {0};
  AdcCorrectedValues correctedValues;

public:
  Adc();
  ~Adc();

public: // IAdcHandler
  ADC_HandleTypeDef *adcHandlerType() const override { return &hadc1; }

  void handleAdcInterrupt() override {
    this->muxAdcValues[selectedChannel] = this->rawAdcValues[0];
    this->switchMuxChannel();
  }

public:
  void scanAdcValues() {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)this->rawAdcValues, ADC_CHANNELS);
  }

  void getCorrectedValues(AdcCorrectedValues *values);

private:
  void switchMuxChannel() {
    HAL_GPIO_WritePin(Mux_A_GPIO_Port, Mux_A_Pin,
                      (this->selectedChannel & 0x01) ? GPIO_PIN_SET
                                                     : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Mux_B_GPIO_Port, Mux_B_Pin,
                      (this->selectedChannel & 0x02) ? GPIO_PIN_SET
                                                     : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Mux_C_GPIO_Port, Mux_C_Pin,
                      (this->selectedChannel & 0x04) ? GPIO_PIN_SET
                                                     : GPIO_PIN_RESET);

    this->selectedChannel++;
    if (this->selectedChannel >= MUX_CHANNELS) {
      this->selectedChannel = 0;
    }
  }
};

#endif /* MUX__HPP_ */