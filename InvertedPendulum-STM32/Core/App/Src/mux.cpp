#include "mux.hpp"
#include "stm32f3xx_hal_adc.h"

Mux::Mux() {}

Mux::~Mux() { HAL_ADC_Stop_DMA(&hadc1); }

void Mux::initialize() {
  HAL_ADC_Start_DMA(&hadc1, this->adcValues, MUX_CHANNELS);
}

inline void Mux::handleAdcInterrupt() { this->switchMuxChannel(); }

inline void Mux::getCorrectedValues(MuxCorrectedValues *values) {
  // ADCの値をfloatに変換
  for (int i = 0; i < MUX_CHANNELS; i++) {
    this->correctedValues.value[i] =
        static_cast<float>(this->adcValues[i]) / 4095.0f; // 12-bit ADC
  }

  *values = this->correctedValues;
}

inline void Mux::switchMuxChannel() {
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