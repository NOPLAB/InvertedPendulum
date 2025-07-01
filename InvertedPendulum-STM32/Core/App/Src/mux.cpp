#include "mux.hpp"
#include "stm32f3xx_hal_adc.h"

Mux::Mux() { HAL_ADC_Start_DMA(&hadc1, this->adcValues, MUX_CHANNELS); }

Mux::~Mux() { HAL_ADC_Stop_DMA(&hadc1); }

void Mux::getCorrectedValues(MuxCorrectedValues *values) {
  // ADCの値をfloatに変換
  for (int i = 0; i < MUX_CHANNELS; i++) {
    this->correctedValues.value[i] =
        static_cast<float>(this->adcValues[i]) / 4095.0f; // 12-bit ADC
  }

  *values = this->correctedValues;
}
