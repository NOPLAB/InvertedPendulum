#include "adc.hpp"

Adc::Adc() {
  // HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
}

Adc::~Adc() { HAL_ADC_Stop_DMA(&hadc1); }

void Adc::getCorrectedValues(AdcCorrectedValues *values) {
  this->correctedValues.p_1 =
      static_cast<float>(this->rawAdcValues[0]) / 4095.0f;
  this->correctedValues.p_2 =
      static_cast<float>(this->rawAdcValues[1]) / 4095.0f;

  for (int i = 0; i < MUX_CHANNELS; i++) {
    this->correctedValues.mux_value[i] =
        static_cast<float>(this->muxAdcValues[i]) / 4095.0f;
  }

  *values = this->correctedValues;
}
