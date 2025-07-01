#ifndef MUX__HPP_
#define MUX__HPP_

#include "adc.h"
#include "interrupt.hpp"

#define ADC1_CHANNELS 3
#define MUX_CHANNELS 8

struct Adc1CorrectedValues {
  float p_1;
  float p_2;
  float mux_value[MUX_CHANNELS];
};

class Adc1 : public IAdcInterruptHandler {
private:
  uint8_t selectedChannel = 0;
  uint16_t rawAdcValues[ADC1_CHANNELS] = {0};
  uint16_t muxAdcValues[MUX_CHANNELS] = {0};
  Adc1CorrectedValues correctedValues;

public:
  Adc1() { HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); }

public: // IAdcHandler
  ADC_HandleTypeDef *adcHandlerType() const override { return &hadc1; }

  void handleAdcInterrupt() override {
    this->muxAdcValues[selectedChannel] = this->rawAdcValues[0];
    this->switchMuxChannel();
  }

public:
  void scanAdcValues() {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)this->rawAdcValues, ADC1_CHANNELS);
  }

  void getCorrectedValues(Adc1CorrectedValues *values) {
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

#define ADC2_CHANNELS 2

struct Adc2CorrectedValues {
  float currentR;
  float currentL;
};

class Adc2 : public IAdcInterruptHandler {
private:
  uint16_t rawAdcValues[ADC2_CHANNELS] = {0};
  Adc2CorrectedValues correctedValues;

public:
  Adc2() {}

public:
  ADC_HandleTypeDef *adcHandlerType() const override { return &hadc2; }

  void handleAdcInterrupt() override {
    this->correctedValues.currentR =
        static_cast<float>(this->rawAdcValues[0]) / 4095.0f;
    this->correctedValues.currentL =
        static_cast<float>(this->rawAdcValues[1]) / 4095.0f;
  }

public:
  void scanAdcValues() {
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)this->rawAdcValues, ADC2_CHANNELS);
  }

  void getCorrectedValues(Adc2CorrectedValues *values) {
    *values = this->correctedValues;
  };
};

#endif /* MUX__HPP_ */