#ifndef MUX__HPP_
#define MUX__HPP_

#include "adc.h"
#include "device.hpp"
#include "interrupt.hpp"

#define MUX_CHANNELS 16

struct MuxCorrectedValues {
  float value[MUX_CHANNELS];
};

class Mux : public IDevice, public IAdcInterruptHandler {
private:
  uint32_t adcValues[MUX_CHANNELS] = {0};
  MuxCorrectedValues correctedValues;

public:
  Mux();
  ~Mux();

public: // IDevice
  void initialize() override;

public: // IAdcHandler
  ADC_HandleTypeDef *adcHandlerType() const override { return &hadc1; }

  void handleAdcInterrupt() override;

public:
  void getCorrectedValues(MuxCorrectedValues *values);

private:
  uint8_t selectedChannel = 0;

  inline void switchMuxChannel();
};

#endif /* MUX__HPP_ */