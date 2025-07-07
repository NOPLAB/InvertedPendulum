#include <cstdint>

// Mock timer handles
struct MockTIM_HandleTypeDef {
  uint32_t channel_values[5];  // TIM_CHANNEL_1 to TIM_CHANNEL_4
};

MockTIM_HandleTypeDef mock_htim1 = {{0}};
MockTIM_HandleTypeDef mock_htim2 = {{0}};
MockTIM_HandleTypeDef mock_htim3 = {{0}};

// Mock HAL functions - only compiled in test mode
#ifdef ENABLE_TESTING
extern "C" {
// These functions are only used in testing and don't conflict with HAL
void mock_hal_tim_set_compare(void* htim, uint32_t channel, uint32_t compare) {
  (void)htim;
  (void)channel;
  (void)compare;
}

void mock_hal_tim_pwm_start(void* htim, uint32_t channel) {
  (void)htim;
  (void)channel;
}
}
#endif