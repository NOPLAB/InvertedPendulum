#include <cstdint>
#include <iostream>

// PC-specific mock implementations
// These don't need to do anything on PC, just prevent linking errors

extern "C" {
  // Mock HAL functions for PC build
  void mock_hal_tim_set_compare(void* htim, uint32_t channel, uint32_t compare) {
    // PC mock - could log if needed for debugging
    (void)htim;
    (void)channel;
    (void)compare;
  }
  
  void mock_hal_tim_pwm_start(void* htim, uint32_t channel) {
    // PC mock - could log if needed for debugging
    (void)htim;
    (void)channel;
  }
}