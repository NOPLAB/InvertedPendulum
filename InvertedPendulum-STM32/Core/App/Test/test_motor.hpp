#ifndef TEST_MOTOR_HPP_
#define TEST_MOTOR_HPP_

#include "test_framework.hpp"

// Mock HAL functions for testing
extern "C" {
  void __HAL_TIM_SET_COMPARE(void* htim, uint32_t channel, uint32_t compare);
  void HAL_TIM_PWM_Start(void* htim, uint32_t channel);
}

// Mock timer handles for testing
struct MockTIM_HandleTypeDef {
  uint32_t channel_values[5]; // TIM_CHANNEL_1 to TIM_CHANNEL_4
};

extern MockTIM_HandleTypeDef mock_htim1;
extern MockTIM_HandleTypeDef mock_htim2;
extern MockTIM_HandleTypeDef mock_htim3;

// Testable motor class that doesn't depend on HAL
class TestableMotors {
public:
  TestableMotors() {
    // Initialize mock timers
    for (int i = 0; i < 5; i++) {
      mock_htim1.channel_values[i] = 0;
      mock_htim2.channel_values[i] = 0;
      mock_htim3.channel_values[i] = 0;
    }
  }

  void setSpeedLeft(const float v) {
    float s = clamp(v, -1.0f, 1.0f);
    
    if (s > 0) {
      setTimerCompare(&mock_htim3, 4, 0);
      setTimerCompare(&mock_htim2, 2, s * 3199.0f);
    } else if (s < 0) {
      setTimerCompare(&mock_htim3, 4, -s * 3199.0f);
      setTimerCompare(&mock_htim2, 2, 0);
    } else {
      setTimerCompare(&mock_htim3, 4, 0);
      setTimerCompare(&mock_htim2, 2, 0);
    }
  }

  void setSpeedRight(const float v) {
    float s = clamp(v, -1.0f, 1.0f);
    
    if (s > 0) {
      setTimerCompare(&mock_htim1, 4, 0);
      setTimerCompare(&mock_htim1, 3, s * 3199.0f);
    } else if (s < 0) {
      setTimerCompare(&mock_htim1, 4, -s * 3199.0f);
      setTimerCompare(&mock_htim1, 3, 0);
    } else {
      setTimerCompare(&mock_htim1, 4, 0);
      setTimerCompare(&mock_htim1, 3, 0);
    }
  }

  // Test helper functions
  uint32_t getTimerCompare(MockTIM_HandleTypeDef* htim, uint32_t channel) {
    return htim->channel_values[channel];
  }

private:
  float clamp(float value, float min_val, float max_val) {
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
  }
  
  void setTimerCompare(MockTIM_HandleTypeDef* htim, uint32_t channel, uint32_t compare) {
    htim->channel_values[channel] = compare;
  }
};

// Test: Motor speed clamping
TEST_FUNCTION(motor_speed_clamping) {
  TestableMotors motors;
  
  // Test upper limit
  motors.setSpeedLeft(1.5f);
  ASSERT_EQ(3199, motors.getTimerCompare(&mock_htim2, 2));
  ASSERT_EQ(0, motors.getTimerCompare(&mock_htim3, 4));
  
  // Test lower limit
  motors.setSpeedLeft(-1.5f);
  ASSERT_EQ(0, motors.getTimerCompare(&mock_htim2, 2));
  ASSERT_EQ(3199, motors.getTimerCompare(&mock_htim3, 4));
  
  return true;
}

// Test: Motor forward direction
TEST_FUNCTION(motor_forward_direction) {
  TestableMotors motors;
  
  motors.setSpeedLeft(0.5f);
  ASSERT_EQ(1599, motors.getTimerCompare(&mock_htim2, 2)); // 0.5 * 3199 = 1599.5
  ASSERT_EQ(0, motors.getTimerCompare(&mock_htim3, 4));
  
  motors.setSpeedRight(0.5f);
  ASSERT_EQ(1599, motors.getTimerCompare(&mock_htim1, 3));
  ASSERT_EQ(0, motors.getTimerCompare(&mock_htim1, 4));
  
  return true;
}

// Test: Motor reverse direction
TEST_FUNCTION(motor_reverse_direction) {
  TestableMotors motors;
  
  motors.setSpeedLeft(-0.5f);
  ASSERT_EQ(0, motors.getTimerCompare(&mock_htim2, 2));
  ASSERT_EQ(1599, motors.getTimerCompare(&mock_htim3, 4));
  
  motors.setSpeedRight(-0.5f);
  ASSERT_EQ(0, motors.getTimerCompare(&mock_htim1, 3));
  ASSERT_EQ(1599, motors.getTimerCompare(&mock_htim1, 4));
  
  return true;
}

// Test: Motor stop
TEST_FUNCTION(motor_stop) {
  TestableMotors motors;
  
  // First set some speed
  motors.setSpeedLeft(0.5f);
  motors.setSpeedRight(0.5f);
  
  // Then stop
  motors.setSpeedLeft(0.0f);
  motors.setSpeedRight(0.0f);
  
  ASSERT_EQ(0, motors.getTimerCompare(&mock_htim2, 2));
  ASSERT_EQ(0, motors.getTimerCompare(&mock_htim3, 4));
  ASSERT_EQ(0, motors.getTimerCompare(&mock_htim1, 3));
  ASSERT_EQ(0, motors.getTimerCompare(&mock_htim1, 4));
  
  return true;
}

// Test: Motor PWM calculation
TEST_FUNCTION(motor_pwm_calculation) {
  TestableMotors motors;
  
  motors.setSpeedLeft(0.25f);
  ASSERT_EQ(799, motors.getTimerCompare(&mock_htim2, 2)); // 0.25 * 3199 = 799.75
  
  motors.setSpeedLeft(0.75f);
  ASSERT_EQ(2399, motors.getTimerCompare(&mock_htim2, 2)); // 0.75 * 3199 = 2399.25
  
  return true;
}

// Test: Motor independence
TEST_FUNCTION(motor_independence) {
  TestableMotors motors;
  
  // Set left motor forward, right motor reverse
  motors.setSpeedLeft(0.5f);
  motors.setSpeedRight(-0.3f);
  
  // Check left motor
  ASSERT_EQ(1599, motors.getTimerCompare(&mock_htim2, 2));
  ASSERT_EQ(0, motors.getTimerCompare(&mock_htim3, 4));
  
  // Check right motor
  ASSERT_EQ(0, motors.getTimerCompare(&mock_htim1, 3));
  ASSERT_EQ(959, motors.getTimerCompare(&mock_htim1, 4)); // 0.3 * 3199 = 959.7
  
  return true;
}

void run_motor_tests() {
  printf("\n=== Motor Control Tests ===\n");
  
  RUN_TEST(motor_speed_clamping);
  RUN_TEST(motor_forward_direction);
  RUN_TEST(motor_reverse_direction);
  RUN_TEST(motor_stop);
  RUN_TEST(motor_pwm_calculation);
  RUN_TEST(motor_independence);
}

#endif /* TEST_MOTOR_HPP_ */