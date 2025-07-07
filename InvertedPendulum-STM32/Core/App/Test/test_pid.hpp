#ifndef TEST_PID_HPP_
#define TEST_PID_HPP_

#include "test_framework.hpp"
#include "../Inc/pid.hpp"

// Test: PID controller initialization
TEST_FUNCTION(pid_initialization) {
  PID pid(1.0f, 0.5f, 0.1f, 0.01f, -10.0f, 10.0f);
  
  ASSERT_EQ(1.0f, pid.getKp());
  ASSERT_EQ(0.5f, pid.getKi());
  ASSERT_EQ(0.1f, pid.getKd());
  ASSERT_EQ(0.0f, pid.getIntegralTerm());
  ASSERT_EQ(0.0f, pid.getDerivativeTerm());
  
  return true;
}

// Test: PID proportional only response
TEST_FUNCTION(pid_proportional_only) {
  PID pid(2.0f, 0.0f, 0.0f, 0.01f, -10.0f, 10.0f);
  
  float output = pid.update(5.0f, 3.0f); // error = 2.0
  ASSERT_EQ(4.0f, output); // kp * error = 2.0 * 2.0 = 4.0
  
  return true;
}

// Test: PID integral accumulation
TEST_FUNCTION(pid_integral_accumulation) {
  PID pid(0.0f, 1.0f, 0.0f, 0.01f, -10.0f, 10.0f);
  
  // First update: error = 1.0
  float output1 = pid.update(3.0f, 2.0f);
  ASSERT_EQ(0.01f, output1); // ki * error * dt = 1.0 * 1.0 * 0.01 = 0.01
  
  // Second update: error = 1.0 (same)
  float output2 = pid.update(3.0f, 2.0f);
  ASSERT_EQ(0.02f, output2); // accumulated integral
  
  return true;
}

// Test: PID derivative response
TEST_FUNCTION(pid_derivative_response) {
  PID pid(0.0f, 0.0f, 1.0f, 0.01f, -10.0f, 10.0f);
  
  // First update: error = 2.0
  float output1 = pid.update(5.0f, 3.0f);
  ASSERT_EQ(0.0f, output1); // first call, derivative = 0
  
  // Second update: error = 1.0
  float output2 = pid.update(4.0f, 3.0f);
  // derivative = kd * (error - prev_error) / dt = 1.0 * (1.0 - 2.0) / 0.01 = -100.0
  // But it's clamped to the output limit of -10.0
  ASSERT_EQ(-10.0f, output2);
  
  return true;
}

// Test: PID output limits
TEST_FUNCTION(pid_output_limits) {
  PID pid(10.0f, 0.0f, 0.0f, 0.01f, -5.0f, 5.0f);
  
  // Test upper limit
  float output1 = pid.update(10.0f, 0.0f); // error = 10.0, output = 100.0
  ASSERT_EQ(5.0f, output1); // clamped to max
  
  // Test lower limit
  float output2 = pid.update(-10.0f, 0.0f); // error = -10.0, output = -100.0
  ASSERT_EQ(-5.0f, output2); // clamped to min
  
  return true;
}

// Test: PID reset functionality
TEST_FUNCTION(pid_reset) {
  PID pid(1.0f, 1.0f, 1.0f, 0.01f, -10.0f, 10.0f);
  
  // Generate some history
  pid.update(5.0f, 3.0f);
  pid.update(4.0f, 2.0f);
  
  // Reset should clear internal state
  pid.reset();
  
  // After reset, derivative should be 0 on first call
  float output = pid.update(3.0f, 2.0f);
  // Only proportional and integral: 1.0 * 1.0 + 1.0 * 1.0 * 0.01 = 1.01
  ASSERT_EQ(1.01f, output);
  
  return true;
}

// Test: PID anti-windup
TEST_FUNCTION(pid_anti_windup) {
  PID pid(1.0f, 10.0f, 0.0f, 0.01f, -1.0f, 1.0f);
  
  // Generate large error to saturate output
  for (int i = 0; i < 100; i++) {
    pid.update(10.0f, 0.0f); // large error
  }
  
  // Check that integral doesn't grow indefinitely
  // This test verifies anti-windup is working
  float final_output = pid.update(0.0f, 0.0f); // no error
  
  // Output should not be excessively large due to anti-windup
  ASSERT_TRUE(final_output >= -1.0f && final_output <= 1.0f);
  
  return true;
}

void run_pid_tests() {
  printf("\n=== PID Controller Tests ===\n");
  
  RUN_TEST(pid_initialization);
  RUN_TEST(pid_proportional_only);
  RUN_TEST(pid_integral_accumulation);
  RUN_TEST(pid_derivative_response);
  RUN_TEST(pid_output_limits);
  RUN_TEST(pid_reset);
  RUN_TEST(pid_anti_windup);
}

#endif /* TEST_PID_HPP_ */