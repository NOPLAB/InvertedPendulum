#ifndef TEST_SIMPLE_PID_HPP_
#define TEST_SIMPLE_PID_HPP_

#include "test_framework.hpp"

// RED PHASE: Write failing test first
// Simple PID class with intentional bug to demonstrate TDD
class SimplePID {
public:
  SimplePID(float kp) : kp_(kp) {}
  
  float update(float error) {
    // GREEN PHASE: Fixed implementation to make tests pass
    return kp_ * error; // Correct proportional control
  }
  
private:
  float kp_;
};

// RED PHASE: This test should fail initially
TEST_FUNCTION(simple_pid_proportional_response) {
  SimplePID pid(2.0f);
  
  float output = pid.update(1.0f);  // error = 1.0
  ASSERT_EQ(2.0f, output);  // kp * error = 2.0 * 1.0 = 2.0
  
  return true;
}

// RED PHASE: This test should also fail initially
TEST_FUNCTION(simple_pid_zero_error) {
  SimplePID pid(1.5f);
  
  float output = pid.update(0.0f);  // no error
  ASSERT_EQ(0.0f, output);  // should be zero
  
  return true;
}

// RED PHASE: This test should also fail initially
TEST_FUNCTION(simple_pid_negative_error) {
  SimplePID pid(3.0f);
  
  float output = pid.update(-2.0f);  // negative error
  ASSERT_EQ(-6.0f, output);  // kp * error = 3.0 * (-2.0) = -6.0
  
  return true;
}

// REFACTOR PHASE: Add input validation
TEST_FUNCTION(simple_pid_input_validation) {
  SimplePID pid(1.0f);
  
  // Test with very large values
  float output = pid.update(1000.0f);
  ASSERT_EQ(1000.0f, output);
  
  // Test with very small values
  output = pid.update(0.001f);
  ASSERT_NEAR(0.001f, output, 0.0001f);
  
  return true;
}

void run_simple_pid_tests() {
  printf("\n=== Simple PID Tests (TDD Complete Cycle) ===\n");
  
  RUN_TEST(simple_pid_proportional_response);
  RUN_TEST(simple_pid_zero_error);
  RUN_TEST(simple_pid_negative_error);
  RUN_TEST(simple_pid_input_validation);
}

#endif /* TEST_SIMPLE_PID_HPP_ */