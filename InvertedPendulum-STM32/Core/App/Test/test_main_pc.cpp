#include "test_framework.hpp"
#include "test_pid.hpp"  // Real PID controller tests
#include "test_simple_pid.hpp"
// #include "test_motor.hpp"
// #include "test_interrupt.hpp"

#include <iostream>

void TestRunner::run_all_tests() {
  total_tests = 0;
  failed_tests = 0;

  std::cout << "=== Starting TDD Test Suite (PC Build) ===" << std::endl;

  // Run TDD test suites
  run_simple_pid_tests();
  run_pid_tests();  // Real PID controller from STM32 project
  // run_motor_tests();
  // run_interrupt_tests();

  // Print final summary
  print_summary();
}

// PC main function
int main() {
  std::cout << "InvertedPendulum TDD Tests - PC Build" << std::endl;
  std::cout << "=====================================" << std::endl;

  TestRunner::run_all_tests();

  // Return exit code based on test results
  return TestRunner::failed_tests == 0 ? 0 : 1;
}