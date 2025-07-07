#include "test_framework.hpp"
#include "test_simple_pid.hpp"
// #include "test_pid.hpp"
// #include "test_motor.hpp"
// #include "test_interrupt.hpp"

void TestRunner::run_all_tests() {
  total_tests = 0;
  failed_tests = 0;
  
  printf("=== Starting TDD Test Suite ===\n");
  
  // Run TDD test suites (RED-GREEN-REFACTOR)
  run_simple_pid_tests();
  // run_pid_tests();
  // run_motor_tests();
  // run_interrupt_tests();
  
  // Print final summary
  print_summary();
}

// Main test function - can be called from embedded system
extern "C" void run_tdd_tests() {
  TestRunner::run_all_tests();
}