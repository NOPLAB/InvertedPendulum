#include "test_framework.hpp"

int TestRunner::total_tests = 0;
int TestRunner::failed_tests = 0;

void TestRunner::print_summary() {
  printf("\n=== Test Summary ===\n");
  printf("Total tests: %d\n", total_tests);
  printf("Passed: %d\n", total_tests - failed_tests);
  printf("Failed: %d\n", failed_tests);
  
  if (failed_tests == 0) {
    printf("✅ All tests passed!\n");
  } else {
    printf("❌ Some tests failed!\n");
  }
  
#ifdef PC_BUILD
  printf("Build: PC Cross-compilation\n");
#else
  printf("Build: STM32 Target\n");
#endif
}