#ifndef TEST_FRAMEWORK_HPP_
#define TEST_FRAMEWORK_HPP_

#include <cstdio>
#include <cmath>

#ifdef PC_BUILD
#include <iostream>
#define printf(...) std::printf(__VA_ARGS__)
#endif

#define ASSERT_EQ(expected, actual) \
  do { \
    if ((expected) != (actual)) { \
      printf("FAIL: %s:%d - Expected: %f, Actual: %f\n", __FILE__, __LINE__, (float)(expected), (float)(actual)); \
      return false; \
    } \
  } while(0)

#define ASSERT_NEAR(expected, actual, tolerance) \
  do { \
    if (std::abs((expected) - (actual)) > (tolerance)) { \
      printf("FAIL: %s:%d - Expected: %f, Actual: %f, Tolerance: %f\n", __FILE__, __LINE__, (float)(expected), (float)(actual), (float)(tolerance)); \
      return false; \
    } \
  } while(0)

#define ASSERT_TRUE(condition) \
  do { \
    if (!(condition)) { \
      printf("FAIL: %s:%d - Condition failed\n", __FILE__, __LINE__); \
      return false; \
    } \
  } while(0)

#define TEST_FUNCTION(name) \
  bool test_##name()

#define RUN_TEST(name) \
  do { \
    printf("Running test: %s\n", #name); \
    if (test_##name()) { \
      printf("PASS: %s\n", #name); \
    } else { \
      printf("FAIL: %s\n", #name); \
      TestRunner::failed_tests++; \
    } \
    TestRunner::total_tests++; \
  } while(0)

class TestRunner {
public:
  static int total_tests;
  static int failed_tests;
  
  static void run_all_tests();
  static void print_summary();
};

#endif /* TEST_FRAMEWORK_HPP_ */