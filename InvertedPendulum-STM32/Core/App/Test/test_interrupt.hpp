#ifndef TEST_INTERRUPT_HPP_
#define TEST_INTERRUPT_HPP_

#include "test_framework.hpp"
#include "../Inc/interrupt.hpp"

// Mock ADC and Timer handles
struct MockADC_HandleTypeDef {
  int id;
};

struct MockTIM_HandleTypeDef {
  int id;
};

// Mock ADC and Timer interrupt handlers
class MockAdcHandler : public IAdcInterruptHandler {
public:
  MockADC_HandleTypeDef* adc_handle;
  bool interrupt_called;
  
  MockAdcHandler(MockADC_HandleTypeDef* handle) : adc_handle(handle), interrupt_called(false) {}
  
  ADC_HandleTypeDef* adcHandlerType() const override {
    return (ADC_HandleTypeDef*)adc_handle;
  }
  
  void handleAdcInterrupt() override {
    interrupt_called = true;
  }
};

class MockTimerHandler : public ITimerInterruptHandler {
public:
  MockTIM_HandleTypeDef* timer_handle;
  bool interrupt_called;
  
  MockTimerHandler(MockTIM_HandleTypeDef* handle) : timer_handle(handle), interrupt_called(false) {}
  
  TIM_HandleTypeDef* timerHandlerType() const override {
    return (TIM_HandleTypeDef*)timer_handle;
  }
  
  void handleTimerInterrupt() override {
    interrupt_called = true;
  }
};

// Test: Register and handle ADC interrupts
TEST_FUNCTION(interrupt_adc_registration) {
  MockADC_HandleTypeDef adc1 = {1};
  MockADC_HandleTypeDef adc2 = {2};
  
  MockAdcHandler handler1(&adc1);
  MockAdcHandler handler2(&adc2);
  
  IAdcInterruptHandler* handlers[] = {&handler1, &handler2};
  
  InterruptHandler interrupt_handler;
  interrupt_handler.registerAdc(handlers, 2);
  
  // Test ADC1 interrupt
  interrupt_handler.handleAdcInterrupts((ADC_HandleTypeDef*)&adc1);
  ASSERT_TRUE(handler1.interrupt_called);
  ASSERT_TRUE(!handler2.interrupt_called);
  
  // Reset and test ADC2 interrupt
  handler1.interrupt_called = false;
  handler2.interrupt_called = false;
  
  interrupt_handler.handleAdcInterrupts((ADC_HandleTypeDef*)&adc2);
  ASSERT_TRUE(!handler1.interrupt_called);
  ASSERT_TRUE(handler2.interrupt_called);
  
  return true;
}

// Test: Register and handle Timer interrupts
TEST_FUNCTION(interrupt_timer_registration) {
  MockTIM_HandleTypeDef tim1 = {1};
  MockTIM_HandleTypeDef tim2 = {2};
  
  MockTimerHandler handler1(&tim1);
  MockTimerHandler handler2(&tim2);
  
  ITimerInterruptHandler* handlers[] = {&handler1, &handler2};
  
  InterruptHandler interrupt_handler;
  interrupt_handler.registerTimer(handlers, 2);
  
  // Test TIM1 interrupt
  interrupt_handler.handleTimerInterrupts((TIM_HandleTypeDef*)&tim1);
  ASSERT_TRUE(handler1.interrupt_called);
  ASSERT_TRUE(!handler2.interrupt_called);
  
  // Reset and test TIM2 interrupt
  handler1.interrupt_called = false;
  handler2.interrupt_called = false;
  
  interrupt_handler.handleTimerInterrupts((TIM_HandleTypeDef*)&tim2);
  ASSERT_TRUE(!handler1.interrupt_called);
  ASSERT_TRUE(handler2.interrupt_called);
  
  return true;
}

// Test: Handle unknown interrupt
TEST_FUNCTION(interrupt_unknown_handler) {
  MockADC_HandleTypeDef adc1 = {1};
  MockADC_HandleTypeDef adc_unknown = {999};
  
  MockAdcHandler handler1(&adc1);
  IAdcInterruptHandler* handlers[] = {&handler1};
  
  InterruptHandler interrupt_handler;
  interrupt_handler.registerAdc(handlers, 1);
  
  // Test unknown ADC interrupt
  interrupt_handler.handleAdcInterrupts((ADC_HandleTypeDef*)&adc_unknown);
  ASSERT_TRUE(!handler1.interrupt_called);
  
  return true;
}

// Test: Multiple handlers for same interrupt
TEST_FUNCTION(interrupt_multiple_handlers) {
  MockADC_HandleTypeDef adc1 = {1};
  
  MockAdcHandler handler1(&adc1);
  MockAdcHandler handler2(&adc1); // Same ADC handle
  
  IAdcInterruptHandler* handlers[] = {&handler1, &handler2};
  
  InterruptHandler interrupt_handler;
  interrupt_handler.registerAdc(handlers, 2);
  
  // Both handlers should be called for the same ADC
  interrupt_handler.handleAdcInterrupts((ADC_HandleTypeDef*)&adc1);
  ASSERT_TRUE(handler1.interrupt_called);
  ASSERT_TRUE(handler2.interrupt_called);
  
  return true;
}

// Test: No handlers registered
TEST_FUNCTION(interrupt_no_handlers) {
  MockADC_HandleTypeDef adc1 = {1};
  
  InterruptHandler interrupt_handler;
  // Don't register any handlers
  
  // Should not crash when no handlers are registered
  interrupt_handler.handleAdcInterrupts((ADC_HandleTypeDef*)&adc1);
  
  return true;
}

// Test: Boundary conditions
TEST_FUNCTION(interrupt_boundary_conditions) {
  MockADC_HandleTypeDef adc1 = {1};
  MockAdcHandler handler1(&adc1);
  
  // Test with null handler array
  InterruptHandler interrupt_handler;
  interrupt_handler.registerAdc(nullptr, 0);
  interrupt_handler.handleAdcInterrupts((ADC_HandleTypeDef*)&adc1);
  
  // Test with zero handlers
  IAdcInterruptHandler* handlers[] = {&handler1};
  interrupt_handler.registerAdc(handlers, 0);
  interrupt_handler.handleAdcInterrupts((ADC_HandleTypeDef*)&adc1);
  
  ASSERT_TRUE(!handler1.interrupt_called);
  
  return true;
}

void run_interrupt_tests() {
  printf("\n=== Interrupt Handler Tests ===\n");
  
  RUN_TEST(interrupt_adc_registration);
  RUN_TEST(interrupt_timer_registration);
  RUN_TEST(interrupt_unknown_handler);
  RUN_TEST(interrupt_multiple_handlers);
  RUN_TEST(interrupt_no_handlers);
  RUN_TEST(interrupt_boundary_conditions);
}

#endif /* TEST_INTERRUPT_HPP_ */