#ifndef APP__wrapper_HPP_
#define APP__wrapper_HPP_

#ifdef __cplusplus
extern "C" {
#endif

void AppRun(void);

void App_DMA1_Channel1_IRQHandler();
void App_TIM6_DAC1_IRQHandler();

#ifdef __cplusplus
}
#endif

#endif /* APP__wrapper_HPP_ */
