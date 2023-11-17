#ifndef HEADERS_H
#define HEADERS_H

#include "stm32f1xx_hal.h"

#define LED_PIN                                GPIO_PIN_5
#define LED_GPIO_PORT                          GPIOA
#define LED_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOA_CLK_ENABLE()

#define BTN_PIN                                GPIO_PIN_13
#define BTN_GPIO_PORT                          GPIOC
#define BTN_GPIO_CLK_ENABLE()                    __HAL_RCC_GPIOC_CLK_ENABLE()
#define BTN_INTERRUPT_LINE                     EXTI15_10_IRQn
#define BTN_INTERRUPT_HANDLER              EXTI15_10_IRQHandler

#define DELAY_TIME                             100

#endif