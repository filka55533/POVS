#ifndef SECOND_THIRD_C
#define SECOND_THIRD_C

#include "headers.h"

volatile int isPushing = 0;
volatile int isPulling = 0;

void Dispatch(void)
{
    if (isPushing)
    {
        HAL_Delay(DELAY_TIME);
        HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
        isPushing--;
    }

    if (isPulling)
    {
        HAL_Delay(DELAY_TIME);
        HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
        isPulling--;
    }
}

void InterruptHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(BTN_PIN))
    {
        if (HAL_GPIO_ReadPin(BTN_GPIO_PORT, BTN_PIN) == 0)
        {
            isPushing++;
        }
        else
        {
            isPulling++;
        }
        __HAL_GPIO_EXTI_CLEAR_IT(BTN_PIN);
    }
}

#endif