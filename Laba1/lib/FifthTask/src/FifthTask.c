#ifndef FIFTH_TASK_C
#define FIFTH_TASK_C

#include "headers.h"

volatile int total = 0;

volatile int countLight = 0;

void Dispatch(void)
{
    for (int i = 0; i < countLight; i++)
    {
        total++;

        for (int j = 0; j < total * 2; j++)
        {
            HAL_Delay(DELAY_TIME);
            HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
        }

        countLight--;
    }

}

void InterruptHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(BTN_PIN))
    {
        if (HAL_GPIO_ReadPin(BTN_GPIO_PORT, BTN_PIN) == 0)
        {
            countLight++;
        }

        __HAL_GPIO_EXTI_CLEAR_IT(BTN_PIN);
    }
}

#endif