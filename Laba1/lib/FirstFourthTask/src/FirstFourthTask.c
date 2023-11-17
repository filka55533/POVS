#ifndef FIRST_FOURTH_TASK_C
#define FIRST_FOURTH_TASK_C
#define TRIPPLE

#include "headers.h"

volatile int isLighting = 0;
volatile int clickCount = 0;

volatile uint32_t prevTime = 0;

inline uint32_t abs(uint32_t x)
{
    return x < 0 ? -x : x;
}

void Dispatch(void)
{
    if (isLighting)
    {
        for (int i = 0; i < 3 * 2; i++)
        {
            HAL_Delay(DELAY_TIME);
            HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
        }
        isLighting--;
    }
}

void InterruptHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(BTN_PIN))
    {
        if (HAL_GPIO_ReadPin(BTN_GPIO_PORT, BTN_PIN) == 0)
        {
            uint32_t currentTime = HAL_GetTick();
            if ( abs(currentTime - prevTime) < 250 ) 
            {
                clickCount++;
                #ifdef TRIPPLE
                if (clickCount >= 2)
                #else
                if (clickCount >= 1)
                #endif
                {
                    isLighting++;
                    clickCount = 0;
                }
            }
            else 
            {
                clickCount = 0;
            }

            prevTime = currentTime;
        }
        __HAL_GPIO_EXTI_CLEAR_IT(BTN_PIN);
    }
}

#endif
