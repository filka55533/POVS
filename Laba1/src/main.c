#include "headers.h"
#include "../lib/SecondThirdTask/src/SecondThird.h"

void LED_Init();
void BTN_Init();

int main(void) 
{
    HAL_Init();
    LED_Init();
    BTN_Init();

    while (1)
    {
        DISPATCH_PROCEDURE();
    }   
}

void LED_Init() 
{
    LED_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
}

void BTN_Init() 
{
    BTN_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_initStruct;
    GPIO_initStruct.Pin = BTN_PIN;
    GPIO_initStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_initStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BTN_GPIO_PORT, &GPIO_initStruct);

    HAL_NVIC_SetPriority(BTN_INTERRUPT_LINE, 0, 0);
    HAL_NVIC_EnableIRQ(BTN_INTERRUPT_LINE);
}

void SysTick_Handler(void) 
{
    HAL_IncTick();
}

void BTN_INTERRUPT_HANDLER(void)
{
    INTERRUPT_HANDLER();
}