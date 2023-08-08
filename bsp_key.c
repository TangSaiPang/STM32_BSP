#include "bsp_key.h"

void KEY_GPIO_Init(void)
{
    GPIO_InitTypeDef KEY_GPIO_Init;

    WKUP_GPIO_CLK_ENABLE();
    KEY0_GPIO_CLK_ENABLE();
    KEY1_GPIO_CLK_ENABLE();

    KEY_GPIO_Init.Pin = WKUP_Pin;
    KEY_GPIO_Init.Mode = GPIO_MODE_IT_RISING;
    KEY_GPIO_Init.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(WKUP_GPIO_Port, &KEY_GPIO_Init);

    KEY_GPIO_Init.Pin = KEY0_Pin;
    KEY_GPIO_Init.Mode = GPIO_MODE_IT_FALLING;
    KEY_GPIO_Init.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY0_GPIO_Port, &KEY_GPIO_Init);

    KEY_GPIO_Init.Pin = KEY0_Pin;
    HAL_GPIO_Init(KEY1_GPIO_Port, &KEY_GPIO_Init);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
