#include "bsp_led.h"

void LED_GPIO_Init(void)
{
    GPIO_InitTypeDef LED_GPIO_Init;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin : PtPin */
    LED_GPIO_Init.Pin = LED0_Pin;
    LED_GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
    LED_GPIO_Init.Pull = GPIO_NOPULL;
    LED_GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LED0_GPIO_Port, &LED_GPIO_Init);

    /*Configure GPIO pin : PtPin */
    LED_GPIO_Init.Pin = LED1_Pin;
    LED_GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
    LED_GPIO_Init.Pull = GPIO_NOPULL;
    LED_GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LED1_GPIO_Port, &LED_GPIO_Init);
}
