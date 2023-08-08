#ifndef __BSP_BUZZER_H__
#define __BSP_BUZZER_H__

#include "stm32f1xx.h"

// BUZZER PC5
#define BUZZER_Pin GPIO_PIN_5
#define BUZZER_GPIO_Port GPIOC
#define BUZZER_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()

// To Operate LED's GPIO Pin
#define BUZZER_ON         HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET)
#define BUZZER_OFF        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET)
#define BUZZER_TOGGLE     HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin)

void BUZZER_GPIO_Init(void);

#endif
