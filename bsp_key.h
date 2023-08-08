#ifndef __BSP_KEY_H__
#define __BSP_KEY_H__

#include "stm32f1xx.h"

// WKUP: PA0	KEY0: PC1	KEY1: PC13
#define WKUP_Pin                    GPIO_PIN_0
#define WKUP_GPIO_Port              GPIOA
#define WKUP_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define KEY0_Pin                    GPIO_PIN_1
#define KEY0_GPIO_Port              GPIOC
#define KEY0_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()
#define KEY1_Pin                    GPIO_PIN_13
#define KEY1_GPIO_Port              GPIOC
#define KEY1_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()

void KEY_GPIO_Init(void);

#endif
