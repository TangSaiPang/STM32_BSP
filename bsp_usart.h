/*
 * bsp_usart.h
 *
 *  Created on: May 28, 2023
 *      Author: Saipa
 */

#ifndef USART_BSP_USART_H_
#define USART_BSP_USART_H_

#include "stm32f1xx.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1;

// USART1 GPIO Definition
#define USART1_RX_Pin GPIO_PIN_10
#define USART1_RX_GPIO_Port GPIOA
#define USART1_TX_Pin GPIO_PIN_9
#define USART1_TX_GPIO_Port GPIOA

void DEBUG_USART_Config(void);


#endif /* USART_BSP_USART_H_ */
