#include "bsp_usart.h"

UART_HandleTypeDef huart1;

#ifdef __GNUC__
/*
 * With GCC, small printf (option LD Linker -> Libraries -> Small printf
 * set to 'Yes') calls __io_putchar()
 * */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(FILE *f)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif

void USART1_GPIO_MspInit(void)
{
    GPIO_InitTypeDef USART_GPIO_Init;

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    USART_GPIO_Init.Pin = USART1_RX_Pin;
    USART_GPIO_Init.Mode = GPIO_MODE_AF_PP;
    USART_GPIO_Init.Pull = GPIO_NOPULL;
    USART_GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(USART1_RX_GPIO_Port, &USART_GPIO_Init);

    USART_GPIO_Init.Pin = USART1_TX_Pin;
    HAL_GPIO_Init(USART1_TX_GPIO_Port, &USART_GPIO_Init);
}

void DEBUG_USART_Config(void)
{
    // USART1 GPIO Init
    USART1_GPIO_MspInit();

    huart1.Instance = USART1;                                   // 串口号
    huart1.Init.BaudRate   = 115200;                                // 波特率
    huart1.Init.WordLength = UART_WORDLENGTH_8B;                   // 有效数据长度
    huart1.Init.StopBits   = UART_STOPBITS_1;                      // 停止位
    huart1.Init.Parity     = UART_PARITY_NONE;                     // 奇偶校验
    huart1.Init.Mode       = UART_MODE_TX_RX;                      // 发送&接收模式
    huart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;                  // 硬件流控制
    huart1.Init.OverSampling    = UART_OVERSAMPLING_16;            // 过采样

    HAL_UART_Init(&huart1);
}

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
	return ch;
}

GETCHAR_PROTOTYPE
{
	uint8_t ch = 0;
	HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
	return ch;
}

