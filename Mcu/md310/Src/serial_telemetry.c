/*
 * serial_telemetry.c
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */

#include "serial_telemetry.h"
#include "common.h"
#include "kiss_telemetry.h"

void telem_UART_Init(void)
{
    LL_USART_InitTypeDef USART_InitStruct = { 0 };

    /* GPIOA configuration */
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
		
    /* Enable GPIOA clock */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    /* Enable USART1 clock */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
    
    /* Select pin 2 */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
    /* Select alternate function mode */
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    /* Select output speed */
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    /* Select push-pull output mode */
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    /* Select pull-up */
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    /* Select USART1 function */
    GPIO_InitStruct.Alternate = LL_GPIO_AF1_USART1;
    /* Initialize GPIOA */
    LL_GPIO_Init(GPIOA,&GPIO_InitStruct);
    
    /* Select pin 3 */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
    /* Select USART1 function */
    GPIO_InitStruct.Alternate = LL_GPIO_AF1_USART1;
    /* Initialize GPIOA */
    LL_GPIO_Init(GPIOA,&GPIO_InitStruct);

    /* USART1 DMA Init */

    /* USART1_TX Init */
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2,
        LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

    /* USART1 interrupt Init */
    NVIC_SetPriority(USART1_IRQn, 3);
    NVIC_EnableIRQ(USART1_IRQn);

    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART1, &USART_InitStruct);
    LL_USART_DisableIT_CTS(USART1);
    LL_USART_ConfigAsyncMode(USART1);
    LL_USART_Enable(USART1);

    // set dma address
    LL_DMA_ConfigAddresses(
        DMA1, LL_DMA_CHANNEL_2, (uint32_t)aTxBuffer,
        LL_USART_DMA_GetRegAddr(USART1),
        LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2));
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, sizeof(aTxBuffer));

    /* (5) Enable DMA transfer complete/error interrupts  */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);
}

void send_telem_DMA(uint8_t bytes)
{
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, bytes);
    LL_USART_EnableDMAReq_TX(USART1);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_RX);
}
