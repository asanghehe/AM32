/*
 * IO.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "IO.h"

#include "common.h"
#include "dshot.h"
#include "functions.h"
#include "serial_telemetry.h"
#include "targets.h"

char ic_timer_prescaler = (CPU_FREQUENCY_MHZ / 5);
uint32_t dma_buffer[64] = { 0 };
char out_put = 0;
uint8_t buffer_padding = 0;

/** 可以用，但用直接指令版 **/
void receiveDshotDma(){
  out_put = 0;
	
	LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM3);	// de-init timer 2
	LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM3);
	
	IC_TIMER_REGISTER->CCMR1 = 0x41;

	//禁止 TIM3_CCR1 寄存器的预装载功能	DisableARRPreload
	//01：CC1 通道被配置为输入，IC1 映射在 TI1 上；
	//DisableFast

	IC_TIMER_REGISTER->CCER = 0xa;	//=LL_TIM_IC_POLARITY_BOTHEDGE            (TIM_CCER_CC1P | TIM_CCER_CC1NP)
	IC_TIMER_REGISTER->PSC = ic_timer_prescaler;
	IC_TIMER_REGISTER->ARR = 0xFFFF;
	IC_TIMER_REGISTER->EGR |= TIM_EGR_UG;	//重新初始化计数器，并产生一个更新事件。注意：预分频器的计数器也被清 0(但是预分频系数不变)。
	//LL_TIM_GenerateEvent_UPDATE(IC_TIMER_REGISTER);
	//out_put = 0;
	
	IC_TIMER_REGISTER->CNT = 0;
	
	DMA1_Channel1->CMAR = (uint32_t)&dma_buffer;
	DMA1_Channel1->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
	DMA1_Channel1->CNDTR = buffersize;
	DMA1_Channel1->CCR = 0x98b;
	
	
	IC_TIMER_REGISTER->DIER |= TIM_DIER_CC1DE | TIM_DIER_TDE;
	IC_TIMER_REGISTER->CCER |= IC_TIMER_CHANNEL;
	IC_TIMER_REGISTER->CR1 |= TIM_CR1_CEN;
}

/**
void receiveDshotDma()
{
  out_put = 0;
	
	LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM3);
	LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM3);
	
	IC_TIMER_REGISTER->CCMR1 = 0x41;
	IC_TIMER_REGISTER->CCER = 0xa;
	
	IC_TIMER_REGISTER->PSC = ic_timer_prescaler;
	IC_TIMER_REGISTER->ARR = 0xFFFF;
	LL_TIM_GenerateEvent_UPDATE(IC_TIMER_REGISTER);
	//out_put = 0;
	
  IC_TIMER_REGISTER->CNT = 0;
	
  LL_DMA_ConfigTransfer(DMA1, INPUT_DMA_CHANNEL, LL_DMA_DIRECTION_PERIPH_TO_MEMORY | \
                        LL_DMA_MODE_NORMAL                  | \
                        LL_DMA_PERIPH_NOINCREMENT | \
                        LL_DMA_MEMORY_INCREMENT  | \
                        LL_DMA_PDATAALIGN_HALFWORD | \
                        LL_DMA_MDATAALIGN_WORD | \
                        LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMemoryAddress(DMA1, INPUT_DMA_CHANNEL, (uint32_t)&dma_buffer);
  LL_DMA_SetPeriphAddress(DMA1, INPUT_DMA_CHANNEL, (uint32_t)&IC_TIMER_REGISTER->CCR1);

	LL_DMA_SetDataLength(DMA1, INPUT_DMA_CHANNEL, buffersize);
	LL_DMA_EnableIT_TC(DMA1, INPUT_DMA_CHANNEL);
	//LL_DMA_EnableIT_HT(DMA1, INPUT_DMA_CHANNEL); 不再使用这个作为PWM处理
	LL_DMA_EnableIT_TE(DMA1, INPUT_DMA_CHANNEL);
	LL_DMA_EnableChannel(DMA1, INPUT_DMA_CHANNEL);

	//Enable capture/compare 1 DMA request (CC1DE).
	//等于   IC_TIMER_REGISTER->DIER |= TIM_DIER_CC1DE;
	LL_TIM_EnableDMAReq_CC1(TIM3);
	
	LL_TIM_CC_EnableChannel(IC_TIMER_REGISTER, IC_TIMER_CHANNEL);
	LL_TIM_EnableCounter(IC_TIMER_REGISTER);
}
**/

void sendDshotDma()
{
    out_put = 1;
#ifdef USE_TIMER_3_CHANNEL_1
    //          // de-init timer 2
    RCC->APBRSTR1 |= LL_APB1_GRP1_PERIPH_TIM3;
    RCC->APBRSTR1 &= ~LL_APB1_GRP1_PERIPH_TIM3;
#endif
#ifdef USE_TIMER_15_CHANNEL_1
    RCC->APB2RSTR |= LL_APB2_GRP1_PERIPH_TIM15;
    RCC->APB2RSTR &= ~LL_APB2_GRP1_PERIPH_TIM15;
#endif
    IC_TIMER_REGISTER->CCMR1 = 0x60;
    IC_TIMER_REGISTER->CCER = 0x3;
    IC_TIMER_REGISTER->PSC = output_timer_prescaler;
    IC_TIMER_REGISTER->ARR = 61;

    IC_TIMER_REGISTER->EGR |= TIM_EGR_UG;
#ifdef USE_TIMER_3_CHANNEL_1
    DMA1_Channel1->CMAR = (uint32_t)&gcr;
    DMA1_Channel1->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
    DMA1_Channel1->CNDTR = 23 + buffer_padding;
    DMA1_Channel1->CCR = 0x99b;
#endif
#ifdef USE_TIMER_15_CHANNEL_1
    //		  LL_DMA_ConfigAddresses(DMA1, INPUT_DMA_CHANNEL,
    //(uint32_t)&gcr, (uint32_t)&IC_TIMER_REGISTER->CCR1,
    // LL_DMA_GetDataTransferDirection(DMA1,
    // INPUT_DMA_CHANNEL));
    DMA1_Channel5->CMAR = (uint32_t)&gcr;
    DMA1_Channel5->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
    DMA1_Channel5->CNDTR = 23 + buffer_padding;
    DMA1_Channel5->CCR = 0x99b;
#endif
    IC_TIMER_REGISTER->DIER |= TIM_DIER_CC1DE;
    IC_TIMER_REGISTER->CCER |= IC_TIMER_CHANNEL;
    IC_TIMER_REGISTER->BDTR |= TIM_BDTR_MOE;
    IC_TIMER_REGISTER->CR1 |= TIM_CR1_CEN;
}

uint8_t getInputPinState() { return (INPUT_PIN_PORT->IDR & INPUT_PIN); }

void setInputPolarityRising()
{
    LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER, IC_TIMER_CHANNEL,
        LL_TIM_IC_POLARITY_RISING);
}

void setInputPullDown()
{
    LL_GPIO_SetPinPull(INPUT_PIN_PORT, INPUT_PIN, LL_GPIO_PULL_DOWN);
}

void setInputPullUp()
{
    LL_GPIO_SetPinPull(INPUT_PIN_PORT, INPUT_PIN, LL_GPIO_PULL_UP);
}

void enableHalfTransferInt() { LL_DMA_EnableIT_HT(DMA1, INPUT_DMA_CHANNEL); }
void setInputPullNone()
{
    LL_GPIO_SetPinPull(INPUT_PIN_PORT, INPUT_PIN, LL_GPIO_PULL_NO);
}
