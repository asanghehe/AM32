/**
  ******************************************************************************
  * @file    py32md3xx_it.c
  * @author  MCU Application Team
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by Puya under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "py32md3xx_it.h"

#include "ADC.h"
#include "targets.h"
#include "IO.h"
//#include "WS2812.h"


extern void transfercomplete();
extern void PeriodElapsedCallback();
extern void interruptRoutine();
extern void tenKhzRoutine();
extern void processDshot();

extern char send_telemetry;
uint16_t interrupt_time = 0;
extern char servoPwm;
extern char dshot_telemetry;
extern char armed;
extern char out_put;
extern uint8_t compute_dshot_flag;
extern uint32_t commutation_interval;

int update_interupt = 0;

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/* PY32MD3xx Peripheral Interrupt Handlers                                     */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file.                                          */
/******************************************************************************/

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void){
		
	//此方法作废，使用dma数据时长判断，不启用这个了
	/**
    if (LL_DMA_IsActiveFlag_HT1(DMA1)) {
			
        if (servoPwm) {
            LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER, IC_TIMER_CHANNEL,
                LL_TIM_IC_POLARITY_FALLING);
            LL_DMA_ClearFlag_HT1(DMA1);
					
        }
    }
	**/
    if (LL_DMA_IsActiveFlag_TC1(DMA1) == 1) {
        LL_DMA_ClearFlag_GI1(DMA1);
			
        LL_DMA_DisableChannel(DMA1, IC_TIMER_CHANNEL);

        transfercomplete();
				
				//软件触发调用处理dma信号
				EXTI->SWIER |= LL_EXTI_LINE_15;

    } else if (LL_DMA_IsActiveFlag_TE1(DMA1) == 1) {
        LL_DMA_ClearFlag_GI1(DMA1);
    }
}

void DMA1_Channel2_3_IRQHandler(void)
{
		/* USER CODE END DMA1_Channel2_3_IRQn 0 */
    if (LL_DMA_IsActiveFlag_TC2(DMA1) == 1) {
        /* Clear flag DMA global interrupt */
        /* (global interrupt flag: half transfer and transfer complete flags) */
        LL_DMA_ClearFlag_GI2(DMA1);
			
        ADC_DMA_Callback();
        /* Call interruption treatment function */
        //   AdcDmaTransferComplete_Callback();
    }

    /* Check whether DMA transfer error caused the DMA interruption */
    if (LL_DMA_IsActiveFlag_TE2(DMA1) == 1) {
        /* Clear flag DMA transfer error */
        LL_DMA_ClearFlag_TE2(DMA1);

        /* Call interruption treatment function */
    }
}

// 使用了DMA，用不到这里
/**
void TIM3_IRQHandler(){
	  
    if (LL_TIM_IsActiveFlag_CC1(TIM3) == 1) {
        LL_TIM_ClearFlag_CC1(TIM3);
			
    }
		
	  if (LL_TIM_IsActiveFlag_CC4(TIM3) == 1) {
        LL_TIM_ClearFlag_CC4(TIM3);
    }
		
    if (LL_TIM_IsActiveFlag_UPDATE(TIM3) == 1) {
        LL_TIM_ClearFlag_UPDATE(TIM3);
        update_interupt++;
    }
}
**/

//TIM14 用作计时器，时间等待用了，不需要中断
//void TIM14_IRQHandler(){	
//}

void TIM16_IRQHandler(){
	
		if(LL_TIM_IsActiveFlag_UPDATE(TIM16) == 1) {
			LL_TIM_ClearFlag_UPDATE(TIM16);
			
			tenKhzRoutine();
		}
		
}

//换相计数器 COM_TIMER 20Khz
void TIM17_IRQHandler(){
		if(LL_TIM_IsActiveFlag_UPDATE(TIM17) == 1){
			
			PeriodElapsedCallback();
			
	    LL_TIM_ClearFlag_UPDATE(TIM17);
	  }
}

//比较器中断 换相
void ADC_COMP_IRQHandler(void){
	
		if(LL_EXTI_IsActiveFlag(EXTI_LINE) != RESET)
	  {

				LL_EXTI_ClearFlag(EXTI_LINE);
			
				interruptRoutine();
	  }
}

void EXTI4_15_IRQHandler(){
		
    LL_EXTI_ClearFlag(LL_EXTI_LINE_15);
    processDshot();
}
