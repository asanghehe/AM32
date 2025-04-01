/*
 * peripherals.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

// PERIPHERAL SETUP

#include "peripherals.h"

#include "ADC.h"
#include "serial_telemetry.h"
#include "targets.h"

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
// static void MX_ADC_Init(void);
void MX_COMP_Init(void);
void MX_TIM1_Init(void);
void MX_TIM14_Init(void);
void TEN_KHZ_Timer_Init(void);
void COM_TIMER_TIM17_Init(void);
// static void MX_USART1_UART_Init(void);
void LED_GPIO_init(void);

void UN_TIM_Init();

void MX_LPTIM_Init(void);

uint8_t driverConfig[1] = {0x69 & (~(3 << 2))};

void initCorePeripherals(void)
{
    SystemClock_Config();
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	
		//NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    //NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));
		
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_TIM14_Init();
    TEN_KHZ_Timer_Init();
    COM_TIMER_TIM17_Init();
		//MX_LPTIM_Init();		//不使用硬件延时了，因为延时时没有使用中断
    MX_COMP_Init();
		
		UN_TIM_Init();
		
#ifdef USE_SERIAL_TELEMETRY
    telem_UART_Init();
#endif
}


//m0+ 支持 VTOR , 不需要复杂操作
void initAfterJump(void)
{
    __enable_irq();
}


LL_UTILS_ClkInitTypeDef UTILS_ClkInitStruct = {LL_RCC_SYSCLK_DIV_1, LL_RCC_APB1_DIV_1};

void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
	
  /* Enable and initialize HSI */
  LL_RCC_HSI_Enable();
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_24MHz);
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Configure system clock with HSI as clock source of the PLL and initialize it */
  LL_PLL_ConfigSystemClock_HSI(&UTILS_ClkInitStruct);
  
	
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
	
  /* Set systick to 1ms */
  LL_Init1msTick(48000000);
  
  /* Update the SystemCoreClock global variable(which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(48000000);
}

void MX_COMP_Init(void)
{
  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  LL_COMP_InitTypeDef COMP_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);
	
  /**COMP2 GPIO Configuration
  PB4 (NJTRST)   ------> COMP2_INP
  PB7   ------> COMP2_INM
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	//COMP2_INM
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  /* Enable clock for Comparator 1 */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_COMP2);

	
  /* USER CODE END COMP2_Init 1 */
  COMP_InitStruct.PowerMode = LL_COMP_POWERMODE_HIGHSPEED;
  COMP_InitStruct.InputPlus = LL_COMP_INPUT_PLUS_IO3;
  COMP_InitStruct.InputMinus = LL_COMP_INPUT_MINUS_IO3;
  COMP_InitStruct.InputHysteresis = LL_COMP_HYSTERESIS_DISABLE;
  //COMP_InitStruct.OutputPolarity = LL_COMP_OUTPUTPOL_NONINVERTED;
  COMP_InitStruct.OutputPolarity = LL_COMP_OUTPUTPOL_INVERTED;	//哎，比较器反相
	COMP_InitStruct.DigitalFilter = 0;
	
  /* Initialize Comparator 1 */
  LL_COMP_Init(MAIN_COMP, &COMP_InitStruct);
	
  /* Disable window mode */
  LL_COMP_SetCommonWindowMode(__LL_COMP_COMMON_INSTANCE(MAIN_COMP), LL_COMP_WINDOWMODE_DISABLE);

  /* Wait loop initialization and execution */
  /* Note: Variable divided by 2 to compensate partially CPU processing cycles */
  __IO uint32_t wait_loop_index = 0;
  wait_loop_index = (LL_COMP_DELAY_VOLTAGE_SCALER_STAB_US * (SystemCoreClock / (1000000 * 2)));
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }
	
  LL_EXTI_DisableEvent(LL_EXTI_LINE_18);
  LL_EXTI_DisableIT(LL_EXTI_LINE_18);
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */


  /* USER CODE BEGIN COMP2_Init 2 */
  NVIC_SetPriority(ADC_COMP_IRQn, 0);
  NVIC_EnableIRQ(ADC_COMP_IRQn);
  //__NVIC_EnableIRQ;
  /* USER CODE END COMP2_Init 2 */
}

void MX_IWDG_Init(void)
{
	  /* Enable LSI */
		LL_RCC_LSI_Enable();
		while (LL_RCC_LSI_IsReady() == 0U) {;}

    LL_IWDG_Enable(IWDG);
		
    LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_16);
    LL_IWDG_SetReloadCounter(IWDG, 4095);
    while (LL_IWDG_IsReady(IWDG) != 1) {
    }

    LL_IWDG_ReloadCounter(IWDG);
}

void MX_GPIO_Init(void)
{
    /* GPIO Ports Clock Enable */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

/**/
#ifdef USE_ALKAS_DEBUG_LED
    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif
}

void MX_TIM1_Init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = { 0 };
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = { 0 };
    LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = { 0 };

    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* Peripheral clock enable */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = TIM1_AUTORELOAD;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIM1, &TIM_InitStruct);
    LL_TIM_EnableARRPreload(TIM1);
		
		LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
		
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
		
#ifdef USE_SWAPPED_OUPUT
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
#else
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
#endif
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 0;
		
#ifdef USE_INVERTED_HIGH
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
#else
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
#endif
#ifdef USE_INVERTED_LOW
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_HIGH;
#else
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
#endif
		
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
		
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
		
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
		
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);
		
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM1);
		
    TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
    TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
    TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
    TIM_BDTRInitStruct.DeadTime = DEAD_TIME;
    TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
    TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
    TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
    LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
		
    /**TIM1 GPIO Configuration
    PB13   ------> TIM1_CH1N
    PB14   ------> TIM1_CH2N
    PB15   ------> TIM1_CH3N
    PA8   ------> TIM1_CH1
    PA9   ------> TIM1_CH2
    PA10   ------> TIM1_CH3
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_14;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void MX_TIM14_Init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

		LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM14);

		TIM_InitStruct.Prescaler = 23;
		TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
		TIM_InitStruct.Autoreload = 0xFFFF;
		TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
		TIM_InitStruct.RepetitionCounter = 0;
		LL_TIM_Init(TIM14, &TIM_InitStruct);
		
		LL_TIM_DisableARRPreload(TIM14);
		
    //LL_TIM_SetClockSource(TIM14, LL_TIM_CLOCKSOURCE_INTERNAL);
    //LL_TIM_SetTriggerOutput(TIM14, LL_TIM_TRGO_RESET);
    //LL_TIM_DisableMasterSlaveMode(TIM14);	//TIM14 没有 SMCR 寄存器
}

void TEN_KHZ_Timer_Init()
{

    LL_TIM_InitTypeDef TIM_InitStruct = { 0 };
    /* Peripheral clock enable */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM16);
		
    TIM_InitStruct.Prescaler = 47;		//CPU 频率非2次方分频，这里分成48M / (47+1) = 1Mhz = 1000000
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 1000000 / LOOP_FREQUENCY_HZ;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIM16, &TIM_InitStruct);
		
    LL_TIM_EnableARRPreload(TIM16);
		
    NVIC_SetPriority(TIM16_IRQn, 2);
    NVIC_EnableIRQ(TIM16_IRQn);
}

//换相计数器 COM_TIMER 20Khz
void COM_TIMER_TIM17_Init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = { 0 };

    /* Peripheral clock enable */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM17);

    /* TIM14 interrupt Init */
    NVIC_SetPriority(TIM17_IRQn, 0);
    NVIC_EnableIRQ(TIM17_IRQn);
		
    TIM_InitStruct.Prescaler = 23;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 0xFFFF;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM17, &TIM_InitStruct);
		
    LL_TIM_EnableARRPreload(TIM17);
}

/** 由于计数器不够用，使用LPTIM当作阻塞延时计数器，节省通用计数器 **/
void MX_LPTIM_Init(void){
	
	/* Enable LPTIM clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);
	
	//使用高速时钟
  LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_PCLK1);
	
  /* Configure LPTIM */
  /* LPTIM prescaler: divide by 64 */
  LL_LPTIM_SetPrescaler(LPTIM1,LL_LPTIM_PRESCALER_DIV32);
  
  /* Update ARR at the end of LPTIM counting period */
  LL_LPTIM_SetUpdateMode(LPTIM1,LL_LPTIM_UPDATE_MODE_ENDOFPERIOD);
  
  /* Configure auto-reload value: 62500 */
  /* 8000000/64/62500 = 2Hz */
  //LL_LPTIM_SetAutoReload(LPTIM1,62500);
}

void MX_DMA_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	
    NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	
    NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}

//输入捕获初始化
void UN_TIM_Init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = { 0 };

    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
		LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
		
    /**TIM16 GPIO Configuration
    PA6   ------> TIM3_CH1
    */
    GPIO_InitStruct.Pin = INPUT_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
		//GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		

#ifdef USE_TIMER_15_CHANNEL_1
    NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
    NVIC_EnableIRQ(IC_DMA_IRQ_NAME);
#endif
#ifdef USE_TIMER_3_CHANNEL_1
    NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
    NVIC_EnableIRQ(IC_DMA_IRQ_NAME);
#endif

    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 63;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(IC_TIMER_REGISTER, &TIM_InitStruct);
		
    LL_TIM_DisableARRPreload(IC_TIMER_REGISTER);
		
		LL_TIM_CC_DisablePreload(TIM3);
		LL_TIM_DisableARRPreload(TIM3);
		LL_TIM_DisableMasterSlaveMode(TIM3);	
	
		LL_TIM_SetTriggerOutput(IC_TIMER_REGISTER, LL_TIM_TRGO_RESET);
	
		LL_TIM_IC_InitTypeDef InputCaptureInit = {0};

		/* Configure capture channel */
		InputCaptureInit.ICActiveInput  = LL_TIM_ACTIVEINPUT_DIRECTTI;
		InputCaptureInit.ICPrescaler    = LL_TIM_ICPSC_DIV1;
		InputCaptureInit.ICPolarity     = LL_TIM_IC_POLARITY_BOTHEDGE;
		InputCaptureInit.ICFilter       = LL_TIM_IC_FILTER_FDIV1;
		LL_TIM_IC_Init(TIM3, LL_TIM_CHANNEL_CH1, &InputCaptureInit);
		

		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
		
		/* Map TIM3 update interrupt to channel 1 */
		/* 映射DMA触发源 */
		LL_SYSCFG_SetDMARemap_CH1(LL_SYSCFG_DMA_MAP_TIM3_CH1);
			
		/* ======================================== */
		/* Enable UPDATE DMA request  计数器溢出事件 */
		//LL_TIM_EnableDMAReq_UPDATE(TIM3);
		//设置DMA触发器
		LL_TIM_CC_SetDMAReqTrigger(TIM3, LL_TIM_CCDMAREQUEST_CC);
		
}


void reloadWatchDogCounter()
{
    LL_IWDG_ReloadCounter(IWDG);
}

inline void setPWMCompare1(uint16_t compareone) { TIM1->CCR1 = compareone; }
inline void setPWMCompare2(uint16_t comparetwo) { TIM1->CCR2 = comparetwo; }
inline void setPWMCompare3(uint16_t comparethree) { TIM1->CCR3 = comparethree; }


inline void generatePwmTimerEvent() { 
	LL_TIM_GenerateEvent_UPDATE(TIM1); 
}

inline void resetInputCaptureTimer()
{
    IC_TIMER_REGISTER->PSC = 0;
    IC_TIMER_REGISTER->CNT = 0;
}

void enableCorePeripherals()
{
		
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);

    //LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4); // timer used for timing adc read
    //TIM1->CCR4 = 100; // set in 10khz loop to match pwm cycle timed to end of pwm on

    /* Enable counter */
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableAllOutputs(TIM1);
    /* Force update generation */
    LL_TIM_GenerateEvent_UPDATE(TIM1);

#ifdef USE_ADC_INPUT

#else
    LL_TIM_CC_EnableChannel(IC_TIMER_REGISTER, IC_TIMER_CHANNEL); // input capture and output compare
    LL_TIM_EnableCounter(IC_TIMER_REGISTER);
#endif

		//换相任务禁用了
    LL_TIM_EnableCounter(COM_TIMER); // commutation_timer priority 0
    LL_TIM_GenerateEvent_UPDATE(COM_TIMER);
    LL_TIM_EnableIT_UPDATE(COM_TIMER);
    COM_TIMER->DIER &= ~((0x1UL << (0U))); // disable for now.
	
		/* Enable LPTIM */
		//LL_LPTIM_Enable(UTILITY_TIMER);
		//LL_LPTIM_DisableResetAfterRead(UTILITY_TIMER);
    //LL_TIM_EnableCounter(UTILITY_TIMER);
    //LL_TIM_GenerateEvent_UPDATE(UTILITY_TIMER);
		
    //
    LL_TIM_EnableCounter(INTERVAL_TIMER);
    LL_TIM_GenerateEvent_UPDATE(INTERVAL_TIMER);

		//COM_TIMER 初始被禁用， 10khz 定时器负责启动
    LL_TIM_EnableCounter(TEN_KHZ_TIMER); // 10khz timer
    LL_TIM_GenerateEvent_UPDATE(TEN_KHZ_TIMER);
    TEN_KHZ_TIMER->DIER |= (0x1UL << (0U)); // enable interrupt 使能溢出中断
    // RCC->APB2ENR  &= ~(1 << 22);  // turn debug off
		
#ifdef USE_ADC
    ADC_Init();
    enableADC_DMA();
    activateADC();
#endif


    __IO uint32_t wait_loop_index = 0;
    /* Enable comparator */
    LL_COMP_Enable(MAIN_COMP);

    wait_loop_index = ((LL_COMP_DELAY_STARTUP_US * (SystemCoreClock / (100000 * 2))) / 10);
    while (wait_loop_index != 0) {
        wait_loop_index--;
    }
		
		//软件设置exti 15 processDshot 处理信号，优先级2 比DMA输入捕获低？高？
    NVIC_SetPriority(EXTI4_15_IRQn, 2);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
    EXTI->IMR |= LL_EXTI_LINE_15;
}
