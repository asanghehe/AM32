/*
 * peripherals.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#endif /* PERIPHERALS_H_ */

#include "main.h"

#define INTERVAL_TIMER_COUNT (INTERVAL_TIMER->CNT)

#define RELOAD_WATCHDOG_COUNTER() (LL_IWDG_ReloadCounter(IWDG))
//#define RELOAD_WATCHDOG_COUNTER() 

#define DISABLE_COM_TIMER_INT() (COM_TIMER->DIER &= ~((0x1UL << (0U))))
//#define ENABLE_COM_TIMER_INT() (COM_TIMER->DIER |= (0x1UL << (0U)))	//没用到

#define SET_AND_ENABLE_COM_INT(time)                                  \
    (COM_TIMER->CNT = 0, COM_TIMER->ARR = time, COM_TIMER->SR = 0x00, \
        COM_TIMER->DIER |= (0x1UL << (0U)))
				
				
#define SET_INTERVAL_TIMER_COUNT(intertime) (INTERVAL_TIMER->CNT = intertime)

#define SET_PRESCALER_PWM(presc) (TIM1->PSC = presc)
#define SET_AUTO_RELOAD_PWM(relval) (TIM1->ARR = relval)
#define SET_DUTY_CYCLE_ALL(newdc) \
    (TIM1->CCR1 = newdc, TIM1->CCR2 = newdc, TIM1->CCR3 = newdc)


void initAfterJump(void);
void initCorePeripherals(void);

void MX_IWDG_Init(void);

void resetInputCaptureTimer();
void setPWMCompare1(uint16_t compareone);
void setPWMCompare2(uint16_t comparetwo);
void setPWMCompare3(uint16_t comparethree);
void enableCorePeripherals(void);
void reloadWatchDogCounter(void);
void generatePwmTimerEvent(void);
