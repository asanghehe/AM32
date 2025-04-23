#include "comparator.h"
#include "targets.h"
#include "common.h"


uint8_t getCompOutputLevel() { 
	return LL_COMP_ReadOutputLevel(MAIN_COMP);
}


void maskPhaseInterrupts(){
		EXTI->IMR &= ~(EXTI_LINE);
		LL_EXTI_ClearFlag(EXTI_LINE);
}

void enableCompInterrupts(){
    EXTI->IMR |= EXTI_LINE;
}

void changeCompInput() {
	
		/** 原本的比较顺序 电流大，卡死, 能转，有力**/
		if (step == 1 || step == 4) {   // c floating
			LL_COMP_ConfigInputs(MAIN_COMP , COMMON_COMP, PHASE_C_COMP);
		}
		if (step == 2 || step == 5) {     // a floating
			LL_COMP_ConfigInputs(MAIN_COMP, COMMON_COMP, PHASE_A_COMP);
		}
		if (step == 3 || step == 6) {      // b floating
			LL_COMP_ConfigInputs(MAIN_COMP, COMMON_COMP, PHASE_B_COMP);
		}
		
		
		/** 不堵死电流的顺序， 不太能转，没力
		if (step == 1 || step == 4) {   // c floating
			LL_COMP_ConfigInputs(MAIN_COMP, COMMON_COMP, PHASE_A_COMP);
		}
		if (step == 2 || step == 5) {     // a floating
			LL_COMP_ConfigInputs(MAIN_COMP, COMMON_COMP, PHASE_B_COMP);
		}
		if (step == 3 || step == 6) {      // b floating
			LL_COMP_ConfigInputs(MAIN_COMP , COMMON_COMP, PHASE_C_COMP);
		}
		**/
		
		/** 怎么调都没有 原来的好
		if (step == 1 || step == 4) {   // c floating
			LL_COMP_ConfigInputs(MAIN_COMP, COMMON_COMP, PHASE_B_COMP);
		}
		if (step == 2 || step == 5) {     // a floating
			LL_COMP_ConfigInputs(MAIN_COMP, COMMON_COMP, PHASE_A_COMP);
		}
		if (step == 3 || step == 6) {      // b floating
			LL_COMP_ConfigInputs(MAIN_COMP , COMMON_COMP, PHASE_C_COMP);
		}
		**/
		
	if (rising){
		  LL_EXTI_DisableRisingTrig(EXTI_LINE);
		  LL_EXTI_EnableFallingTrig(EXTI_LINE);
	}else{                          // falling bemf
		  LL_EXTI_EnableRisingTrig(EXTI_LINE);
		  LL_EXTI_DisableFallingTrig(EXTI_LINE);
	}
}

