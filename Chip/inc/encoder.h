#ifndef __ENCODER_H__
#define __ENCODER_H__
#endif
//#define INT_COUNT 0xFFFF
#undef  VECTOR_101
#define  VECTOR_101  lptmr_pulse_get 
//extern void lptmr_pulse_get(void);
//void FTM1_INPUT_IRQHandler(void);
void GetPulse(void);