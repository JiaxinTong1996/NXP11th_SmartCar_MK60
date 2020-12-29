#ifndef __CAMERA_TEST_H__
#define __CAMERA_TEST_H__
#endif
#include "common.h"
#include "include.h"

extern uint8 imgbuff[CAMERA_SIZE];

void Mycamera(void);
void PORTB_IRQHandler (void);
void DMA_CH0_IRQHandler (void);
void StopMotor();
void ovtFlag();
void ovtStart();
void ovtCul();
//extern void img_extract(uint8 *dst, uint8 *src, uint32_t srclen);


