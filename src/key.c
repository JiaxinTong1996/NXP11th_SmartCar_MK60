#include "common.h"
#include "include.h"
#include "key.h"
#include "motor.h"
//volatile uint32 Moduty=15;
extern int16 speed_out_set;
extern int16 speed_out_set_str;
void KeyScan()
{
    set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler);
   
    enable_irq (PORTE_IRQn);                                //使能PORTE中断
}

void PORTE_IRQHandler(void)
{  
    uint8  n = 0;
    uint8  n2 = 0;
    uint8  n3 = 0;
    uint8  n4 = 0;
    n = 9;
    n2 = 10;
    n3=11;
    n4=12;
    if(PORTE_ISFR & (1 << n))          
      {
          PORTE_ISFR  = (1 << n);        //写1清中断标志位
          speed_out_set_str=speed_out_set_str+10;
      }
    if(PORTE_ISFR & (1 << n2))          
      {
          PORTE_ISFR  = (1 << n2);        //写1清中断标志位
          speed_out_set_str=speed_out_set_str-10;
      }
    if(PORTE_ISFR & (1 << n3))          
      {
          PORTE_ISFR  = (1 << n3);        //写1清中断标志位
          speed_out_set=speed_out_set+10;
      }
    if(PORTE_ISFR & (1 << n4))          
      {
          PORTE_ISFR  = (1 << n4);        //写1清中断标志位
          speed_out_set=speed_out_set-10;
      }

 }


