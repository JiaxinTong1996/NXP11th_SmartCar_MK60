#include "common.h"
#include "include.h"
#include "motor.h"
#include "camera_test.h"   
#include "OLED.h"
#include "servo.h"
#include "PID.h"
#include "key.h"
#include "encoder.h"

#define S3010_FTM   FTM2
#define S3010_CH    FTM_CH0
#define S3010_HZ    (70)
#define INT_COUNT 0xFFFF

extern int stopflag;
void MyInit();
void  LPT_Handler(void);
void PITInit();
void main()             
{
     DisableInterrupts;
     MyInit();
   // OLED_Print(0,3,"22");
  //   DELAY();
     PITInit();
     EnableInterrupts; 
    
     
     while(1)
       {
   /*    led_init(LED0);
       led(LED0, LED_ON);*/
      //gpio_set (PTE25, 1);
       
       Mycamera();
       Servo();
       MotorDuty();
       Motor();
       servPWMDutyDis();
       centureErrDis(); 
       DutyDis();
       KeyScan();
//       StopMotor();
//       if(stopflag==1)
//       {break;}

     
    }
}
extern volatile PIDConst SevPID;
 void MyInit()
 {  
    camera_init(imgbuff);
    OLED_Init();
    ServoInit();
    MotorInit();
    port_init(PTE9, ALT1 | IRQ_FALLING | PULLUP ); //按键左一
    port_init(PTE10,ALT1 | IRQ_FALLING | PULLUP ); //按键左三
    port_init(PTE11, ALT1 | IRQ_FALLING | PULLUP ); //按键左2
    port_init(PTE12,ALT1 | IRQ_FALLING | PULLUP ); //按键左4
    SevPID.ExcOutput=1;   
    InitPID(&SevPID,9,0,3);//9//3si
   // port_init(PTE25,ALT1 | IRQ_FALLING | PULLDOWN );
    
    
    
    //pit_init(PIT1,10);
 }
 void PITInit()
 {
   lptmr_pulse_init(LPT0_ALT1,INT_COUNT,LPT_Rising);//A19  脉冲计数初始化
   set_vector_handler(PIT0_VECTORn,GetPulse);
// set_vector_handler(LPTMR_VECTORn,lptmr_pulse_get);
   pit_init_ms(PIT0,10);
  // pit_init_ms(PIT1,1);
  // set_vector_handler(PIT1_VECTORn,StopMotor);
   set_irq_priority((INT_LPTimer -16),6) ;    //设置优先级
   set_vector_handler(INT_LPTimer,LPT_Handler);  //设置中断地址函数

  
 }
vuint8 LPT_INT_count = 0;

void  LPT_Handler(void)
{
  LPTMR0_CSR |= LPTMR_CSR_TCF_MASK;   //清除LPTMR比较标志
  LPT_INT_count++;                    //中断溢出加1
}






