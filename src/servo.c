#include "common.h"
#include "include.h"
#include "Servo.h"
#include "MK60_FTM.h"
#include "PID.h"
#define S3010_FTM   FTM2
#define S3010_CH    FTM_CH0
#define S3010_HZ    (70)

//#define servMotorCenture  1310//san 1185  si 1310  
//#define servMotorLeft     1430//san 1300  si 1430
//#define servMotorRight    1190//san 1070  si 1190
//#define servPram           8
extern uint32 servPWMDuty;
void Servo(void)
{   
    uint8 i;
  //  ftm_pwm_init(S3010_FTM, S3010_CH,S3010_HZ,1170);      //³õÊ¼»¯ ¶æ»ú PWM
                                                            //1170 1070  1300
    
                                  

       ftm_pwm_duty(S3010_FTM,S3010_CH,servPWMDuty);
}
void ServoInit()
{
  ftm_pwm_init(S3010_FTM, S3010_CH,S3010_HZ,1175);
}
