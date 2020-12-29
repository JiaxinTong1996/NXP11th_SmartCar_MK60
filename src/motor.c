#include "common.h"
#include "include.h"
#include "Motor.h"
#include "MK60_gpio.h"
#include "MK60_FTM.h"
#include "VCAN_LED.H"
#include "servo.h"

//滑行模式下，频率应该是 30~100。
//常规模式下，频率应该是 20k 左右
#if 0
#define MOTOR_HZ    (50)
#else
#define MOTOR_HZ    (20*1000)
#endif

extern float act_speed;
extern float act_last_speed;
extern int32 centureErr;
int motorErr=0;
int motorErr1=0;
int aim_speed;
int motor_last_pid=0;
float motor_pid=0;
int motor_max=1000;
int motor_min=0;
int16 speed_out_set=210;//210
int16 speed_out_set_str=330;//330
int16 motor_kp=630;//630
float kpp=0;
int16 motor_ki=4;//4  
int32 out;




void Motor(void)
{   // motor_pid//out
    ftm_pwm_duty(MOTOR_FTM,MOTOR1_PWM,out); 
}
void MotorInit()
{
     ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,0);
}
void MotorDuty()
{
//   speed_out_set=210;
//   speed_out_set_str=300;
   if(centureErr<5&&centureErr>-5)//5
    aim_speed=speed_out_set_str;
   else
    aim_speed=speed_out_set;
    motorErr=aim_speed-act_speed;
    motorErr1=aim_speed-act_last_speed;
    motor_last_pid=motor_pid;
    kpp=motor_kp/10;
    out=motor_last_pid+motor_ki*(motorErr-motorErr1)+(kpp*motorErr);
   if(out>9999)  out=10000;
   if(out<1)  out=0;
}