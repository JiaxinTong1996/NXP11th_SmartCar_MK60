#include "common.h"
#include "include.h"
#include "encoder.h"

//#define INT_COUNT 0xFFFF
uint16 count;
float act_speed;
float act_last_speed;
extern vuint8 LPT_INT_count;
int16 tee=0;
void GetPulse(void)
{  
   DisableInterrupts;
 //  PIT_Flag_Clear(PIT0); 
    tee++;
    lptmr_pulse_get();
    lptmr_pulse_clean();
    count+=LPT_INT_count * 5000;

    act_speed=(int)count;
    act_last_speed=act_speed;
    PIT_Flag_Clear(PIT0); 
    EnableInterrupts;
}

