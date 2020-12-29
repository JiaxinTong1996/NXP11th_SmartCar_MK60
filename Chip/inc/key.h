#ifndef __KEY_H__
#define __KEY_H__
#endif




void PORTE_IRQHandler(void);        
void key_handler(void);             //按键按下的测试中断服务函数
void KeyScan();
void dutyDis(uint32 duty);