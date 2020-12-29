#include    "MK60_it.h"
#include "common.h"
#include "include.h"

void PORTB_IRQHandler()
{
    uint8  n=0;   //引脚号
    uint32 flag;

    while(!PORTB_ISFR);
    flag = PORTB_ISFR;
    PORTB_ISFR  = ~0;                                   //清中断标志位

    n = 17;                                             //场中断
    if(flag & (1 << n))                                 //PTB17触发中断
    {
        camera_vsync();
    }
 #if ( CAMERA_USE_HREF == 1 )                            //使用行中断
    n = 21;
    if(flag & (1 << n))                                 //PTB21触发中断
    {
        camera_href();
    }
#endif
}

/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */

void DMA_CH0_IRQHandler()
{
   //DMA_IRQ_CLEAN(DMA_CH0);  更加直接
   camera_dma();
   //DMA_DIS(DMA_CH0);

}

