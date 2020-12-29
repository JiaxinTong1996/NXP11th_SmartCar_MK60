#include    "MK60_it.h"
#include "common.h"
#include "include.h"

void PORTB_IRQHandler()
{
    uint8  n=0;   //���ź�
    uint32 flag;

    while(!PORTB_ISFR);
    flag = PORTB_ISFR;
    PORTB_ISFR  = ~0;                                   //���жϱ�־λ

    n = 17;                                             //���ж�
    if(flag & (1 << n))                                 //PTB17�����ж�
    {
        camera_vsync();
    }
 #if ( CAMERA_USE_HREF == 1 )                            //ʹ�����ж�
    n = 21;
    if(flag & (1 << n))                                 //PTB21�����ж�
    {
        camera_href();
    }
#endif
}

/*!
 *  @brief      DMA0�жϷ�����
 *  @since      v5.0
 */

void DMA_CH0_IRQHandler()
{
   //DMA_IRQ_CLEAN(DMA_CH0);  ����ֱ��
   camera_dma();
   //DMA_DIS(DMA_CH0);

}

