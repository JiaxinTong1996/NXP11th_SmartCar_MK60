#ifndef __MK60_IT_H__
#define __MK60_IT_H__

/*                          ���¶����ж�������
 *  ��ȡ��Ĭ�ϵ��ж�����Ԫ�غ궨��        #undef  VECTOR_xxx
 *  �����¶��嵽�Լ���д���жϺ���      #define VECTOR_xxx    xxx_IRQHandler
 *  ���磺
 *       #undef  VECTOR_003                         ��ȡ��ӳ�䵽�ж�����������жϺ�����ַ�궨��
 *       #define VECTOR_003    HardFault_Handler    ���¶���Ӳ���Ϸ��жϷ�����
*/

#undef VECTOR_016
#define VECTOR_016 DMA_CH0_IRQHandler
extern void DMA_CH0_IRQHandler(void);

#undef  VECTOR_104
#define  VECTOR_104 PORTB_IRQHandler
extern void PORTB_IRQHandler(void);






#endif  //__MK60_IT_H__
