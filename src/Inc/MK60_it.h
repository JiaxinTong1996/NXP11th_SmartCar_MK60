#ifndef __MK60_IT_H__
#define __MK60_IT_H__

/*                          重新定义中断向量表
 *  先取消默认的中断向量元素宏定义        #undef  VECTOR_xxx
 *  在重新定义到自己编写的中断函数      #define VECTOR_xxx    xxx_IRQHandler
 *  例如：
 *       #undef  VECTOR_003                         先取消映射到中断向量表里的中断函数地址宏定义
 *       #define VECTOR_003    HardFault_Handler    重新定义硬件上访中断服务函数
*/

#undef VECTOR_016
#define VECTOR_016 DMA_CH0_IRQHandler
extern void DMA_CH0_IRQHandler(void);

#undef  VECTOR_104
#define  VECTOR_104 PORTB_IRQHandler
extern void PORTB_IRQHandler(void);






#endif  //__MK60_IT_H__
