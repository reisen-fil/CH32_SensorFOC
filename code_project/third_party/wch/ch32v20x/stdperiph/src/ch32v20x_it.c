/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v20x_it.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/29
 * Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v20x_it.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
// void HardFault_Handler(void)
// {
//   printf("mepc :%08x\r\n", __get_MEPC());

//   printf("mcause:%08x\r\n", __get_MCAUSE());

//   printf("mtval :%08x\r\n", __get_MTVAL());
  
//   while (1)
//   {
     
//   }
// }

// void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// void TIM2_IRQHandler(void)  
// {
//     if(TIM_GetITStatus(TIM2, TIM_IT_Update)==SET)
//     {
//         remaining++;
//         if(remaining >= 500)
//         {
//             remaining = 0;
//             if(test_cnt == 0) 
//             {
//                 GPIO_SetBits(GPIOC,GPIO_Pin_14);
//                 test_cnt = 1;
//             }
//             else 
//             {
//                 GPIO_ResetBits(GPIOC,GPIO_Pin_14);
//                 test_cnt = 0;            
//             }
//         }

//     }
//     TIM_ClearITPendingBit( TIM2, TIM_IT_Update );    
// }


