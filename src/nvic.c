/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         STM32F407VE
 *
 * Type:           module
 *
 * Description:    helper functions for the nested vector interrupt controller
 *
 * Compiler:       ANSI-C
 *
 * Filename:       nvic.c
 *
 * Version:        1.0
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  03.03.2020
 *******************************************************************************
   Modification History:
   [1.0]    03.03.2020    Tobias Plüss <tpluess@ieee.org>
   - created
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "nvic.h"
#include "stm32f407.h"
#include "misc.h"

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

extern uint32_t StackTop; /* defined by the linker */

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

#define REGISTERS(name, addr) static volatile uint32_t* const name = \
(volatile uint32_t* const)(addr)

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);
extern void vPortSVCHandler(void);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

REGISTERS(NVIC_ISER, 0xE000E100);
REGISTERS(NVIC_ICER, 0xE000E180);
REGISTERS(NVIC_ISPR, 0xE000E200);
REGISTERS(NVIC_ICPR, 0xE000E280);
REGISTERS(NVIC_IABR, 0xE000E300);
REGISTERS(NVIC_IPR, 0xE000E400);

static uint8_t default_prio = 0u;

__attribute__((aligned (1024))) static funcptr_t vector_table[] =
{
    /* __initial_sp */                      (funcptr_t)&StackTop,
    /* Reset_Handler */                     0,
    /* NMI_Handler */                       0,
    /* HardFault_Handler */                 0,
    /* MemManage_Handler */                 0,
    /* BusFault_Handler */                  0,
    /* UsageFault_Handler */                0,
    /* 0 */                                 0,
    /* 0 */                                 0,
    /* 0 */                                 0,
    /* 0 */                                 0,
    /* SVC_Handler */                       (funcptr_t)vPortSVCHandler,
    /* DebugMon_Handler */                  0,
    /* 0 */                                 0,
    /* PendSV_Handler */                    (funcptr_t)xPortPendSVHandler,
    /* SysTick_Handler */                   (funcptr_t)xPortSysTickHandler,
    /* 0: WWDG_IRQHandler */                0,
    /* 1: PVD_IRQHandler */                 0,
    /* 2: TAMPER_STAMP_IRQHandler */        0,
    /* 3: RTC_WKUP_IRQHandler */            0,
    /* 4: FLASH_IRQHandler */               0,
    /* 5: RCC_IRQHandler */                 0,
    /* 6: EXTI0_IRQHandler */               0,
    /* 7: EXTI1_IRQHandler */               0,
    /* 8: EXTI2_TS_IRQHandler */            0,
    /* 9: EXTI3_IRQHandler */               0,
    /* 10: EXTI4_IRQHandler */              0,
    /* 11: DMA1_Channel1_IRQHandler */      0,
    /* 12: DMA1_Channel2_IRQHandler */      0,
    /* 13: DMA1_Channel3_IRQHandler */      0,
    /* 14: DMA1_Channel4_IRQHandler */      0,
    /* 15: DMA1_Channel5_IRQHandler */      0,
    /* 16: DMA1_Channel6_IRQHandler */      0,
    /* 17: DMA1_Channel7_IRQHandler */      0,
    /* 18: ADC1_2_IRQHandler */             0,
    /* 19: USB_HP_CAN1_TX_IRQHandler */     0,
    /* 20: USB_LP_CAN1_RX0_IRQHandler */    0,
    /* 21: CAN1_RX1_IRQHandler */           0,
    /* 22: CAN1_SCE_IRQHandler */           0,
    /* 23: EXTI9_5_IRQHandler */            0,
    /* 24: TIM1_BRK_TIM15_IRQHandler */     0,
    /* 25: TIM1_UP_TIM16_IRQHandler */      0,
    /* 26: TIM1_TRG_COM_TIM17_IRQHandler */ 0,
    /* 27: TIM1_CC_IRQHandler */            0,
    /* 28: TIM2_IRQHandler */               0,
    /* 29: TIM3_IRQHandler */               0,
    /* 30: TIM4_IRQHandler */               0,
    /* 31: I2C1_EV_IRQHandler */            0,
    /* 32: I2C1_ER_IRQHandler */            0,
    /* 33: I2C2_EV_IRQHandler */            0,
    /* 34: I2C2_ER_IRQHandler */            0,
    /* 35: SPI1_IRQHandler */               0,
    /* 36: SPI2_IRQHandler */               0,
    /* 37: USART1_IRQHandler */             0,
    /* 38: USART2_IRQHandler */             0,
    /* 39: USART3_IRQHandler */             0,
    /* 40: EXTI15_10_IRQHandler */          0,
    /* 41: RTC_Alarm_IRQHandler */          0,
    /* 42: USBWakeUp_IRQHandler */          0,
    /* 43: TIM8_BRK_IRQHandler */           0,
    /* 44: TIM8_UP_IRQHandler */            0,
    /* 45: TIM8_TRG_COM_IRQHandler */       0,
    /* 46: TIM8_CC_IRQHandler */            0,
    /* 47: ADC3_IRQHandler */               0,
    /* 48: 0 */                             0,
    /* 49: 0 */                             0,
    /* 50: 0 */                             0,
    /* 51: SPI3_IRQHandler */               0,
    /* 52: UART4_IRQHandler */              0,
    /* 53: UART5_IRQHandler */              0,
    /* 54: TIM6_DAC_IRQHandler */           0,
    /* 55: TIM7_IRQHandler */               0,
    /* 56: DMA2_Channel1_IRQHandler */      0,
    /* 57: DMA2_Channel2_IRQHandler */      0,
    /* 58: DMA2_Channel3_IRQHandler */      0,
    /* 59: DMA2_Channel4_IRQHandler */      0,
    /* 60: DMA2_Channel5_IRQHandler */      0,
    /* 61: ADC4_IRQHandler */               0,
    /* 62: 0 */                             0,
    /* 63: 0 */                             0,
    /* 64: COMP1_2_3_IRQHandler */          0,
    /* 65: COMP4_5_6_IRQHandler */          0,
    /* 66: COMP7_IRQHandler */              0,
    /* 67: 0 */                             0,
    /* 68: 0 */                             0,
    /* 69: 0 */                             0,
    /* 70: 0 */                             0,
    /* 71: 0 */                             0,
    /* 72: 0 */                             0,
    /* 73: 0 */                             0,
    /* 74: USB_HP_IRQHandler */             0,
    /* 75: USB_LP_IRQHandler */             0,
    /* 76: USBWakeUp_RMP_IRQHandler */      0,
    /* 77: 0 */                             0,
    /* 78: 0 */                             0,
    /* 79: 0 */                             0,
    /* 80: 0 */                             0,
    /* 81: FPU_IRQHandler */                0
};

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void vic_init(void)
{
  VTOR = (uint32_t)vector_table;
  AIRCR = 0x05FA0000;
  //AIRCR = 0x05FA0300;

  uint32_t orig = NVIC_IPR[0];
  NVIC_IPR[0] = 0xffu;

  /* all interrupts use one priority higher than the lowest possible one.
     the lowest possible interrupt priority should be used by the pendsv and
     systick etc. */
  default_prio = NVIC_IPR[0] - 1;
  NVIC_IPR[0] = orig;

  /*IP0 = 0x60606060u;
  IP1 = 0x60606060u;
  IP2 = 0x60606060u;
  IP3 = 0x60606060u;
  IP4 = 0x60606060u;
  IP5 = 0x60606060u;
  IP6 = 0x60606060u;
  IP7 = 0x60606060u;
  IP8 = 0x60606060u;
  IP9 = 0x60606060u;
  IP10 = 0x60606060u;
  IP11 = 0x60606060u;
  IP12 = 0x60606060u;
  IP13 = 0x60606060u;
  IP14 = 0x60606060u;
  IP15 = 0x60606060u;
  IP16 = 0x60606060u;
  IP17 = 0x60606060u;
  IP18 = 0x60606060u;
  IP19 = 0x60606060u;*/
}

void vic_enableirq(int32_t intnum, funcptr_t func)
{
  vector_table[intnum + 16] = func;

  if(intnum >= 0)
  {
    int32_t index = intnum / 32;
    NVIC_ISER[index] = (1u << intnum);

    index = intnum / 4;
    NVIC_IPR[index] = default_prio << (intnum % 4);
  }
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/