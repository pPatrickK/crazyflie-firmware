/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * nvic.c - Contains all Cortex-M3 processor exceptions handlers
 */
#include "exti.h"
#include "led.h"
//#include "i2croutines.h"
#include "i2cdev.h"
#include "ws2812.h"

#if defined(DEBUG_PRINT_ON_SEGGER_RTT)
  #include "debug.h"
  #define FAULT_PRINT    DEBUG_PRINT
#else
  #ifdef PLATFORM_CF1
  #include "uart.h"
  #define FAULT_PRINT    uartPrintf
  #else
  #include "uart1.h"
  #define FAULT_PRINT    uart1Printf
  #endif
#endif

#define DONT_DISCARD __attribute__((used))

void nvicInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}

/**
 * @brief  This function handles SysTick Handler.
 */
extern void tickFreeRTOS(void);

#ifndef PLATFORM_CF1
extern void SD_disk_timerproc (void);
#endif

void DONT_DISCARD SysTick_Handler(void)
{
    tickFreeRTOS();

#ifndef PLATFORM_CF1
    // TODO krri Should we really call this from here? Find better solution
    SD_disk_timerproc();
#endif
}

#ifdef NVIC_NOT_USED_BY_FREERTOS

/**
  * @brief  This function handles SVCall exception.
  */
void DONT_DISCARD SVC_Handler(void)
{
  FAULT_PRINT("[ERROR] SVC\n");
}

/**
 * @brief  This function handles PendSV_Handler exception.
 */
void DONT_DISCARD PendSV_Handler(void)
{
  FAULT_PRINT("[ERROR] PendSV\n");
}
#endif

/**
  * @brief  This function handles NMI exception.
  */
void DONT_DISCARD NMI_Handler(void)
{
  FAULT_PRINT("[ERROR] NMI\n");
}

/**
 * @brief  This function handles Hard Fault exception.
 */
void DONT_DISCARD HardFault_Handler(void)
{
  FAULT_PRINT("[ERROR] HardFault\n");
  //http://www.st.com/mcu/forums-cat-6778-23.html
  //****************************************************
  //To test this application, you can use this snippet anywhere:
  // //Let's crash the MCU!
  // asm (" MOVS r0, #1 \n"
  // " LDM r0,{r1-r2} \n"
  // " BX LR; \n");
  asm( "TST LR, #4 \n"
  "ITE EQ \n"
  "MRSEQ R0, MSP \n"
  "MRSNE R0, PSP \n"
  "B printHardFault");
}

void DONT_DISCARD printHardFault(uint32_t* hardfaultArgs)
{
  unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;
  unsigned int stacked_lr;
  unsigned int stacked_pc;
  unsigned int stacked_psr;

  stacked_r0 = ((unsigned long) hardfaultArgs[0]);
  stacked_r1 = ((unsigned long) hardfaultArgs[1]);
  stacked_r2 = ((unsigned long) hardfaultArgs[2]);
  stacked_r3 = ((unsigned long) hardfaultArgs[3]);

  stacked_r12 = ((unsigned long) hardfaultArgs[4]);
  stacked_lr = ((unsigned long) hardfaultArgs[5]);
  stacked_pc = ((unsigned long) hardfaultArgs[6]);
  stacked_psr = ((unsigned long) hardfaultArgs[7]);

  FAULT_PRINT("[Hard fault handler]\n");
  FAULT_PRINT("R0 = %x\n", stacked_r0);
  FAULT_PRINT("R1 = %x\n", stacked_r1);
  FAULT_PRINT("R2 = %x\n", stacked_r2);
  FAULT_PRINT("R3 = %x\n", stacked_r3);
  FAULT_PRINT("R12 = %x\n", stacked_r12);
  FAULT_PRINT("LR = %x\n", stacked_lr);
  FAULT_PRINT("PC = %x\n", stacked_pc);
  FAULT_PRINT("PSR = %x\n", stacked_psr);
  FAULT_PRINT("BFAR = %x\n", (*((volatile unsigned int *)(0xE000ED38))));
  FAULT_PRINT("CFSR = %x\n", (*((volatile unsigned int *)(0xE000ED28))));
  FAULT_PRINT("HFSR = %x\n", (*((volatile unsigned int *)(0xE000ED2C))));
  FAULT_PRINT("DFSR = %x\n", (*((volatile unsigned int *)(0xE000ED30))));
  FAULT_PRINT("AFSR = %x\n", (*((volatile unsigned int *)(0xE000ED3C))));

  while (1)
  {}
}
/**
 * @brief  This function handles Memory Manage exception.
 */
void DONT_DISCARD MemManage_Handler(void)
{
  FAULT_PRINT("[ERROR] MemManage\n");
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
 * @brief  This function handles Bus Fault exception.
 */
void DONT_DISCARD BusFault_Handler(void)
{
  FAULT_PRINT("[ERROR] BusFault\n");
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
 * @brief  This function handles Usage Fault exception.
 */
void DONT_DISCARD UsageFault_Handler(void)
{
  FAULT_PRINT("[ERROR] UsageFault\n");
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
 * @brief  This function handles Debug Monitor exception.
 */
void DONT_DISCARD DebugMon_Handler(void)
{
  FAULT_PRINT("[ERROR] DebugMon\n");
}

void DONT_DISCARD DMA1_Stream5_IRQHandler(void)
{
  ws2812DmaIsr();
}
