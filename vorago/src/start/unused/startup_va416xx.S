/**************************************************************************//**
 * @file     startup_ARMCM4.S
 * @brief    CMSIS-Core(M) Device Startup File for Cortex-M4 Device
 * @version  V2.0.1
 * @date     23. July 2019
 ******************************************************************************/
/*
 * Copyright (c) 2009-2019 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* This startup file was provided by Thales Alenia Space. If the default one is problematic,
 * It might help to look at this file as well */

                .syntax  unified
                .arch    armv7e-m

                .section .vectors
                .align   2
                .globl   __Vectors
                .globl   __Vectors_End
                .globl   __Vectors_Size
__Vectors:
                .long    __StackTop                         /*     Top of Stack */
                .long    Reset_Handler                      /*     Reset Handler */
                .long    NMI_Handler                        /* -14 NMI Handler */
                .long    HardFault_Handler                  /* -13 Hard Fault Handler */
                .long    MemManage_Handler                  /* -12 MPU Fault Handler */
                .long    BusFault_Handler                   /* -11 Bus Fault Handler */
                .long    UsageFault_Handler                 /* -10 Usage Fault Handler */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    SVC_Handler                        /*  -5 SVCall Handler */
                .long    DebugMon_Handler                   /*  -4 Debug Monitor Handler */
                .long    0                                  /*     Reserved */
                .long    PendSV_Handler                     /*  -2 PendSV Handler */
                .long    SysTick_Handler                    /*  -1 SysTick Handler */

//               Interrupts
                .long     OC0_IRQHandler             //  0: Always 0
                .long     OC1_IRQHandler             //  1: Always 0
                .long     OC2_IRQHandler             //  2: Always 0
                .long     OC3_IRQHandler             //  3: Always 0
                .long     OC4_IRQHandler             //  4: Always 0
                .long     OC5_IRQHandler             //  5: Always 0
                .long     OC6_IRQHandler             //  6: Always 0
                .long     OC7_IRQHandler             //  7: Always 0
                .long     OC8_IRQHandler             //  8: Always 0
                .long     OC9_IRQHandler             //  9: Always 0
                .long     OC10_IRQHandler            // 10: Always 0
                .long     OC11_IRQHandler            // 11: Always 0
                .long     OC12_IRQHandler            // 12: Always 0
                .long     OC13_IRQHandler            // 13: Always 0
                .long     OC14_IRQHandler            // 14: Always 0
                .long     OC15_IRQHandler            // 15: Always 0
                .long     SPI0_TX_IRQHandler         // 16: SPI0 TX
                .long     SPI0_RX_IRQHandler         // 17: SPI0 RX
                .long     SPI1_TX_IRQHandler         // 18: SPI1 TX
                .long     SPI1_RX_IRQHandler         // 19: SPI1 RX
                .long     SPI2_TX_IRQHandler         // 20: SPI2 TX
                .long     SPI2_RX_IRQHandler         // 21: SPI2 RX
                .long     SPI3_TX_IRQHandler         // 22: SPI3 TX
                .long     SPI3_RX_IRQHandler         // 23: SPI3 RX
                .long     UART0_TX_IRQHandler        // 24: UART0 TX
                .long     UART0_RX_IRQHandler        // 25: UART0 RX
                .long     UART1_TX_IRQHandler        // 26: UART1 TX
                .long     UART1_RX_IRQHandler        // 27: UART1 RX
                .long     UART2_TX_IRQHandler        // 28: UART2 TX
                .long     UART2_RX_IRQHandler        // 29: UART2 RX
                .long     I2C0_MS_IRQHandler         // 30: I2C0_MS
                .long     I2C0_SL_IRQHandler         // 31: I2C0_SL
                .long     I2C1_MS_IRQHandler         // 32: I2C1_MS
                .long     I2C1_SL_IRQHandler         // 33: I2C1_SL
                .long     I2C2_MS_IRQHandler         // 34: I2C2_MS
                .long     I2C2_SL_IRQHandler         // 35: I2C2_SL
                .long     Ethernet_IRQHandler        // 36: Ethernet TX
                .long     OC37_IRQHandler            // 37: Always 0
                .long     SpW_IRQHandler             // 38: Space Wire
                .long     OC39_IRQHandler            // 39: Always 0
                .long     DAC0_IRQHandler            // 40: DAC 0
                .long     DAC1_IRQHandler            // 41: DAC 1
                .long     TRNG_IRQHandler            // 42: Random Number Generator
                .long     DMA_Error_IRQHandler       // 43: DMA error
                .long     ADC_IRQHandler             // 44: ADC
                .long     LoCLK_IRQHandler           // 45: LoCLK
                .long     LVD_IRQHandler             // 46: LVD
                .long     WDT_IRQHandler             // 47: Watchdog
                .long     TIM0_IRQHandler            // 48: Timer 0
                .long     TIM1_IRQHandler            // 49: Timer 1
                .long     TIM2_IRQHandler            // 50: Timer 2
                .long     TIM3_IRQHandler            // 51: Timer 3
                .long     TIM4_IRQHandler            // 52: Timer 4
                .long     TIM5_IRQHandler            // 53: Timer 5
                .long     TIM6_IRQHandler            // 54: Timer 6
                .long     TIM7_IRQHandler            // 55: Timer 7
                .long     TIM8_IRQHandler            // 56: Timer 8
                .long     TIM9_IRQHandler            // 57: Timer 9
                .long     TIM10_IRQHandler           // 58: Timer 10
                .long     TIM11_IRQHandler           // 59: Timer 11
                .long     TIM12_IRQHandler           // 60: Timer 12
                .long     TIM13_IRQHandler           // 61: Timer 13
                .long     TIM14_IRQHandler           // 62: Timer 14
                .long     TIM15_IRQHandler           // 63: Timer 15
                .long     TIM16_IRQHandler           // 64: Timer 16
                .long     TIM17_IRQHandler           // 65: Timer 17
                .long     TIM18_IRQHandler           // 66: Timer 18
                .long     TIM19_IRQHandler           // 67: Timer 19
                .long     TIM20_IRQHandler           // 68: Timer 20
                .long     TIM21_IRQHandler           // 69: Timer 21
                .long     TIM22_IRQHandler           // 70: Timer 22
                .long     TIM23_IRQHandler           // 71: Timer 23
                .long     CAN0_IRQHandler            // 72: CAN 0
                .long     OC73_IRQHandler            // 73: Always 0
                .long     CAN1_IRQHandler            // 74: CAN 1
                .long     OC75_IRQHandler            // 75: Always 0
                .long     EDAC_MBE_IRQHandler        // 76: EDAC Multi Bit Error
                .long     EDAC_SBE_IRQHandler        // 77: EDAC Single Bit Error
                .long     PA0_IRQHandler             // 78: PORTA 0
                .long     PA1_IRQHandler             // 79: PORTA 1
                .long     PA2_IRQHandler             // 80: PORTA 2
                .long     PA3_IRQHandler             // 81: PORTA 3
                .long     PA4_IRQHandler             // 82: PORTA 4
                .long     PA5_IRQHandler             // 83: PORTA 5
                .long     PA6_IRQHandler             // 84: PORTA 6
                .long     PA7_IRQHandler             // 85: PORTA 7
                .long     PA8_IRQHandler             // 86: PORTA 8
                .long     PA9_IRQHandler             // 87: PORTA 9
                .long     PA10_IRQHandler            // 88: PORTA 10
                .long     PA11_IRQHandler            // 89: PORTA 11
                .long     PA12_IRQHandler            // 90: PORTA 12
                .long     PA13_IRQHandler            // 91: PORTA 13
                .long     PA14_IRQHandler            // 92: PORTA 14
                .long     PA15_IRQHandler            // 93: PORTA 15
                .long     PB0_IRQHandler             // 94: PORTB 0
                .long     PB1_IRQHandler             // 95: PORTB 1
                .long     PB2_IRQHandler             // 96: PORTB 2
                .long     PB3_IRQHandler             // 97: PORTB 3
                .long     PB4_IRQHandler             // 98: PORTB 4
                .long     PB5_IRQHandler             // 99: PORTB 5
                .long     PB6_IRQHandler             // 100: PORTB 6
                .long     PB7_IRQHandler             // 101: PORTB 7
                .long     PB8_IRQHandler             // 102: PORTB 8
                .long     PB9_IRQHandler             // 103: PORTB 9
                .long     PB10_IRQHandler            // 104: PORTB 10
                .long     PB11_IRQHandler            // 105: PORTB 11
                .long     PB12_IRQHandler            // 106: PORTB 12
                .long     PB13_IRQHandler            // 107: PORTB 13
                .long     PB14_IRQHandler            // 108: PORTB 14
                .long     PB15_IRQHandler            // 109: PORTB 15
                .long     PC0_IRQHandler             // 110: PORTC 0
                .long     PC1_IRQHandler             // 111: PORTC 1
                .long     PC2_IRQHandler             // 112: PORTC 2
                .long     PC3_IRQHandler             // 113: PORTC 3
                .long     PC4_IRQHandler             // 114: PORTC 4
                .long     PC5_IRQHandler             // 115: PORTC 5
                .long     PC6_IRQHandler             // 116: PORTC 6
                .long     PC7_IRQHandler             // 117: PORTC 7
                .long     PC8_IRQHandler             // 118: PORTC 8
                .long     PC9_IRQHandler             // 119: PORTC 9
                .long     PC10_IRQHandler            // 120: PORTC 10
                .long     PC11_IRQHandler            // 121: PORTC 11
                .long     PC12_IRQHandler            // 122: PORTC 12
                .long     PC13_IRQHandler            // 123: PORTC 13
                .long     PC14_IRQHandler            // 124: PORTC 14
                .long     PC15_IRQHandler            // 125: PORTC 15
                .long     PD0_IRQHandler             // 126: PORTD 0
                .long     PD1_IRQHandler             // 127: PORTD 1
                .long     PD2_IRQHandler             // 128: PORTD 2
                .long     PD3_IRQHandler             // 129: PORTD 3
                .long     PD4_IRQHandler             // 130: PORTD 4
                .long     PD5_IRQHandler             // 131: PORTD 5
                .long     PD6_IRQHandler             // 132: PORTD 6
                .long     PD7_IRQHandler             // 133: PORTD 7
                .long     PD8_IRQHandler             // 134: PORTD 8
                .long     PD9_IRQHandler             // 135: PORTD 9
                .long     PD10_IRQHandler            // 136: PORTD 10
                .long     PD11_IRQHandler            // 137: PORTD 11
                .long     PD12_IRQHandler            // 138: PORTD 12
                .long     PD13_IRQHandler            // 139: PORTD 13
                .long     PD14_IRQHandler            // 140: PORTD 14
                .long     PD15_IRQHandler            // 141: PORTD 15
                .long     PE0_IRQHandler             // 142: PORTE 0
                .long     PE1_IRQHandler             // 143: PORTE 1
                .long     PE2_IRQHandler             // 144: PORTE 2
                .long     PE3_IRQHandler             // 145: PORTE 3
                .long     PE4_IRQHandler             // 146: PORTE 4
                .long     PE5_IRQHandler             // 147: PORTE 5
                .long     PE6_IRQHandler             // 148: PORTE 6
                .long     PE7_IRQHandler             // 149: PORTE 7
                .long     PE8_IRQHandler             // 150: PORTE 8
                .long     PE9_IRQHandler             // 151: PORTE 9
                .long     PE10_IRQHandler            // 152: PORTE 10
                .long     PE11_IRQHandler            // 153: PORTE 11
                .long     PE12_IRQHandler            // 154: PORTE 12
                .long     PE13_IRQHandler            // 155: PORTE 13
                .long     PE14_IRQHandler            // 156: PORTE 14
                .long     PE15_IRQHandler            // 157: PORTE 15
                .long     PF0_IRQHandler             // 158: PORTF 0
                .long     PF1_IRQHandler             // 159: PORTF 1
                .long     PF2_IRQHandler             // 160: PORTF 2
                .long     PF3_IRQHandler             // 161: PORTF 3
                .long     PF4_IRQHandler             // 162: PORTF 4
                .long     PF5_IRQHandler             // 163: PORTF 5
                .long     PF6_IRQHandler             // 164: PORTF 6
                .long     PF7_IRQHandler             // 165: PORTF 7
                .long     PF8_IRQHandler             // 166: PORTF 8
                .long     PF9_IRQHandler             // 167: PORTF 9
                .long     PF10_IRQHandler            // 168: PORTF 10
                .long     PF11_IRQHandler            // 169: PORTF 11
                .long     PF12_IRQHandler            // 170: PORTF 12
                .long     PF13_IRQHandler            // 171: PORTF 13
                .long     PF14_IRQHandler            // 172: PORTF 14
                .long     PF15_IRQHandler            // 173: PORTF 15
                .long     DMA_Active_0_IRQHandler    // 174: DMA Active 0
                .long     DMA_Active_1_IRQHandler    // 175: DMA Active 1
                .long     DMA_Active_2_IRQHandler    // 176: DMA Active 2
                .long     DMA_Active_3_IRQHandler    // 177: DMA Active 3
                .long     DMA_Done_0_IRQHandler      // 178: DMA Done 0
                .long     DMA_Done_1_IRQHandler      // 179: DMA Done 1
                .long     DMA_Done_2_IRQHandler      // 180: DMA Done 2
                .long     DMA_Done_3_IRQHandler      // 181: DMA Done 3
                .long     I2C0_MS_RX_IRQHandler      // 182: I2C0 Master RX
                .long     I2C0_MS_TX_IRQHandler      // 183: I2C0 Master TX
                .long     I2C0_SL_RX_IRQHandler      // 184: I2C0 Slave RX
                .long     I2C0_SL_TX_IRQHandler      // 185: I2C0 Slave TX
                .long     I2C1_MS_RX_IRQHandler      // 186: I2C1 Master RX
                .long     I2C1_MS_TX_IRQHandler      // 187: I2C1 Master TX
                .long     I2C1_SL_RX_IRQHandler      // 188: I2C1 Slave RX
                .long     I2C1_SL_TX_IRQHandler      // 189: I2C1 Slave TX
                .long     I2C2_MS_RX_IRQHandler      // 190: I2C2 Master RX
                .long     I2C2_MS_TX_IRQHandler      // 191: I2C2 Master TX
                .long     I2C2_SL_RX_IRQHandler      // 192: I2C2 Slave RX
                .long     I2C2_SL_TX_IRQHandler      // 193: I2C2 Slave TX
                .long     FPU_IRQHandler             // 194: FPU
                .long     TXEV_IRQHandler            // 195: TXEV

                .space   (214 * 4)                          /* Interrupts 10 .. 224 are left out */
__Vectors_End:
                .equ     __Vectors_Size, __Vectors_End - __Vectors
                .size    __Vectors, . - __Vectors


                .thumb
                .section .text
                .align   2

                .thumb_func
                .type    Reset_Handler, %function
                .globl   Reset_Handler
                .fnstart
Reset_Handler:
                bl       SystemInit

//...                ldr      r4, =__copy_table_start__
//...                ldr      r5, =__copy_table_end__
                ldr      r4, =__init_array_start
                ldr      r5, =__init_array_end

.L_loop0:
                cmp      r4, r5
                bge      .L_loop0_done
                ldr      r1, [r4]
                ldr      r2, [r4, #4]
                ldr      r3, [r4, #8]

.L_loop0_0:
                subs     r3, #4
                ittt     ge
                ldrge    r0, [r1, r3]
                strge    r0, [r2, r3]
                bge      .L_loop0_0

                adds     r4, #12
                b        .L_loop0
.L_loop0_done:

//...                ldr      r3, =__zero_table_start__
//...                ldr      r4, =__zero_table_end__
                ldr      r3, =__fini_array_start
                ldr      r4, =__fini_array_end

.L_loop2:
                cmp      r3, r4
                bge      .L_loop2_done
                ldr      r1, [r3]
                ldr      r2, [r3, #4]
                movs     r0, 0

.L_loop2_0:
                subs     r2, #4
                itt      ge
                strge    r0, [r1, r2]
                bge      .L_loop2_0

                adds     r3, #8
                b        .L_loop2
.L_loop2_done:

                bl       _start

                .fnend
                .size    Reset_Handler, . - Reset_Handler

/* The default macro is not used for HardFault_Handler
 * because this results in a poor debug illusion.
 */
                .thumb_func
                .type    HardFault_Handler, %function
                .weak    HardFault_Handler
                .fnstart
HardFault_Handler:
                b        .
                .fnend
                .size    HardFault_Handler, . - HardFault_Handler

                .thumb_func
                .type    Default_Handler, %function
                .weak    Default_Handler
                .fnstart
Default_Handler:
                b        .
                .fnend
                .size    Default_Handler, . - Default_Handler

/* Macro to define default exception/interrupt handlers.
 * Default handler are weak symbols with an endless loop.
 * They can be overwritten by real handlers.
 */
                .macro   Set_Default_Handler  Handler_Name
                .weak    \Handler_Name
                .set     \Handler_Name, Default_Handler
                .endm


/* Default exception/interrupt handler */

                Set_Default_Handler  NMI_Handler
                Set_Default_Handler  MemManage_Handler
                Set_Default_Handler  BusFault_Handler
                Set_Default_Handler  UsageFault_Handler
                Set_Default_Handler  SVC_Handler
                Set_Default_Handler  DebugMon_Handler
                Set_Default_Handler  PendSV_Handler
                Set_Default_Handler  SysTick_Handler

//                Set_Default_Handler  Interrupt0_Handler
//                Set_Default_Handler  Interrupt1_Handler
//                Set_Default_Handler  Interrupt2_Handler
//                Set_Default_Handler  Interrupt3_Handler
//                Set_Default_Handler  Interrupt4_Handler
//                Set_Default_Handler  Interrupt5_Handler
//                Set_Default_Handler  Interrupt6_Handler
//                Set_Default_Handler  Interrupt7_Handler
//                Set_Default_Handler  Interrupt8_Handler
//                Set_Default_Handler  Interrupt9_Handler
//				
                Set_Default_Handler  OC0_IRQHandler             //  0: Always 0
                Set_Default_Handler  OC1_IRQHandler             //  1: Always 0
                Set_Default_Handler  OC2_IRQHandler             //  2: Always 0
                Set_Default_Handler  OC3_IRQHandler             //  3: Always 0
                Set_Default_Handler  OC4_IRQHandler             //  4: Always 0
                Set_Default_Handler  OC5_IRQHandler             //  5: Always 0
                Set_Default_Handler  OC6_IRQHandler             //  6: Always 0
                Set_Default_Handler  OC7_IRQHandler             //  7: Always 0
                Set_Default_Handler  OC8_IRQHandler             //  8: Always 0
                Set_Default_Handler  OC9_IRQHandler             //  9: Always 0
                Set_Default_Handler  OC10_IRQHandler            // 10: Always 0
                Set_Default_Handler  OC11_IRQHandler            // 11: Always 0
                Set_Default_Handler  OC12_IRQHandler            // 12: Always 0
                Set_Default_Handler  OC13_IRQHandler            // 13: Always 0
                Set_Default_Handler  OC14_IRQHandler            // 14: Always 0
                Set_Default_Handler  OC15_IRQHandler            // 15: Always 0
                Set_Default_Handler  SPI0_TX_IRQHandler         // 16: SPI0 TX
                Set_Default_Handler  SPI0_RX_IRQHandler         // 17: SPI0 RX
                Set_Default_Handler  SPI1_TX_IRQHandler         // 18: SPI1 TX
                Set_Default_Handler  SPI1_RX_IRQHandler         // 19: SPI1 RX
                Set_Default_Handler  SPI2_TX_IRQHandler         // 20: SPI2 TX
                Set_Default_Handler  SPI2_RX_IRQHandler         // 21: SPI2 RX
                Set_Default_Handler  SPI3_TX_IRQHandler         // 22: SPI3 TX
                Set_Default_Handler  SPI3_RX_IRQHandler         // 23: SPI3 RX
                Set_Default_Handler  UART0_TX_IRQHandler        // 24: UART0 TX
                Set_Default_Handler  UART0_RX_IRQHandler        // 25: UART0 RX
                Set_Default_Handler  UART1_TX_IRQHandler        // 26: UART1 TX
                Set_Default_Handler  UART1_RX_IRQHandler        // 27: UART1 RX
                Set_Default_Handler  UART2_TX_IRQHandler        // 28: UART2 TX
                Set_Default_Handler  UART2_RX_IRQHandler        // 29: UART2 RX
                Set_Default_Handler  I2C0_MS_IRQHandler         // 30: I2C0_MS
                Set_Default_Handler  I2C0_SL_IRQHandler         // 31: I2C0_SL
                Set_Default_Handler  I2C1_MS_IRQHandler         // 32: I2C1_MS
                Set_Default_Handler  I2C1_SL_IRQHandler         // 33: I2C1_SL
                Set_Default_Handler  I2C2_MS_IRQHandler         // 34: I2C2_MS
                Set_Default_Handler  I2C2_SL_IRQHandler         // 35: I2C2_SL
                Set_Default_Handler  Ethernet_IRQHandler        // 36: Ethernet TX
                Set_Default_Handler  OC37_IRQHandler            // 37: Always 0
                Set_Default_Handler  SpW_IRQHandler             // 38: Space Wire
                Set_Default_Handler  OC39_IRQHandler            // 39: Always 0
                Set_Default_Handler  DAC0_IRQHandler            // 40: DAC 0
                Set_Default_Handler  DAC1_IRQHandler            // 41: DAC 1
                Set_Default_Handler  TRNG_IRQHandler            // 42: Random Number Generator
                Set_Default_Handler  DMA_Error_IRQHandler       // 43: DMA error
                Set_Default_Handler  ADC_IRQHandler             // 44: ADC
                Set_Default_Handler  LoCLK_IRQHandler           // 45: LoCLK
                Set_Default_Handler  LVD_IRQHandler             // 46: LVD
                Set_Default_Handler  WDT_IRQHandler             // 47: Watchdog
                Set_Default_Handler  TIM0_IRQHandler            // 48: Timer 0
                Set_Default_Handler  TIM1_IRQHandler            // 49: Timer 1
                Set_Default_Handler  TIM2_IRQHandler            // 50: Timer 2
                Set_Default_Handler  TIM3_IRQHandler            // 51: Timer 3
                Set_Default_Handler  TIM4_IRQHandler            // 52: Timer 4
                Set_Default_Handler  TIM5_IRQHandler            // 53: Timer 5
                Set_Default_Handler  TIM6_IRQHandler            // 54: Timer 6
                Set_Default_Handler  TIM7_IRQHandler            // 55: Timer 7
                Set_Default_Handler  TIM8_IRQHandler            // 56: Timer 8
                Set_Default_Handler  TIM9_IRQHandler            // 57: Timer 9
                Set_Default_Handler  TIM10_IRQHandler           // 58: Timer 10
                Set_Default_Handler  TIM11_IRQHandler           // 59: Timer 11
                Set_Default_Handler  TIM12_IRQHandler           // 60: Timer 12
                Set_Default_Handler  TIM13_IRQHandler           // 61: Timer 13
                Set_Default_Handler  TIM14_IRQHandler           // 62: Timer 14
                Set_Default_Handler  TIM15_IRQHandler           // 63: Timer 15
                Set_Default_Handler  TIM16_IRQHandler           // 64: Timer 16
                Set_Default_Handler  TIM17_IRQHandler           // 65: Timer 17
                Set_Default_Handler  TIM18_IRQHandler           // 66: Timer 18
                Set_Default_Handler  TIM19_IRQHandler           // 67: Timer 19
                Set_Default_Handler  TIM20_IRQHandler           // 68: Timer 20
                Set_Default_Handler  TIM21_IRQHandler           // 69: Timer 21
                Set_Default_Handler  TIM22_IRQHandler           // 70: Timer 22
                Set_Default_Handler  TIM23_IRQHandler           // 71: Timer 23
                Set_Default_Handler  CAN0_IRQHandler            // 72: CAN 0
                Set_Default_Handler  OC73_IRQHandler            // 73: Always 0
                Set_Default_Handler  CAN1_IRQHandler            // 74: CAN 1
                Set_Default_Handler  OC75_IRQHandler            // 75: Always 0
                Set_Default_Handler  EDAC_MBE_IRQHandler        // 76: EDAC Multi Bit Error
                Set_Default_Handler  EDAC_SBE_IRQHandler        // 77: EDAC Single Bit Error
                Set_Default_Handler  PA0_IRQHandler             // 78: PORTA 0
                Set_Default_Handler  PA1_IRQHandler             // 79: PORTA 1
                Set_Default_Handler  PA2_IRQHandler             // 80: PORTA 2
                Set_Default_Handler  PA3_IRQHandler             // 81: PORTA 3
                Set_Default_Handler  PA4_IRQHandler             // 82: PORTA 4
                Set_Default_Handler  PA5_IRQHandler             // 83: PORTA 5
                Set_Default_Handler  PA6_IRQHandler             // 84: PORTA 6
                Set_Default_Handler  PA7_IRQHandler             // 85: PORTA 7
                Set_Default_Handler  PA8_IRQHandler             // 86: PORTA 8
                Set_Default_Handler  PA9_IRQHandler             // 87: PORTA 9
                Set_Default_Handler  PA10_IRQHandler            // 88: PORTA 10
                Set_Default_Handler  PA11_IRQHandler            // 89: PORTA 11
                Set_Default_Handler  PA12_IRQHandler            // 90: PORTA 12
                Set_Default_Handler  PA13_IRQHandler            // 91: PORTA 13
                Set_Default_Handler  PA14_IRQHandler            // 92: PORTA 14
                Set_Default_Handler  PA15_IRQHandler            // 93: PORTA 15
                Set_Default_Handler  PB0_IRQHandler             // 94: PORTB 0
                Set_Default_Handler  PB1_IRQHandler             // 95: PORTB 1
                Set_Default_Handler  PB2_IRQHandler             // 96: PORTB 2
                Set_Default_Handler  PB3_IRQHandler             // 97: PORTB 3
                Set_Default_Handler  PB4_IRQHandler             // 98: PORTB 4
                Set_Default_Handler  PB5_IRQHandler             // 99: PORTB 5
                Set_Default_Handler  PB6_IRQHandler             // 100: PORTB 6
                Set_Default_Handler  PB7_IRQHandler             // 101: PORTB 7
                Set_Default_Handler  PB8_IRQHandler             // 102: PORTB 8
                Set_Default_Handler  PB9_IRQHandler             // 103: PORTB 9
                Set_Default_Handler  PB10_IRQHandler            // 104: PORTB 10
                Set_Default_Handler  PB11_IRQHandler            // 105: PORTB 11
                Set_Default_Handler  PB12_IRQHandler            // 106: PORTB 12
                Set_Default_Handler  PB13_IRQHandler            // 107: PORTB 13
                Set_Default_Handler  PB14_IRQHandler            // 108: PORTB 14
                Set_Default_Handler  PB15_IRQHandler            // 109: PORTB 15
                Set_Default_Handler  PC0_IRQHandler             // 110: PORTC 0
                Set_Default_Handler  PC1_IRQHandler             // 111: PORTC 1
                Set_Default_Handler  PC2_IRQHandler             // 112: PORTC 2
                Set_Default_Handler  PC3_IRQHandler             // 113: PORTC 3
                Set_Default_Handler  PC4_IRQHandler             // 114: PORTC 4
                Set_Default_Handler  PC5_IRQHandler             // 115: PORTC 5
                Set_Default_Handler  PC6_IRQHandler             // 116: PORTC 6
                Set_Default_Handler  PC7_IRQHandler             // 117: PORTC 7
                Set_Default_Handler  PC8_IRQHandler             // 118: PORTC 8
                Set_Default_Handler  PC9_IRQHandler             // 119: PORTC 9
                Set_Default_Handler  PC10_IRQHandler            // 120: PORTC 10
                Set_Default_Handler  PC11_IRQHandler            // 121: PORTC 11
                Set_Default_Handler  PC12_IRQHandler            // 122: PORTC 12
                Set_Default_Handler  PC13_IRQHandler            // 123: PORTC 13
                Set_Default_Handler  PC14_IRQHandler            // 124: PORTC 14
                Set_Default_Handler  PC15_IRQHandler            // 125: PORTC 15
                Set_Default_Handler  PD0_IRQHandler             // 126: PORTD 0
                Set_Default_Handler  PD1_IRQHandler             // 127: PORTD 1
                Set_Default_Handler  PD2_IRQHandler             // 128: PORTD 2
                Set_Default_Handler  PD3_IRQHandler             // 129: PORTD 3
                Set_Default_Handler  PD4_IRQHandler             // 130: PORTD 4
                Set_Default_Handler  PD5_IRQHandler             // 131: PORTD 5
                Set_Default_Handler  PD6_IRQHandler             // 132: PORTD 6
                Set_Default_Handler  PD7_IRQHandler             // 133: PORTD 7
                Set_Default_Handler  PD8_IRQHandler             // 134: PORTD 8
                Set_Default_Handler  PD9_IRQHandler             // 135: PORTD 9
                Set_Default_Handler  PD10_IRQHandler            // 136: PORTD 10
                Set_Default_Handler  PD11_IRQHandler            // 137: PORTD 11
                Set_Default_Handler  PD12_IRQHandler            // 138: PORTD 12
                Set_Default_Handler  PD13_IRQHandler            // 139: PORTD 13
                Set_Default_Handler  PD14_IRQHandler            // 140: PORTD 14
                Set_Default_Handler  PD15_IRQHandler            // 141: PORTD 15
                Set_Default_Handler  PE0_IRQHandler             // 142: PORTE 0
                Set_Default_Handler  PE1_IRQHandler             // 143: PORTE 1
                Set_Default_Handler  PE2_IRQHandler             // 144: PORTE 2
                Set_Default_Handler  PE3_IRQHandler             // 145: PORTE 3
                Set_Default_Handler  PE4_IRQHandler             // 146: PORTE 4
                Set_Default_Handler  PE5_IRQHandler             // 147: PORTE 5
                Set_Default_Handler  PE6_IRQHandler             // 148: PORTE 6
                Set_Default_Handler  PE7_IRQHandler             // 149: PORTE 7
                Set_Default_Handler  PE8_IRQHandler             // 150: PORTE 8
                Set_Default_Handler  PE9_IRQHandler             // 151: PORTE 9
                Set_Default_Handler  PE10_IRQHandler            // 152: PORTE 10
                Set_Default_Handler  PE11_IRQHandler            // 153: PORTE 11
                Set_Default_Handler  PE12_IRQHandler            // 154: PORTE 12
                Set_Default_Handler  PE13_IRQHandler            // 155: PORTE 13
                Set_Default_Handler  PE14_IRQHandler            // 156: PORTE 14
                Set_Default_Handler  PE15_IRQHandler            // 157: PORTE 15
                Set_Default_Handler  PF0_IRQHandler             // 158: PORTF 0
                Set_Default_Handler  PF1_IRQHandler             // 159: PORTF 1
                Set_Default_Handler  PF2_IRQHandler             // 160: PORTF 2
                Set_Default_Handler  PF3_IRQHandler             // 161: PORTF 3
                Set_Default_Handler  PF4_IRQHandler             // 162: PORTF 4
                Set_Default_Handler  PF5_IRQHandler             // 163: PORTF 5
                Set_Default_Handler  PF6_IRQHandler             // 164: PORTF 6
                Set_Default_Handler  PF7_IRQHandler             // 165: PORTF 7
                Set_Default_Handler  PF8_IRQHandler             // 166: PORTF 8
                Set_Default_Handler  PF9_IRQHandler             // 167: PORTF 9
                Set_Default_Handler  PF10_IRQHandler            // 168: PORTF 10
                Set_Default_Handler  PF11_IRQHandler            // 169: PORTF 11
                Set_Default_Handler  PF12_IRQHandler            // 170: PORTF 12
                Set_Default_Handler  PF13_IRQHandler            // 171: PORTF 13
                Set_Default_Handler  PF14_IRQHandler            // 172: PORTF 14
                Set_Default_Handler  PF15_IRQHandler            // 173: PORTF 15
                Set_Default_Handler  DMA_Active_0_IRQHandler    // 174: DMA Active 0
                Set_Default_Handler  DMA_Active_1_IRQHandler    // 175: DMA Active 1
                Set_Default_Handler  DMA_Active_2_IRQHandler    // 176: DMA Active 2
                Set_Default_Handler  DMA_Active_3_IRQHandler    // 177: DMA Active 3
                Set_Default_Handler  DMA_Done_0_IRQHandler      // 178: DMA Done 0
                Set_Default_Handler  DMA_Done_1_IRQHandler      // 179: DMA Done 1
                Set_Default_Handler  DMA_Done_2_IRQHandler      // 180: DMA Done 2
                Set_Default_Handler  DMA_Done_3_IRQHandler      // 181: DMA Done 3
                Set_Default_Handler  I2C0_MS_RX_IRQHandler      // 182: I2C0 Master RX
                Set_Default_Handler  I2C0_MS_TX_IRQHandler      // 183: I2C0 Master TX
                Set_Default_Handler  I2C0_SL_RX_IRQHandler      // 184: I2C0 Slave RX
                Set_Default_Handler  I2C0_SL_TX_IRQHandler      // 185: I2C0 Slave TX
                Set_Default_Handler  I2C1_MS_RX_IRQHandler      // 186: I2C1 Master RX
                Set_Default_Handler  I2C1_MS_TX_IRQHandler      // 187: I2C1 Master TX
                Set_Default_Handler  I2C1_SL_RX_IRQHandler      // 188: I2C1 Slave RX
                Set_Default_Handler  I2C1_SL_TX_IRQHandler      // 189: I2C1 Slave TX
                Set_Default_Handler  I2C2_MS_RX_IRQHandler      // 190: I2C2 Master RX
                Set_Default_Handler  I2C2_MS_TX_IRQHandler      // 191: I2C2 Master TX
                Set_Default_Handler  I2C2_SL_RX_IRQHandler      // 192: I2C2 Slave RX
                Set_Default_Handler  I2C2_SL_TX_IRQHandler      // 193: I2C2 Slave TX
                Set_Default_Handler  FPU_IRQHandler             // 194: FPU
                Set_Default_Handler  TXEV_IRQHandler            // 195: TXEV

                .end

