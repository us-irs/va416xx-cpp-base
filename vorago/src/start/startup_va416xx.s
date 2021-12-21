;/**************************************************************************//**
; * @file     startup_ARMCM4.s
; * @brief    CMSIS Core Device Startup File for
; *           ARMCM4 Device Series
; * @version  V5.00
; * @date     02. March 2016
; ******************************************************************************/
;/*
; * Copyright (c) 2009-2016 ARM Limited. All rights reserved.
; *
; * SPDX-License-Identifier: Apache-2.0
; *
; * Licensed under the Apache License, Version 2.0 (the License); you may
; * not use this file except in compliance with the License.
; * You may obtain a copy of the License at
; *
; * www.apache.org/licenses/LICENSE-2.0
; *
; * Unless required by applicable law or agreed to in writing, software
; * distributed under the License is distributed on an AS IS BASIS, WITHOUT
; * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; * See the License for the specific language governing permissions and
; * limitations under the License.
; */

;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000C00

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                DCD     OC0_IRQHandler             ;  0: Always 0
                DCD     OC1_IRQHandler             ;  1: Always 0
                DCD     OC2_IRQHandler             ;  2: Always 0
                DCD     OC3_IRQHandler             ;  3: Always 0
                DCD     OC4_IRQHandler             ;  4: Always 0
                DCD     OC5_IRQHandler             ;  5: Always 0
                DCD     OC6_IRQHandler             ;  6: Always 0
                DCD     OC7_IRQHandler             ;  7: Always 0
                DCD     OC8_IRQHandler             ;  8: Always 0
                DCD     OC9_IRQHandler             ;  9: Always 0
                DCD     OC10_IRQHandler            ; 10: Always 0
                DCD     OC11_IRQHandler            ; 11: Always 0
                DCD     OC12_IRQHandler            ; 12: Always 0
                DCD     OC13_IRQHandler            ; 13: Always 0
                DCD     OC14_IRQHandler            ; 14: Always 0
                DCD     OC15_IRQHandler            ; 15: Always 0
                DCD     SPI0_TX_IRQHandler         ; 16: SPI0 TX
                DCD     SPI0_RX_IRQHandler         ; 17: SPI0 RX
                DCD     SPI1_TX_IRQHandler         ; 18: SPI1 TX
                DCD     SPI1_RX_IRQHandler         ; 19: SPI1 RX
                DCD     SPI2_TX_IRQHandler         ; 20: SPI2 TX
                DCD     SPI2_RX_IRQHandler         ; 21: SPI2 RX
                DCD     SPI3_TX_IRQHandler         ; 22: SPI3 TX
                DCD     SPI3_RX_IRQHandler         ; 23: SPI3 RX
                DCD     UART0_TX_IRQHandler        ; 24: UART0 TX
                DCD     UART0_RX_IRQHandler        ; 25: UART0 RX
                DCD     UART1_TX_IRQHandler        ; 26: UART1 TX
                DCD     UART1_RX_IRQHandler        ; 27: UART1 RX
                DCD     UART2_TX_IRQHandler        ; 28: UART2 TX
                DCD     UART2_RX_IRQHandler        ; 29: UART2 RX
                DCD     I2C0_MS_IRQHandler         ; 30: I2C0_MS
                DCD     I2C0_SL_IRQHandler         ; 31: I2C0_SL
                DCD     I2C1_MS_IRQHandler         ; 32: I2C1_MS
                DCD     I2C1_SL_IRQHandler         ; 33: I2C1_SL
                DCD     I2C2_MS_IRQHandler         ; 34: I2C2_MS
                DCD     I2C2_SL_IRQHandler         ; 35: I2C2_SL
                DCD     Ethernet_IRQHandler        ; 36: Ethernet TX
                DCD     OC37_IRQHandler            ; 37: Always 0
                DCD     SpW_IRQHandler             ; 38: Space Wire
                DCD     OC39_IRQHandler            ; 39: Always 0
                DCD     DAC0_IRQHandler            ; 40: DAC 0
                DCD     DAC1_IRQHandler            ; 41: DAC 1
                DCD     TRNG_IRQHandler            ; 42: Random Number Generator
                DCD     DMA_Error_IRQHandler       ; 43: DMA error
                DCD     ADC_IRQHandler             ; 44: ADC
                DCD     LoCLK_IRQHandler           ; 45: LoCLK
                DCD     LVD_IRQHandler             ; 46: LVD
                DCD     WDT_IRQHandler             ; 47: Watchdog
                DCD     TIM0_IRQHandler            ; 48: Timer 0
                DCD     TIM1_IRQHandler            ; 49: Timer 1
                DCD     TIM2_IRQHandler            ; 50: Timer 2
                DCD     TIM3_IRQHandler            ; 51: Timer 3
                DCD     TIM4_IRQHandler            ; 52: Timer 4
                DCD     TIM5_IRQHandler            ; 53: Timer 5
                DCD     TIM6_IRQHandler            ; 54: Timer 6
                DCD     TIM7_IRQHandler            ; 55: Timer 7
                DCD     TIM8_IRQHandler            ; 56: Timer 8
                DCD     TIM9_IRQHandler            ; 57: Timer 9
                DCD     TIM10_IRQHandler           ; 58: Timer 10
                DCD     TIM11_IRQHandler           ; 59: Timer 11
                DCD     TIM12_IRQHandler           ; 60: Timer 12
                DCD     TIM13_IRQHandler           ; 61: Timer 13
                DCD     TIM14_IRQHandler           ; 62: Timer 14
                DCD     TIM15_IRQHandler           ; 63: Timer 15
                DCD     TIM16_IRQHandler           ; 64: Timer 16
                DCD     TIM17_IRQHandler           ; 65: Timer 17
                DCD     TIM18_IRQHandler           ; 66: Timer 18
                DCD     TIM19_IRQHandler           ; 67: Timer 19
                DCD     TIM20_IRQHandler           ; 68: Timer 20
                DCD     TIM21_IRQHandler           ; 69: Timer 21
                DCD     TIM22_IRQHandler           ; 70: Timer 22
                DCD     TIM23_IRQHandler           ; 71: Timer 23
                DCD     CAN0_IRQHandler            ; 72: CAN 0
                DCD     OC73_IRQHandler            ; 73: Always 0
                DCD     CAN1_IRQHandler            ; 74: CAN 1
                DCD     OC75_IRQHandler            ; 75: Always 0
                DCD     EDAC_MBE_IRQHandler        ; 76: EDAC Multi Bit Error
                DCD     EDAC_SBE_IRQHandler        ; 77: EDAC Single Bit Error
                DCD     PA0_IRQHandler             ; 78: PORTA 0
                DCD     PA1_IRQHandler             ; 79: PORTA 1
                DCD     PA2_IRQHandler             ; 80: PORTA 2
                DCD     PA3_IRQHandler             ; 81: PORTA 3
                DCD     PA4_IRQHandler             ; 82: PORTA 4
                DCD     PA5_IRQHandler             ; 83: PORTA 5
                DCD     PA6_IRQHandler             ; 84: PORTA 6
                DCD     PA7_IRQHandler             ; 85: PORTA 7
                DCD     PA8_IRQHandler             ; 86: PORTA 8
                DCD     PA9_IRQHandler             ; 87: PORTA 9
                DCD     PA10_IRQHandler            ; 88: PORTA 10
                DCD     PA11_IRQHandler            ; 89: PORTA 11
                DCD     PA12_IRQHandler            ; 90: PORTA 12
                DCD     PA13_IRQHandler            ; 91: PORTA 13
                DCD     PA14_IRQHandler            ; 92: PORTA 14
                DCD     PA15_IRQHandler            ; 93: PORTA 15
                DCD     PB0_IRQHandler             ; 94: PORTB 0
                DCD     PB1_IRQHandler             ; 95: PORTB 1
                DCD     PB2_IRQHandler             ; 96: PORTB 2
                DCD     PB3_IRQHandler             ; 97: PORTB 3
                DCD     PB4_IRQHandler             ; 98: PORTB 4
                DCD     PB5_IRQHandler             ; 99: PORTB 5
                DCD     PB6_IRQHandler             ; 100: PORTB 6
                DCD     PB7_IRQHandler             ; 101: PORTB 7
                DCD     PB8_IRQHandler             ; 102: PORTB 8
                DCD     PB9_IRQHandler             ; 103: PORTB 9
                DCD     PB10_IRQHandler            ; 104: PORTB 10
                DCD     PB11_IRQHandler            ; 105: PORTB 11
                DCD     PB12_IRQHandler            ; 106: PORTB 12
                DCD     PB13_IRQHandler            ; 107: PORTB 13
                DCD     PB14_IRQHandler            ; 108: PORTB 14
                DCD     PB15_IRQHandler            ; 109: PORTB 15
                DCD     PC0_IRQHandler             ; 110: PORTC 0
                DCD     PC1_IRQHandler             ; 111: PORTC 1
                DCD     PC2_IRQHandler             ; 112: PORTC 2
                DCD     PC3_IRQHandler             ; 113: PORTC 3
                DCD     PC4_IRQHandler             ; 114: PORTC 4
                DCD     PC5_IRQHandler             ; 115: PORTC 5
                DCD     PC6_IRQHandler             ; 116: PORTC 6
                DCD     PC7_IRQHandler             ; 117: PORTC 7
                DCD     PC8_IRQHandler             ; 118: PORTC 8
                DCD     PC9_IRQHandler             ; 119: PORTC 9
                DCD     PC10_IRQHandler            ; 120: PORTC 10
                DCD     PC11_IRQHandler            ; 121: PORTC 11
                DCD     PC12_IRQHandler            ; 122: PORTC 12
                DCD     PC13_IRQHandler            ; 123: PORTC 13
                DCD     PC14_IRQHandler            ; 124: PORTC 14
                DCD     PC15_IRQHandler            ; 125: PORTC 15
                DCD     PD0_IRQHandler             ; 126: PORTD 0
                DCD     PD1_IRQHandler             ; 127: PORTD 1
                DCD     PD2_IRQHandler             ; 128: PORTD 2
                DCD     PD3_IRQHandler             ; 129: PORTD 3
                DCD     PD4_IRQHandler             ; 130: PORTD 4
                DCD     PD5_IRQHandler             ; 131: PORTD 5
                DCD     PD6_IRQHandler             ; 132: PORTD 6
                DCD     PD7_IRQHandler             ; 133: PORTD 7
                DCD     PD8_IRQHandler             ; 134: PORTD 8
                DCD     PD9_IRQHandler             ; 135: PORTD 9
                DCD     PD10_IRQHandler            ; 136: PORTD 10
                DCD     PD11_IRQHandler            ; 137: PORTD 11
                DCD     PD12_IRQHandler            ; 138: PORTD 12
                DCD     PD13_IRQHandler            ; 139: PORTD 13
                DCD     PD14_IRQHandler            ; 140: PORTD 14
                DCD     PD15_IRQHandler            ; 141: PORTD 15
                DCD     PE0_IRQHandler             ; 142: PORTE 0
                DCD     PE1_IRQHandler             ; 143: PORTE 1
                DCD     PE2_IRQHandler             ; 144: PORTE 2
                DCD     PE3_IRQHandler             ; 145: PORTE 3
                DCD     PE4_IRQHandler             ; 146: PORTE 4
                DCD     PE5_IRQHandler             ; 147: PORTE 5
                DCD     PE6_IRQHandler             ; 148: PORTE 6
                DCD     PE7_IRQHandler             ; 149: PORTE 7
                DCD     PE8_IRQHandler             ; 150: PORTE 8
                DCD     PE9_IRQHandler             ; 151: PORTE 9
                DCD     PE10_IRQHandler            ; 152: PORTE 10
                DCD     PE11_IRQHandler            ; 153: PORTE 11
                DCD     PE12_IRQHandler            ; 154: PORTE 12
                DCD     PE13_IRQHandler            ; 155: PORTE 13
                DCD     PE14_IRQHandler            ; 156: PORTE 14
                DCD     PE15_IRQHandler            ; 157: PORTE 15
                DCD     PF0_IRQHandler             ; 158: PORTF 0
                DCD     PF1_IRQHandler             ; 159: PORTF 1
                DCD     PF2_IRQHandler             ; 160: PORTF 2
                DCD     PF3_IRQHandler             ; 161: PORTF 3
                DCD     PF4_IRQHandler             ; 162: PORTF 4
                DCD     PF5_IRQHandler             ; 163: PORTF 5
                DCD     PF6_IRQHandler             ; 164: PORTF 6
                DCD     PF7_IRQHandler             ; 165: PORTF 7
                DCD     PF8_IRQHandler             ; 166: PORTF 8
                DCD     PF9_IRQHandler             ; 167: PORTF 9
                DCD     PF10_IRQHandler            ; 168: PORTF 10
                DCD     PF11_IRQHandler            ; 169: PORTF 11
                DCD     PF12_IRQHandler            ; 170: PORTF 12
                DCD     PF13_IRQHandler            ; 171: PORTF 13
                DCD     PF14_IRQHandler            ; 172: PORTF 14
                DCD     PF15_IRQHandler            ; 173: PORTF 15
                DCD     DMA_Active_0_IRQHandler    ; 174: DMA Active 0
                DCD     DMA_Active_1_IRQHandler    ; 175: DMA Active 1
                DCD     DMA_Active_2_IRQHandler    ; 176: DMA Active 2
                DCD     DMA_Active_3_IRQHandler    ; 177: DMA Active 3
                DCD     DMA_Done_0_IRQHandler      ; 178: DMA Done 0
                DCD     DMA_Done_1_IRQHandler      ; 179: DMA Done 1
                DCD     DMA_Done_2_IRQHandler      ; 180: DMA Done 2
                DCD     DMA_Done_3_IRQHandler      ; 181: DMA Done 3
                DCD     I2C0_MS_RX_IRQHandler      ; 182: I2C0 Master RX
                DCD     I2C0_MS_TX_IRQHandler      ; 183: I2C0 Master TX
                DCD     I2C0_SL_RX_IRQHandler      ; 184: I2C0 Slave RX
                DCD     I2C0_SL_TX_IRQHandler      ; 185: I2C0 Slave TX
                DCD     I2C1_MS_RX_IRQHandler      ; 186: I2C1 Master RX
                DCD     I2C1_MS_TX_IRQHandler      ; 187: I2C1 Master TX
                DCD     I2C1_SL_RX_IRQHandler      ; 188: I2C1 Slave RX
                DCD     I2C1_SL_TX_IRQHandler      ; 189: I2C1 Slave TX
                DCD     I2C2_MS_RX_IRQHandler      ; 190: I2C2 Master RX
                DCD     I2C2_MS_TX_IRQHandler      ; 191: I2C2 Master TX
                DCD     I2C2_SL_RX_IRQHandler      ; 192: I2C2 Slave RX
                DCD     I2C2_SL_TX_IRQHandler      ; 193: I2C2 Slave TX
                DCD     FPU_IRQHandler             ; 194: FPU
                DCD     TXEV_IRQHandler            ; 195: TXEV
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  OC0_IRQHandler             [WEAK] ;  0: Always 0
                EXPORT  OC1_IRQHandler             [WEAK] ;  1: Always 0
                EXPORT  OC2_IRQHandler             [WEAK] ;  2: Always 0
                EXPORT  OC3_IRQHandler             [WEAK] ;  3: Always 0
                EXPORT  OC4_IRQHandler             [WEAK] ;  4: Always 0
                EXPORT  OC5_IRQHandler             [WEAK] ;  5: Always 0
                EXPORT  OC6_IRQHandler             [WEAK] ;  6: Always 0
                EXPORT  OC7_IRQHandler             [WEAK] ;  7: Always 0
                EXPORT  OC8_IRQHandler             [WEAK] ;  8: Always 0
                EXPORT  OC9_IRQHandler             [WEAK] ;  9: Always 0
                EXPORT  OC10_IRQHandler            [WEAK] ; 10: Always 0
                EXPORT  OC11_IRQHandler            [WEAK] ; 11: Always 0
                EXPORT  OC12_IRQHandler            [WEAK] ; 12: Always 0
                EXPORT  OC13_IRQHandler            [WEAK] ; 13: Always 0
                EXPORT  OC14_IRQHandler            [WEAK] ; 14: Always 0
                EXPORT  OC15_IRQHandler            [WEAK] ; 15: Always 0
                EXPORT  SPI0_TX_IRQHandler         [WEAK] ; 16: SPI0 TX
                EXPORT  SPI0_RX_IRQHandler         [WEAK] ; 17: SPI0 RX
                EXPORT  SPI1_TX_IRQHandler         [WEAK] ; 18: SPI1 TX
                EXPORT  SPI1_RX_IRQHandler         [WEAK] ; 19: SPI1 RX
                EXPORT  SPI2_TX_IRQHandler         [WEAK] ; 20: SPI2 TX
                EXPORT  SPI2_RX_IRQHandler         [WEAK] ; 21: SPI2 RX
                EXPORT  SPI3_TX_IRQHandler         [WEAK] ; 22: SPI3 TX
                EXPORT  SPI3_RX_IRQHandler         [WEAK] ; 23: SPI3 RX
                EXPORT  UART0_TX_IRQHandler        [WEAK] ; 24: UART0 TX
                EXPORT  UART0_RX_IRQHandler        [WEAK] ; 25: UART0 RX
                EXPORT  UART1_TX_IRQHandler        [WEAK] ; 26: UART1 TX
                EXPORT  UART1_RX_IRQHandler        [WEAK] ; 27: UART1 RX
                EXPORT  UART2_TX_IRQHandler        [WEAK] ; 28: UART2 TX
                EXPORT  UART2_RX_IRQHandler        [WEAK] ; 29: UART2 RX
                EXPORT  I2C0_MS_IRQHandler         [WEAK] ; 30: I2C0_MS
                EXPORT  I2C0_SL_IRQHandler         [WEAK] ; 31: I2C0_SL
                EXPORT  I2C1_MS_IRQHandler         [WEAK] ; 32: I2C1_MS
                EXPORT  I2C1_SL_IRQHandler         [WEAK] ; 33: I2C1_SL
                EXPORT  I2C2_MS_IRQHandler         [WEAK] ; 34: I2C2_MS
                EXPORT  I2C2_SL_IRQHandler         [WEAK] ; 35: I2C2_SL
                EXPORT  Ethernet_IRQHandler        [WEAK] ; 36: Ethernet TX
                EXPORT  OC37_IRQHandler            [WEAK] ; 37: Always 0
                EXPORT  SpW_IRQHandler             [WEAK] ; 38: Space Wire
                EXPORT  OC39_IRQHandler            [WEAK] ; 39: Always 0
                EXPORT  DAC0_IRQHandler            [WEAK] ; 40: DAC 0
                EXPORT  DAC1_IRQHandler            [WEAK] ; 41: DAC 1
                EXPORT  TRNG_IRQHandler            [WEAK] ; 42: Random Number Generator
                EXPORT  DMA_Error_IRQHandler       [WEAK] ; 43: DMA error
                EXPORT  ADC_IRQHandler             [WEAK] ; 44: ADC
                EXPORT  LoCLK_IRQHandler           [WEAK] ; 45: LoCLK
                EXPORT  LVD_IRQHandler             [WEAK] ; 46: LVD
                EXPORT  WDT_IRQHandler             [WEAK] ; 47: Watchdog
                EXPORT  TIM0_IRQHandler            [WEAK] ; 48: Timer 0
                EXPORT  TIM1_IRQHandler            [WEAK] ; 49: Timer 1
                EXPORT  TIM2_IRQHandler            [WEAK] ; 50: Timer 2
                EXPORT  TIM3_IRQHandler            [WEAK] ; 51: Timer 3
                EXPORT  TIM4_IRQHandler            [WEAK] ; 52: Timer 4
                EXPORT  TIM5_IRQHandler            [WEAK] ; 53: Timer 5
                EXPORT  TIM6_IRQHandler            [WEAK] ; 54: Timer 6
                EXPORT  TIM7_IRQHandler            [WEAK] ; 55: Timer 7
                EXPORT  TIM8_IRQHandler            [WEAK] ; 56: Timer 8
                EXPORT  TIM9_IRQHandler            [WEAK] ; 57: Timer 9
                EXPORT  TIM10_IRQHandler           [WEAK] ; 58: Timer 10
                EXPORT  TIM11_IRQHandler           [WEAK] ; 59: Timer 11
                EXPORT  TIM12_IRQHandler           [WEAK] ; 60: Timer 12
                EXPORT  TIM13_IRQHandler           [WEAK] ; 61: Timer 13
                EXPORT  TIM14_IRQHandler           [WEAK] ; 62: Timer 14
                EXPORT  TIM15_IRQHandler           [WEAK] ; 63: Timer 15
                EXPORT  TIM16_IRQHandler           [WEAK] ; 64: Timer 16
                EXPORT  TIM17_IRQHandler           [WEAK] ; 65: Timer 17
                EXPORT  TIM18_IRQHandler           [WEAK] ; 66: Timer 18
                EXPORT  TIM19_IRQHandler           [WEAK] ; 67: Timer 19
                EXPORT  TIM20_IRQHandler           [WEAK] ; 68: Timer 20
                EXPORT  TIM21_IRQHandler           [WEAK] ; 69: Timer 21
                EXPORT  TIM22_IRQHandler           [WEAK] ; 70: Timer 22
                EXPORT  TIM23_IRQHandler           [WEAK] ; 71: Timer 23
                EXPORT  CAN0_IRQHandler            [WEAK] ; 72: CAN 0
                EXPORT  OC73_IRQHandler            [WEAK] ; 73: Always 0
                EXPORT  CAN1_IRQHandler            [WEAK] ; 74: CAN 1
                EXPORT  OC75_IRQHandler            [WEAK] ; 75: Always 0
                EXPORT  EDAC_MBE_IRQHandler        [WEAK] ; 76: EDAC Multi Bit Error
                EXPORT  EDAC_SBE_IRQHandler        [WEAK] ; 77: EDAC Single Bit Error
                EXPORT  PA0_IRQHandler             [WEAK] ; 78: PORTA 0
                EXPORT  PA1_IRQHandler             [WEAK] ; 79: PORTA 1
                EXPORT  PA2_IRQHandler             [WEAK] ; 80: PORTA 2
                EXPORT  PA3_IRQHandler             [WEAK] ; 81: PORTA 3
                EXPORT  PA4_IRQHandler             [WEAK] ; 82: PORTA 4
                EXPORT  PA5_IRQHandler             [WEAK] ; 83: PORTA 5
                EXPORT  PA6_IRQHandler             [WEAK] ; 84: PORTA 6
                EXPORT  PA7_IRQHandler             [WEAK] ; 85: PORTA 7
                EXPORT  PA8_IRQHandler             [WEAK] ; 86: PORTA 8
                EXPORT  PA9_IRQHandler             [WEAK] ; 87: PORTA 9
                EXPORT  PA10_IRQHandler            [WEAK] ; 88: PORTA 10
                EXPORT  PA11_IRQHandler            [WEAK] ; 89: PORTA 11
                EXPORT  PA12_IRQHandler            [WEAK] ; 90: PORTA 12
                EXPORT  PA13_IRQHandler            [WEAK] ; 91: PORTA 13
                EXPORT  PA14_IRQHandler            [WEAK] ; 92: PORTA 14
                EXPORT  PA15_IRQHandler            [WEAK] ; 93: PORTA 15
                EXPORT  PB0_IRQHandler             [WEAK] ; 94: PORTB 0
                EXPORT  PB1_IRQHandler             [WEAK] ; 95: PORTB 1
                EXPORT  PB2_IRQHandler             [WEAK] ; 96: PORTB 2
                EXPORT  PB3_IRQHandler             [WEAK] ; 97: PORTB 3
                EXPORT  PB4_IRQHandler             [WEAK] ; 98: PORTB 4
                EXPORT  PB5_IRQHandler             [WEAK] ; 99: PORTB 5
                EXPORT  PB6_IRQHandler             [WEAK] ; 100: PORTB 6
                EXPORT  PB7_IRQHandler             [WEAK] ; 101: PORTB 7
                EXPORT  PB8_IRQHandler             [WEAK] ; 102: PORTB 8
                EXPORT  PB9_IRQHandler             [WEAK] ; 103: PORTB 9
                EXPORT  PB10_IRQHandler            [WEAK] ; 104: PORTB 10
                EXPORT  PB11_IRQHandler            [WEAK] ; 105: PORTB 11
                EXPORT  PB12_IRQHandler            [WEAK] ; 106: PORTB 12
                EXPORT  PB13_IRQHandler            [WEAK] ; 107: PORTB 13
                EXPORT  PB14_IRQHandler            [WEAK] ; 108: PORTB 14
                EXPORT  PB15_IRQHandler            [WEAK] ; 109: PORTB 15
                EXPORT  PC0_IRQHandler             [WEAK] ; 110: PORTC 0
                EXPORT  PC1_IRQHandler             [WEAK] ; 111: PORTC 1
                EXPORT  PC2_IRQHandler             [WEAK] ; 112: PORTC 2
                EXPORT  PC3_IRQHandler             [WEAK] ; 113: PORTC 3
                EXPORT  PC4_IRQHandler             [WEAK] ; 114: PORTC 4
                EXPORT  PC5_IRQHandler             [WEAK] ; 115: PORTC 5
                EXPORT  PC6_IRQHandler             [WEAK] ; 116: PORTC 6
                EXPORT  PC7_IRQHandler             [WEAK] ; 117: PORTC 7
                EXPORT  PC8_IRQHandler             [WEAK] ; 118: PORTC 8
                EXPORT  PC9_IRQHandler             [WEAK] ; 119: PORTC 9
                EXPORT  PC10_IRQHandler            [WEAK] ; 120: PORTC 10
                EXPORT  PC11_IRQHandler            [WEAK] ; 121: PORTC 11
                EXPORT  PC12_IRQHandler            [WEAK] ; 122: PORTC 12
                EXPORT  PC13_IRQHandler            [WEAK] ; 123: PORTC 13
                EXPORT  PC14_IRQHandler            [WEAK] ; 124: PORTC 14
                EXPORT  PC15_IRQHandler            [WEAK] ; 125: PORTC 15
                EXPORT  PD0_IRQHandler             [WEAK] ; 126: PORTD 0
                EXPORT  PD1_IRQHandler             [WEAK] ; 127: PORTD 1
                EXPORT  PD2_IRQHandler             [WEAK] ; 128: PORTD 2
                EXPORT  PD3_IRQHandler             [WEAK] ; 129: PORTD 3
                EXPORT  PD4_IRQHandler             [WEAK] ; 130: PORTD 4
                EXPORT  PD5_IRQHandler             [WEAK] ; 131: PORTD 5
                EXPORT  PD6_IRQHandler             [WEAK] ; 132: PORTD 6
                EXPORT  PD7_IRQHandler             [WEAK] ; 133: PORTD 7
                EXPORT  PD8_IRQHandler             [WEAK] ; 134: PORTD 8
                EXPORT  PD9_IRQHandler             [WEAK] ; 135: PORTD 9
                EXPORT  PD10_IRQHandler            [WEAK] ; 136: PORTD 10
                EXPORT  PD11_IRQHandler            [WEAK] ; 137: PORTD 11
                EXPORT  PD12_IRQHandler            [WEAK] ; 138: PORTD 12
                EXPORT  PD13_IRQHandler            [WEAK] ; 139: PORTD 13
                EXPORT  PD14_IRQHandler            [WEAK] ; 140: PORTD 14
                EXPORT  PD15_IRQHandler            [WEAK] ; 141: PORTD 15
                EXPORT  PE0_IRQHandler             [WEAK] ; 142: PORTE 0
                EXPORT  PE1_IRQHandler             [WEAK] ; 143: PORTE 1
                EXPORT  PE2_IRQHandler             [WEAK] ; 144: PORTE 2
                EXPORT  PE3_IRQHandler             [WEAK] ; 145: PORTE 3
                EXPORT  PE4_IRQHandler             [WEAK] ; 146: PORTE 4
                EXPORT  PE5_IRQHandler             [WEAK] ; 147: PORTE 5
                EXPORT  PE6_IRQHandler             [WEAK] ; 148: PORTE 6
                EXPORT  PE7_IRQHandler             [WEAK] ; 149: PORTE 7
                EXPORT  PE8_IRQHandler             [WEAK] ; 150: PORTE 8
                EXPORT  PE9_IRQHandler             [WEAK] ; 151: PORTE 9
                EXPORT  PE10_IRQHandler            [WEAK] ; 152: PORTE 10
                EXPORT  PE11_IRQHandler            [WEAK] ; 153: PORTE 11
                EXPORT  PE12_IRQHandler            [WEAK] ; 154: PORTE 12
                EXPORT  PE13_IRQHandler            [WEAK] ; 155: PORTE 13
                EXPORT  PE14_IRQHandler            [WEAK] ; 156: PORTE 14
                EXPORT  PE15_IRQHandler            [WEAK] ; 157: PORTE 15
                EXPORT  PF0_IRQHandler             [WEAK] ; 158: PORTF 0
                EXPORT  PF1_IRQHandler             [WEAK] ; 159: PORTF 1
                EXPORT  PF2_IRQHandler             [WEAK] ; 160: PORTF 2
                EXPORT  PF3_IRQHandler             [WEAK] ; 161: PORTF 3
                EXPORT  PF4_IRQHandler             [WEAK] ; 162: PORTF 4
                EXPORT  PF5_IRQHandler             [WEAK] ; 163: PORTF 5
                EXPORT  PF6_IRQHandler             [WEAK] ; 164: PORTF 6
                EXPORT  PF7_IRQHandler             [WEAK] ; 165: PORTF 7
                EXPORT  PF8_IRQHandler             [WEAK] ; 166: PORTF 8
                EXPORT  PF9_IRQHandler             [WEAK] ; 167: PORTF 9
                EXPORT  PF10_IRQHandler            [WEAK] ; 168: PORTF 10
                EXPORT  PF11_IRQHandler            [WEAK] ; 169: PORTF 11
                EXPORT  PF12_IRQHandler            [WEAK] ; 170: PORTF 12
                EXPORT  PF13_IRQHandler            [WEAK] ; 171: PORTF 13
                EXPORT  PF14_IRQHandler            [WEAK] ; 172: PORTF 14
                EXPORT  PF15_IRQHandler            [WEAK] ; 173: PORTF 15
                EXPORT  DMA_Active_0_IRQHandler    [WEAK] ; 174: DMA Active 0
                EXPORT  DMA_Active_1_IRQHandler    [WEAK] ; 175: DMA Active 1
                EXPORT  DMA_Active_2_IRQHandler    [WEAK] ; 176: DMA Active 2
                EXPORT  DMA_Active_3_IRQHandler    [WEAK] ; 177: DMA Active 3
                EXPORT  DMA_Done_0_IRQHandler      [WEAK] ; 178: DMA Done 0
                EXPORT  DMA_Done_1_IRQHandler      [WEAK] ; 179: DMA Done 1
                EXPORT  DMA_Done_2_IRQHandler      [WEAK] ; 180: DMA Done 2
                EXPORT  DMA_Done_3_IRQHandler      [WEAK] ; 181: DMA Done 3
                EXPORT  I2C0_MS_RX_IRQHandler      [WEAK] ; 182: I2C0 Master RX
                EXPORT  I2C0_MS_TX_IRQHandler      [WEAK] ; 183: I2C0 Master TX
                EXPORT  I2C0_SL_RX_IRQHandler      [WEAK] ; 184: I2C0 Slave RX
                EXPORT  I2C0_SL_TX_IRQHandler      [WEAK] ; 185: I2C0 Slave TX
                EXPORT  I2C1_MS_RX_IRQHandler      [WEAK] ; 186: I2C1 Master RX
                EXPORT  I2C1_MS_TX_IRQHandler      [WEAK] ; 187: I2C1 Master TX
                EXPORT  I2C1_SL_RX_IRQHandler      [WEAK] ; 188: I2C1 Slave RX
                EXPORT  I2C1_SL_TX_IRQHandler      [WEAK] ; 189: I2C1 Slave TX
                EXPORT  I2C2_MS_RX_IRQHandler      [WEAK] ; 190: I2C2 Master RX
                EXPORT  I2C2_MS_TX_IRQHandler      [WEAK] ; 191: I2C2 Master TX
                EXPORT  I2C2_SL_RX_IRQHandler      [WEAK] ; 192: I2C2 Slave RX
                EXPORT  I2C2_SL_TX_IRQHandler      [WEAK] ; 193: I2C2 Slave TX
                EXPORT  FPU_IRQHandler             [WEAK] ; 194: FPU
                EXPORT  TXEV_IRQHandler            [WEAK] ; 195: TXEV


OC0_IRQHandler             ;  0: Always 0
OC1_IRQHandler             ;  1: Always 0
OC2_IRQHandler             ;  2: Always 0
OC3_IRQHandler             ;  3: Always 0
OC4_IRQHandler             ;  4: Always 0
OC5_IRQHandler             ;  5: Always 0
OC6_IRQHandler             ;  6: Always 0
OC7_IRQHandler             ;  7: Always 0
OC8_IRQHandler             ;  8: Always 0
OC9_IRQHandler             ;  9: Always 0
OC10_IRQHandler            ; 10: Always 0
OC11_IRQHandler            ; 11: Always 0
OC12_IRQHandler            ; 12: Always 0
OC13_IRQHandler            ; 13: Always 0
OC14_IRQHandler            ; 14: Always 0
OC15_IRQHandler            ; 15: Always 0
SPI0_TX_IRQHandler         ; 16: SPI0 TX
SPI0_RX_IRQHandler         ; 17: SPI0 RX
SPI1_TX_IRQHandler         ; 18: SPI1 TX
SPI1_RX_IRQHandler         ; 19: SPI1 RX
SPI2_TX_IRQHandler         ; 20: SPI2 TX
SPI2_RX_IRQHandler         ; 21: SPI2 RX
SPI3_TX_IRQHandler         ; 22: SPI3 TX
SPI3_RX_IRQHandler         ; 23: SPI3 RX
UART0_TX_IRQHandler        ; 24: UART0 TX
UART0_RX_IRQHandler        ; 25: UART0 RX
UART1_TX_IRQHandler        ; 26: UART1 TX
UART1_RX_IRQHandler        ; 27: UART1 RX
UART2_TX_IRQHandler        ; 28: UART2 TX
UART2_RX_IRQHandler        ; 29: UART2 RX
I2C0_MS_IRQHandler         ; 30: I2C0_MS
I2C0_SL_IRQHandler         ; 31: I2C0_SL
I2C1_MS_IRQHandler         ; 32: I2C1_MS
I2C1_SL_IRQHandler         ; 33: I2C1_SL
I2C2_MS_IRQHandler         ; 34: I2C2_MS
I2C2_SL_IRQHandler         ; 35: I2C2_SL
Ethernet_IRQHandler        ; 36: Ethernet TX
OC37_IRQHandler            ; 37: Always 0
SpW_IRQHandler             ; 38: Space Wire
OC39_IRQHandler            ; 39: Always 0
DAC0_IRQHandler            ; 40: DAC 0
DAC1_IRQHandler            ; 41: DAC 1
TRNG_IRQHandler            ; 42: Random Number Generator
DMA_Error_IRQHandler       ; 43: DMA error
ADC_IRQHandler             ; 44: ADC
LoCLK_IRQHandler           ; 45: LoCLK
LVD_IRQHandler             ; 46: LVD
WDT_IRQHandler             ; 47: Watchdog
TIM0_IRQHandler            ; 48: Timer 0
TIM1_IRQHandler            ; 49: Timer 1
TIM2_IRQHandler            ; 50: Timer 2
TIM3_IRQHandler            ; 51: Timer 3
TIM4_IRQHandler            ; 52: Timer 4
TIM5_IRQHandler            ; 53: Timer 5
TIM6_IRQHandler            ; 54: Timer 6
TIM7_IRQHandler            ; 55: Timer 7
TIM8_IRQHandler            ; 56: Timer 8
TIM9_IRQHandler            ; 57: Timer 9
TIM10_IRQHandler           ; 58: Timer 10
TIM11_IRQHandler           ; 59: Timer 11
TIM12_IRQHandler           ; 60: Timer 12
TIM13_IRQHandler           ; 61: Timer 13
TIM14_IRQHandler           ; 62: Timer 14
TIM15_IRQHandler           ; 63: Timer 15
TIM16_IRQHandler           ; 64: Timer 16
TIM17_IRQHandler           ; 65: Timer 17
TIM18_IRQHandler           ; 66: Timer 18
TIM19_IRQHandler           ; 67: Timer 19
TIM20_IRQHandler           ; 68: Timer 20
TIM21_IRQHandler           ; 69: Timer 21
TIM22_IRQHandler           ; 70: Timer 22
TIM23_IRQHandler           ; 71: Timer 23
CAN0_IRQHandler            ; 72: CAN 0
OC73_IRQHandler            ; 73: Always 0
CAN1_IRQHandler            ; 74: CAN 1
OC75_IRQHandler            ; 75: Always 0
EDAC_MBE_IRQHandler        ; 76: EDAC Multi Bit Error
EDAC_SBE_IRQHandler        ; 77: EDAC Single Bit Error
PA0_IRQHandler             ; 78: PORTA 0
PA1_IRQHandler             ; 79: PORTA 1
PA2_IRQHandler             ; 80: PORTA 2
PA3_IRQHandler             ; 81: PORTA 3
PA4_IRQHandler             ; 82: PORTA 4
PA5_IRQHandler             ; 83: PORTA 5
PA6_IRQHandler             ; 84: PORTA 6
PA7_IRQHandler             ; 85: PORTA 7
PA8_IRQHandler             ; 86: PORTA 8
PA9_IRQHandler             ; 87: PORTA 9
PA10_IRQHandler            ; 88: PORTA 10
PA11_IRQHandler            ; 89: PORTA 11
PA12_IRQHandler            ; 90: PORTA 12
PA13_IRQHandler            ; 91: PORTA 13
PA14_IRQHandler            ; 92: PORTA 14
PA15_IRQHandler            ; 93: PORTA 15
PB0_IRQHandler             ; 94: PORTB 0
PB1_IRQHandler             ; 95: PORTB 1
PB2_IRQHandler             ; 96: PORTB 2
PB3_IRQHandler             ; 97: PORTB 3
PB4_IRQHandler             ; 98: PORTB 4
PB5_IRQHandler             ; 99: PORTB 5
PB6_IRQHandler             ; 100: PORTB 6
PB7_IRQHandler             ; 101: PORTB 7
PB8_IRQHandler             ; 102: PORTB 8
PB9_IRQHandler             ; 103: PORTB 9
PB10_IRQHandler            ; 104: PORTB 10
PB11_IRQHandler            ; 105: PORTB 11
PB12_IRQHandler            ; 106: PORTB 12
PB13_IRQHandler            ; 107: PORTB 13
PB14_IRQHandler            ; 108: PORTB 14
PB15_IRQHandler            ; 109: PORTB 15
PC0_IRQHandler             ; 110: PORTC 0
PC1_IRQHandler             ; 111: PORTC 1
PC2_IRQHandler             ; 112: PORTC 2
PC3_IRQHandler             ; 113: PORTC 3
PC4_IRQHandler             ; 114: PORTC 4
PC5_IRQHandler             ; 115: PORTC 5
PC6_IRQHandler             ; 116: PORTC 6
PC7_IRQHandler             ; 117: PORTC 7
PC8_IRQHandler             ; 118: PORTC 8
PC9_IRQHandler             ; 119: PORTC 9
PC10_IRQHandler            ; 120: PORTC 10
PC11_IRQHandler            ; 121: PORTC 11
PC12_IRQHandler            ; 122: PORTC 12
PC13_IRQHandler            ; 123: PORTC 13
PC14_IRQHandler            ; 124: PORTC 14
PC15_IRQHandler            ; 125: PORTC 15
PD0_IRQHandler             ; 126: PORTD 0
PD1_IRQHandler             ; 127: PORTD 1
PD2_IRQHandler             ; 128: PORTD 2
PD3_IRQHandler             ; 129: PORTD 3
PD4_IRQHandler             ; 130: PORTD 4
PD5_IRQHandler             ; 131: PORTD 5
PD6_IRQHandler             ; 132: PORTD 6
PD7_IRQHandler             ; 133: PORTD 7
PD8_IRQHandler             ; 134: PORTD 8
PD9_IRQHandler             ; 135: PORTD 9
PD10_IRQHandler            ; 136: PORTD 10
PD11_IRQHandler            ; 137: PORTD 11
PD12_IRQHandler            ; 138: PORTD 12
PD13_IRQHandler            ; 139: PORTD 13
PD14_IRQHandler            ; 140: PORTD 14
PD15_IRQHandler            ; 141: PORTD 15
PE0_IRQHandler             ; 142: PORTE 0
PE1_IRQHandler             ; 143: PORTE 1
PE2_IRQHandler             ; 144: PORTE 2
PE3_IRQHandler             ; 145: PORTE 3
PE4_IRQHandler             ; 146: PORTE 4
PE5_IRQHandler             ; 147: PORTE 5
PE6_IRQHandler             ; 148: PORTE 6
PE7_IRQHandler             ; 149: PORTE 7
PE8_IRQHandler             ; 150: PORTE 8
PE9_IRQHandler             ; 151: PORTE 9
PE10_IRQHandler            ; 152: PORTE 10
PE11_IRQHandler            ; 153: PORTE 11
PE12_IRQHandler            ; 154: PORTE 12
PE13_IRQHandler            ; 155: PORTE 13
PE14_IRQHandler            ; 156: PORTE 14
PE15_IRQHandler            ; 157: PORTE 15
PF0_IRQHandler             ; 158: PORTF 0
PF1_IRQHandler             ; 159: PORTF 1
PF2_IRQHandler             ; 160: PORTF 2
PF3_IRQHandler             ; 161: PORTF 3
PF4_IRQHandler             ; 162: PORTF 4
PF5_IRQHandler             ; 163: PORTF 5
PF6_IRQHandler             ; 164: PORTF 6
PF7_IRQHandler             ; 165: PORTF 7
PF8_IRQHandler             ; 166: PORTF 8
PF9_IRQHandler             ; 167: PORTF 9
PF10_IRQHandler            ; 168: PORTF 10
PF11_IRQHandler            ; 169: PORTF 11
PF12_IRQHandler            ; 170: PORTF 12
PF13_IRQHandler            ; 171: PORTF 13
PF14_IRQHandler            ; 172: PORTF 14
PF15_IRQHandler            ; 173: PORTF 15
DMA_Active_0_IRQHandler    ; 174: DMA Active 0
DMA_Active_1_IRQHandler    ; 175: DMA Active 1
DMA_Active_2_IRQHandler    ; 176: DMA Active 2
DMA_Active_3_IRQHandler    ; 177: DMA Active 3
DMA_Done_0_IRQHandler      ; 178: DMA Done 0
DMA_Done_1_IRQHandler      ; 179: DMA Done 1
DMA_Done_2_IRQHandler      ; 180: DMA Done 2
DMA_Done_3_IRQHandler      ; 181: DMA Done 3
I2C0_MS_RX_IRQHandler      ; 182: I2C0 Master RX
I2C0_MS_TX_IRQHandler      ; 183: I2C0 Master TX
I2C0_SL_RX_IRQHandler      ; 184: I2C0 Slave RX
I2C0_SL_TX_IRQHandler      ; 185: I2C0 Slave TX
I2C1_MS_RX_IRQHandler      ; 186: I2C1 Master RX
I2C1_MS_TX_IRQHandler      ; 187: I2C1 Master TX
I2C1_SL_RX_IRQHandler      ; 188: I2C1 Slave RX
I2C1_SL_TX_IRQHandler      ; 189: I2C1 Slave TX
I2C2_MS_RX_IRQHandler      ; 190: I2C2 Master RX
I2C2_MS_TX_IRQHandler      ; 191: I2C2 Master TX
I2C2_SL_RX_IRQHandler      ; 192: I2C2 Slave RX
I2C2_SL_TX_IRQHandler      ; 193: I2C2 Slave TX
FPU_IRQHandler             ; 194: FPU
TXEV_IRQHandler            ; 195: TXEV
                B       .

                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF


                END
