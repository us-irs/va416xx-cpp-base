/* File: startup_ARMCM4.S
 * Purpose: startup file for Cortex-M4 devices. Should use with
 *   GCC for ARM Embedded Processors
 * Version: V2.01
 * Date: 12 June 2014
 *
 */
/* Copyright (c) 2011 - 2014 ARM LIMITED
   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

    .thumb
	.syntax	unified
	.cpu cortex-m4
	.arch	armv7-m

	.section .vectors
	.align	2
	.globl	__Vectors
__Vectors:
	.long	_stack_start          /* Top of Stack */
	.long	Reset_Handler         /* Reset Handler */
	.long	NMI_Handler           /* NMI Handler */
	.long	HardFault_Handler     /* Hard Fault Handler */
	.long	MemManage_Handler     /* MPU Fault Handler */
	.long	BusFault_Handler      /* Bus Fault Handler */
	.long	UsageFault_Handler    /* Usage Fault Handler */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	SVC_Handler           /* SVCall Handler */
	.long	DebugMon_Handler      /* Debug Monitor Handler */
	.long	0                     /* Reserved */
	.long	PendSV_Handler        /* PendSV Handler */
	.long	SysTick_Handler       /* SysTick Handler */

	/* External interrupts */
	.long	OC0_IRQHandler        	//   0:  Always 0
	.long	OC1_IRQHandler          //   1:  Always 0
    .long   OC2_IRQHandler       	//   2:  Always 0
    .long   OC3_IRQHandler        	//   3:  Always 0
    .long   OC4_IRQHandler        	//   4:  Always 0
    .long   OC5_IRQHandler        	//   5:  Always 0
    .long   OC6_IRQHandler        	//   6:  Always 0
    .long   OC7_IRQHandler        	//   7:  Always 0
    .long   OC8_IRQHandler        	//   8:  Always 0
    .long   OC9_IRQHandler        	//   9:  Always 0
    .long   OC10_IRQHandler      	//  10:  Always 0
    .long   OC11_IRQHandler       	//  11:  Always 0
    .long   OC12_IRQHandler       	//  12:  Always 0
    .long   OC13_IRQHandler       	//  13:  Always 0
    .long   OC14_IRQHandler       	//  14:  Always 0
    .long   OC15_IRQHandler       	//  15:  Always 0
    .long   SPI0_TX_IRQHandler    	//  16
    .long   SPI0_RX_IRQHandler    	//  17
    .long   SPI1_TX_IRQHandler    	//  18
    .long   SPI1_RX_IRQHandler    	//  19
    .long   SPI2_TX_IRQHandler    	//  20
    .long   SPI2_RX_IRQHandler    	//  21
    .long   SPI3_TX_IRQHandler    	//  22
    .long   SPI3_RX_IRQHandler    	//  23
    .long   UART0_TX_IRQHandler  	//  24
    .long   UART0_RX_IRQHandler  	//  25
    .long   UART1_TX_IRQHandler  	//  26
    .long   UART1_RX_IRQHandler  	//  27
    .long   UART2_TX_IRQHandler  	//  28
    .long   UART2_RX_IRQHandler  	//  29
    .long   I2C0_MS_IRQHandler    	//  30
    .long   I2C0_SL_IRQHandler    	//  31
    .long   I2C1_MS_IRQHandler    	//  32
    .long   I2C1_SL_IRQHandler    	//  33
    .long   I2C2_MS_IRQHandler    	//  34
    .long   I2C2_SL_IRQHandler    	//  35
    .long   Ethernet_IRQHandler   	//  36
    .long   OC37_IRQHandler       	//  37
    .long   SpW_IRQHandler        	//  38
    .long   OC39_IRQHandler       	//  39
    .long   DAC0_IRQHandler       	//  40
    .long   DAC1_IRQHandler       	//  41
    .long   TRNG_IRQHandler      	//  42
    .long   DMA_Error_IRQHandler  	//  43
    .long   ADC_IRQHandler        	//  44
    .long   LoCLK_IRQHandler      	//  45
    .long   LVD_IRQHandler        	//  46
    .long   WDT_IRQHandler        	//  47
    .long   TIM0_IRQHandler       	//  48
    .long   TIM1_IRQHandler       	//  49
    .long   TIM2_IRQHandler       	//  50
    .long   TIM3_IRQHandler       	//  51
    .long   TIM4_IRQHandler       	//  52
    .long   TIM5_IRQHandler       	//  53
    .long   TIM6_IRQHandler       	//  54
    .long   TIM7_IRQHandler       	//  55
    .long   TIM8_IRQHandler       	//  56
    .long   TIM9_IRQHandler       	//  57
    .long   TIM10_IRQHandler      	//  58
    .long   TIM11_IRQHandler      	//  59
    .long   TIM12_IRQHandler      	//  60
    .long   TIM13_IRQHandler      	//  61
    .long   TIM14_IRQHandler      	//  62
    .long   TIM15_IRQHandler      	//  63
    .long   TIM16_IRQHandler      	//  64
    .long   TIM17_IRQHandler      	//  65
    .long   TIM18_IRQHandler      	//  66
    .long   TIM19_IRQHandler      	//  67
    .long   TIM20_IRQHandler      	//  68
    .long   TIM21_IRQHandler      	//  69
    .long   TIM22_IRQHandler      	//  70
    .long   TIM23_IRQHandler      	//  71
    .long   CAN0_IRQHandler       	//  72
    .long   OC73_IRQHandler       	//  73
    .long   CAN1_IRQHandler       	//  74
    .long   OC75_IRQHandler       	//  75
    .long   EDAC_MBE_IRQHandler   	//  76
    .long   EDAC_SBE_IRQHandler   	//  77
    .long   PA0_IRQHandler        	//  78
    .long   PA1_IRQHandler        	//  79
    .long   PA2_IRQHandler       	//  80
    .long   PA3_IRQHandler        	//  81
    .long   PA4_IRQHandler        	//  82
    .long   PA5_IRQHandler        	//  83
    .long   PA6_IRQHandler        	//  84
    .long   PA7_IRQHandler        	//  85
    .long   PA8_IRQHandler        	//  86
    .long   PA9_IRQHandler        	//  87
    .long   PA10_IRQHandler       	//  88
    .long   PA11_IRQHandler       	//  89
    .long   PA12_IRQHandler       	//  90
    .long   PA13_IRQHandler       	//  91
    .long   PA14_IRQHandler       	//  92
    .long   PA15_IRQHandler       	//  93
    .long   PB0_IRQHandler        	//  94
    .long   PB1_IRQHandler        	//  95
    .long   PB2_IRQHandler        	//  96
    .long   PB3_IRQHandler        	//  97
    .long   PB4_IRQHandler        	//  98
    .long   PB5_IRQHandler       	//  99
    .long   PB6_IRQHandler        	//  100
    .long   PB7_IRQHandler        	//  101
    .long   PB8_IRQHandler        	//  102
    .long   PB9_IRQHandler        	//  103
    .long   PB10_IRQHandler       	//  104
    .long   PB11_IRQHandler       	//  105
    .long   PB12_IRQHandler      	//  106
    .long   PB13_IRQHandler       	//  107
    .long   PB14_IRQHandler       	//  108
    .long   PB15_IRQHandler       	//  109
    .long   PC0_IRQHandler        	//  110
    .long   PC1_IRQHandler        	//  111
    .long   PC2_IRQHandler        	//  112
    .long   PC3_IRQHandler        	//  113
    .long   PC4_IRQHandler        	//  114
    .long   PC5_IRQHandler        	//  115
    .long   PC6_IRQHandler        	//  116
    .long   PC7_IRQHandler        	//  117
    .long   PC8_IRQHandler        	//  118
    .long   PC9_IRQHandler        	//  119
    .long   PC10_IRQHandler       	//  120
    .long   PC11_IRQHandler       	//  121
    .long   PC12_IRQHandler       	//  122
    .long   PC13_IRQHandler       	//  123
    .long   PC14_IRQHandler       	//  124
    .long   PC15_IRQHandler       	//  125
    .long   PD0_IRQHandler        	//  126
    .long   PD1_IRQHandler        	//  127
    .long   PD2_IRQHandler        	//  128
    .long   PD3_IRQHandler       	//  129
    .long   PD4_IRQHandler        	//  130
    .long   PD5_IRQHandler        	//  131
    .long   PD6_IRQHandler        	//  132
    .long   PD7_IRQHandler        	//  133
    .long   PD8_IRQHandler        	//  134
    .long   PD9_IRQHandler        	//  135
    .long   PD10_IRQHandler       	//  136
    .long   PD11_IRQHandler       	//  137
    .long   PD12_IRQHandler       	//  138
    .long   PD13_IRQHandler       	//  139
    .long   PD14_IRQHandler       	//  140
    .long   PD15_IRQHandler       	//  141
    .long   PE0_IRQHandler        	//  142
    .long   PE1_IRQHandler        	//  143
    .long   PE2_IRQHandler        	//  144
    .long   PE3_IRQHandler        	//  145
    .long   PE4_IRQHandler        	//  146
    .long   PE5_IRQHandler        	//  147
    .long   PE6_IRQHandler        	//  148
    .long   PE7_IRQHandler        	//  149
    .long   PE8_IRQHandler        	//  150
    .long   PE9_IRQHandler        	//  151
    .long   PE10_IRQHandler       	//  152
    .long   PE11_IRQHandler       	//  153
    .long   PE12_IRQHandler         //  154
    .long   PE13_IRQHandler         //  155
    .long   PE14_IRQHandler         //  156
    .long   PE15_IRQHandler         //  157
    .long   PF0_IRQHandler          //  158
    .long   PF1_IRQHandler          //  159
    .long   PF2_IRQHandler          //  160
    .long   PF3_IRQHandler          //  161
    .long   PF4_IRQHandler          //  162
    .long   PF5_IRQHandler          //  163
    .long   PF6_IRQHandler          //  164
    .long   PF7_IRQHandler          //  165
    .long   PF8_IRQHandler          //  166
    .long   PF9_IRQHandler          //  167
    .long   PF10_IRQHandler         //  168
    .long   PF11_IRQHandler         //  169
    .long   PF12_IRQHandler         //  170
    .long   PF13_IRQHandler         //  171
    .long   PF14_IRQHandler         //  172
    .long   PF15_IRQHandler         //  173
    .long   DMA_Active_0_IRQHandler //  174
    .long   DMA_Active_1_IRQHandler //  175
    .long   DMA_Active_2_IRQHandler //  176
    .long   DMA_Active_3_IRQHandler //  177
    .long   DMA_Done_0_IRQHandler   //  178
    .long   DMA_Done_1_IRQHandler   //  179
    .long   DMA_Done_2_IRQHandler   //  180
    .long   DMA_Done_3_IRQHandler   //  181
    .long   I2C0_MS_RX_IRQHandler   //  182
    .long   I2C0_MS_TX_IRQHandler   //  183
    .long   I2C0_SL_RX_IRQHandler   //  184
    .long   I2C0_SL_TX_IRQHandler   //  185
    .long   I2C1_MS_RX_IRQHandler   //  186
    .long   I2C1_MS_TX_IRQHandler   //  187
    .long   I2C1_SL_RX_IRQHandler   //  188
    .long   I2C1_SL_TX_IRQHandler   //  189
    .long   I2C2_MS_RX_IRQHandler   //  190
    .long   I2C2_MS_TX_IRQHandler   //  191
    .long   I2C2_SL_RX_IRQHandler   //  192
    .long   I2C2_SL_TX_IRQHandler   //  193
    .long   FPU_IRQHandler          //  194
    .long   TXEV_IRQHandler         //  195

	.size	__Vectors, . - __Vectors

	.text
	.thumb
	.thumb_func
	.align	2
	.globl	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:
/*  Firstly it copies data from read only memory to RAM. There are two schemes
 *  to copy. One can copy more than one sections. Another can only copy
 *  one section.  The former scheme needs more instructions and read-only
 *  data to implement than the latter.
 *  Macro __STARTUP_COPY_MULTIPLE is used to choose between two schemes.  */

#ifndef __NO_SYSTEM_INIT
	bl	SystemInit
#endif

    bl   main

	.pool
	.size	Reset_Handler, . - Reset_Handler

	.align	1
	.thumb_func
	.weak	Default_Handler
	.type	Default_Handler, %function
Default_Handler:
	b	.
	.size	Default_Handler, . - Default_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
	.macro	def_irq_handler	handler_name
	.weak	\handler_name
	.set	\handler_name, Default_Handler
	.endm

	def_irq_handler	NMI_Handler
	def_irq_handler	HardFault_Handler
	def_irq_handler	MemManage_Handler
	def_irq_handler	BusFault_Handler
	def_irq_handler	UsageFault_Handler
	def_irq_handler	SVC_Handler
	def_irq_handler	DebugMon_Handler
	def_irq_handler	PendSV_Handler
	def_irq_handler	SysTick_Handler

	def_irq_handler	OC0_IRQHandler
	def_irq_handler	OC1_IRQHandler
	def_irq_handler	OC2_IRQHandler
	def_irq_handler	OC3_IRQHandler
	def_irq_handler	OC4_IRQHandler
	def_irq_handler	OC5_IRQHandler
	def_irq_handler	OC6_IRQHandler
	def_irq_handler	OC7_IRQHandler
	def_irq_handler	OC8_IRQHandler
    def_irq_handler OC9_IRQHandler
    def_irq_handler OC10_IRQHandler
    def_irq_handler OC11_IRQHandler
    def_irq_handler OC12_IRQHandler
    def_irq_handler OC13_IRQHandler
    def_irq_handler OC14_IRQHandler
    def_irq_handler OC15_IRQHandler
    def_irq_handler SPI0_TX_IRQHandler
    def_irq_handler SPI0_RX_IRQHandler
    def_irq_handler SPI1_TX_IRQHandler
    def_irq_handler SPI1_RX_IRQHandler
    def_irq_handler SPI2_TX_IRQHandler
    def_irq_handler SPI2_RX_IRQHandler
    def_irq_handler SPI3_TX_IRQHandler
    def_irq_handler SPI3_RX_IRQHandler
    def_irq_handler UART0_TX_IRQHandler
    def_irq_handler UART0_RX_IRQHandler
    def_irq_handler UART1_TX_IRQHandler
    def_irq_handler UART1_RX_IRQHandler
    def_irq_handler UART2_TX_IRQHandler
    def_irq_handler UART2_RX_IRQHandler
    def_irq_handler I2C0_MS_IRQHandler
    def_irq_handler I2C0_SL_IRQHandler
    def_irq_handler I2C1_MS_IRQHandler
    def_irq_handler I2C1_SL_IRQHandler
    def_irq_handler I2C2_MS_IRQHandler
    def_irq_handler I2C2_SL_IRQHandler
    def_irq_handler Ethernet_IRQHandler
    def_irq_handler OC37_IRQHandler
    def_irq_handler SpW_IRQHandler
    def_irq_handler OC39_IRQHandler
    def_irq_handler DAC0_IRQHandler
    def_irq_handler DAC1_IRQHandler
    def_irq_handler TRNG_IRQHandler
    def_irq_handler DMA_Error_IRQHandler
    def_irq_handler ADC_IRQHandler
    def_irq_handler LoCLK_IRQHandler
    def_irq_handler LVD_IRQHandler
    def_irq_handler WDT_IRQHandler
    def_irq_handler TIM0_IRQHandler
    def_irq_handler TIM1_IRQHandler
    def_irq_handler TIM2_IRQHandler
    def_irq_handler TIM3_IRQHandler
    def_irq_handler TIM4_IRQHandler
    def_irq_handler TIM5_IRQHandler
    def_irq_handler TIM6_IRQHandler
    def_irq_handler TIM7_IRQHandler
    def_irq_handler TIM8_IRQHandler
    def_irq_handler TIM9_IRQHandler
    def_irq_handler TIM10_IRQHandler
    def_irq_handler TIM11_IRQHandler
    def_irq_handler TIM12_IRQHandler
    def_irq_handler TIM13_IRQHandler
    def_irq_handler TIM14_IRQHandler
    def_irq_handler TIM15_IRQHandler
    def_irq_handler TIM16_IRQHandler
    def_irq_handler TIM17_IRQHandler
    def_irq_handler TIM18_IRQHandler
    def_irq_handler TIM19_IRQHandler
    def_irq_handler TIM20_IRQHandler
    def_irq_handler TIM21_IRQHandler
    def_irq_handler TIM22_IRQHandler
    def_irq_handler TIM23_IRQHandler
    def_irq_handler CAN0_IRQHandler
    def_irq_handler OC73_IRQHandler
    def_irq_handler CAN1_IRQHandler
    def_irq_handler OC75_IRQHandler
    def_irq_handler EDAC_MBE_IRQHandler
    def_irq_handler EDAC_SBE_IRQHandler
    def_irq_handler PA0_IRQHandler
    def_irq_handler PA1_IRQHandler
    def_irq_handler PA2_IRQHandler
    def_irq_handler PA3_IRQHandler
    def_irq_handler PA4_IRQHandler
    def_irq_handler PA5_IRQHandler
    def_irq_handler PA6_IRQHandler
    def_irq_handler PA7_IRQHandler
    def_irq_handler PA8_IRQHandler
    def_irq_handler PA9_IRQHandler
    def_irq_handler PA10_IRQHandler
    def_irq_handler PA11_IRQHandler
    def_irq_handler PA12_IRQHandler
    def_irq_handler PA13_IRQHandler
    def_irq_handler PA14_IRQHandler
    def_irq_handler PA15_IRQHandler
    def_irq_handler PB0_IRQHandler
    def_irq_handler PB1_IRQHandler
    def_irq_handler PB2_IRQHandler
    def_irq_handler PB3_IRQHandler
    def_irq_handler PB4_IRQHandler
    def_irq_handler PB5_IRQHandler
    def_irq_handler PB6_IRQHandler
    def_irq_handler PB7_IRQHandler
    def_irq_handler PB8_IRQHandler
    def_irq_handler PB9_IRQHandler
    def_irq_handler PB10_IRQHandler
    def_irq_handler PB11_IRQHandler
    def_irq_handler PB12_IRQHandler
    def_irq_handler PB13_IRQHandler
    def_irq_handler PB14_IRQHandler
    def_irq_handler PB15_IRQHandler
    def_irq_handler PC0_IRQHandler
    def_irq_handler PC1_IRQHandler
    def_irq_handler PC2_IRQHandler
    def_irq_handler PC3_IRQHandler
    def_irq_handler PC4_IRQHandler
    def_irq_handler PC5_IRQHandler
    def_irq_handler PC6_IRQHandler
    def_irq_handler PC7_IRQHandler
    def_irq_handler PC8_IRQHandler
    def_irq_handler PC9_IRQHandler
    def_irq_handler PC10_IRQHandler
    def_irq_handler PC11_IRQHandler
    def_irq_handler PC12_IRQHandler
    def_irq_handler PC13_IRQHandler
    def_irq_handler PC14_IRQHandler
    def_irq_handler PC15_IRQHandler
    def_irq_handler PD0_IRQHandler
    def_irq_handler PD1_IRQHandler
    def_irq_handler PD2_IRQHandler
    def_irq_handler PD3_IRQHandler
    def_irq_handler PD4_IRQHandler
    def_irq_handler PD5_IRQHandler
    def_irq_handler PD6_IRQHandler
    def_irq_handler PD7_IRQHandler
    def_irq_handler PD8_IRQHandler
    def_irq_handler PD9_IRQHandler
    def_irq_handler PD10_IRQHandler
    def_irq_handler PD11_IRQHandler
    def_irq_handler PD12_IRQHandler
    def_irq_handler PD13_IRQHandler
    def_irq_handler PD14_IRQHandler
    def_irq_handler PD15_IRQHandler
    def_irq_handler PE0_IRQHandler
    def_irq_handler PE1_IRQHandler
    def_irq_handler PE2_IRQHandler
    def_irq_handler PE3_IRQHandler
    def_irq_handler PE4_IRQHandler
    def_irq_handler PE5_IRQHandler
    def_irq_handler PE6_IRQHandler
    def_irq_handler PE7_IRQHandler
    def_irq_handler PE8_IRQHandler
    def_irq_handler PE9_IRQHandler
    def_irq_handler PE10_IRQHandler
    def_irq_handler PE11_IRQHandler
    def_irq_handler PE12_IRQHandler
    def_irq_handler PE13_IRQHandler
    def_irq_handler PE14_IRQHandler
    def_irq_handler PE15_IRQHandler
    def_irq_handler PF0_IRQHandler
    def_irq_handler PF1_IRQHandler
    def_irq_handler PF2_IRQHandler
    def_irq_handler PF3_IRQHandler
    def_irq_handler PF4_IRQHandler
    def_irq_handler PF5_IRQHandler
    def_irq_handler PF6_IRQHandler
    def_irq_handler PF7_IRQHandler
    def_irq_handler PF8_IRQHandler
    def_irq_handler PF9_IRQHandler
    def_irq_handler PF10_IRQHandler
    def_irq_handler PF11_IRQHandler
    def_irq_handler PF12_IRQHandler
    def_irq_handler PF13_IRQHandler
    def_irq_handler PF14_IRQHandler
    def_irq_handler PF15_IRQHandler
    def_irq_handler DMA_Active_0_IRQHandler
    def_irq_handler DMA_Active_1_IRQHandler
    def_irq_handler DMA_Active_2_IRQHandler
    def_irq_handler DMA_Active_3_IRQHandler
    def_irq_handler DMA_Done_0_IRQHandler
    def_irq_handler DMA_Done_1_IRQHandler
    def_irq_handler DMA_Done_2_IRQHandler
    def_irq_handler DMA_Done_3_IRQHandler
    def_irq_handler I2C0_MS_RX_IRQHandler
    def_irq_handler I2C0_MS_TX_IRQHandler
    def_irq_handler I2C0_SL_RX_IRQHandler
    def_irq_handler I2C0_SL_TX_IRQHandler
    def_irq_handler I2C1_MS_RX_IRQHandler
    def_irq_handler I2C1_MS_TX_IRQHandler
    def_irq_handler I2C1_SL_RX_IRQHandler
    def_irq_handler I2C1_SL_TX_IRQHandler
    def_irq_handler I2C2_MS_RX_IRQHandler
    def_irq_handler I2C2_MS_TX_IRQHandler
    def_irq_handler I2C2_SL_RX_IRQHandler
    def_irq_handler I2C2_SL_TX_IRQHandler
    def_irq_handler FPU_IRQHandler
    def_irq_handler TXEV_IRQHandler

	.end
