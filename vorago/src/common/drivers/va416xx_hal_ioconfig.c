/***************************************************************************************
 * @file     va416xx_hal_ioconfig.c
 * @version  V0.4
 * @date     30 January 2019
 *
 * @note
 * VORAGO Technologies
 *
 * @note
 * Copyright (c) 2013-2019 VORAGO Technologies.
 *
 * @par
 * BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND BY
 * ALL THE TERMS AND CONDITIONS OF THE VORAGO TECHNOLOGIES END USER LICENSE AGREEMENT.
 * THIS SOFTWARE IS PROVIDED "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. VORAGO TECHNOLOGIES
 * SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ****************************************************************************************/
 
/*****************************************************************************/ 
/* Include files                                                             */ 
/*****************************************************************************/

#include "va416xx_hal_ioconfig.h"
#include "board.h"
#include "va416xx_debug.h"

/*****************************************************************************/ 
/* Local pre-processor symbols/macros ('#define')                            */ 
/*****************************************************************************/

#define IOCONFIG_PERID (0x028207E9)

#ifndef DEFAULT_PIN_IOCFG
// usually defined in project's board.h
// if not defined, create a default
#define DEFAULT_PIN_IOCFG (IOCFG_REG_PULLDN)
#endif

#ifndef DEFAULT_PIN_DIR 
// usually defined in project's board.h
// if not defined, create a default
#define DEFAULT_PIN_DIR   (en_iocfg_dir_input)
#endif

/*****************************************************************************/ 
/* Global variable definitions (declared in header file with 'extern')       */ 
/*****************************************************************************/

/** Alias for GPIO Ports. */
VOR_GPIO_Type* const PORTA = &(VOR_GPIO->BANK[0]);
VOR_GPIO_Type* const PORTB = &(VOR_GPIO->BANK[1]);
VOR_GPIO_Type* const PORTC = &(VOR_GPIO->BANK[2]);
VOR_GPIO_Type* const PORTD = &(VOR_GPIO->BANK[3]);
VOR_GPIO_Type* const PORTE = &(VOR_GPIO->BANK[4]);
VOR_GPIO_Type* const PORTF = &(VOR_GPIO->BANK[5]);
VOR_GPIO_Type* const PORTG = &(VOR_GPIO->BANK[6]);

/*****************************************************************************/ 
/* Local type definitions ('typedef')                                        */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Local variable definitions ('static')                                     */ 
/*****************************************************************************/

/*****************************************************************************/ 
/* Local function prototypes ('static')                                      */ 
/*****************************************************************************/

static void ConfigureIODefault(void);

/*****************************************************************************/ 
/* Function implementation - global ('extern') and local ('static')          */ 
/*****************************************************************************/

/*******************************************************************************
 **
 ** @brief  Sets the IOCONFIG peripheral to the desired default value (not HW default)
 **
 ** @return void
 **
 ******************************************************************************/
static void ConfigureIODefault(void)
{
  uint32_t pin;

  // Set all pins to default state (user configured default, not POR default)
  for(pin = 0; pin < PORTA_F_NUM_PINS; pin++){
    HAL_Iocfg_SetupPin(VOR_PORTA, pin, DEFAULT_PIN_DIR, DEFAULT_PIN_IOCFG);
    HAL_Iocfg_SetupPin(VOR_PORTB, pin, DEFAULT_PIN_DIR, DEFAULT_PIN_IOCFG);
    HAL_Iocfg_SetupPin(VOR_PORTC, pin, DEFAULT_PIN_DIR, DEFAULT_PIN_IOCFG);
    HAL_Iocfg_SetupPin(VOR_PORTD, pin, DEFAULT_PIN_DIR, DEFAULT_PIN_IOCFG);
    HAL_Iocfg_SetupPin(VOR_PORTE, pin, DEFAULT_PIN_DIR, DEFAULT_PIN_IOCFG);
    HAL_Iocfg_SetupPin(VOR_PORTF, pin, DEFAULT_PIN_DIR, DEFAULT_PIN_IOCFG);
    if(pin < PORTG_NUM_PINS){
      HAL_Iocfg_SetupPin(VOR_PORTG, pin, DEFAULT_PIN_DIR, DEFAULT_PIN_IOCFG);
    }
  }
}

/*******************************************************************************
 **
 ** @brief  Initializes the IOCONFIG peripheral
 ** 
 ** @param  pinConfig - pointer to an array of stc_iocfg_pin_cfg_t structures (pin configs)
 **
 ** @return hal_status_t Status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Iocfg_Init(const stc_iocfg_pin_cfg_t* pinConfig)
{
  COMPILE_TIME_ASSERT(sizeof(stc_iocfg_reg_t) == 4);
  COMPILE_TIME_ASSERT(sizeof(un_iocfg_reg_t) == 4);
	
  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |= 
        (CLK_ENABLE_PORTA | CLK_ENABLE_PORTB | CLK_ENABLE_PORTC |
        CLK_ENABLE_PORTD | CLK_ENABLE_PORTE | CLK_ENABLE_PORTF |
        CLK_ENABLE_PORTG | CLK_ENABLE_IOCONFIG);

  // check peripheral ID
  if(VOR_IOCONFIG->PERID != IOCONFIG_PERID) { return hal_status_badPeriphID; }

  ConfigureIODefault();

  return HAL_Iocfg_SetupPins(pinConfig);
}

/*******************************************************************************
 **
 ** @brief  Initializes pins in pinConfig list
 ** 
 ** @param  pinConfig - pointer to an array of stc_iocfg_pin_cfg_t structures (pin configs)
 **
 ** @return hal_status_t Status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Iocfg_SetupPins(const stc_iocfg_pin_cfg_t* pinConfig)
{
  bool error = false;
  while(0 != pinConfig->port){
    if(hal_status_ok != HAL_Iocfg_SetupPin(pinConfig->port, pinConfig->pinNum, pinConfig->dir, pinConfig->reg)){
      // there was a problem
      error = true;
    }
    pinConfig++;
  }
  if(error){ 
    return hal_status_badParam;
  }
  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Resets and declocks the IOCONFIG peripheral
 **
 ** @return hal_status_t Status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Iocfg_DeInit(void)
{
  // check for IOCONFIG clock enable
  if (0 == (VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE & CLK_ENABLE_IOCONFIG)){ 
    return hal_status_notInitialized;
  }

  // reset IOCONFIG peripheral
  VOR_SYSCONFIG->PERIPHERAL_RESET &= ~SYSCONFIG_PERIPHERAL_RESET_IOCONFIG_Msk;
  __NOP();
  __NOP();
  VOR_SYSCONFIG->PERIPHERAL_RESET |= SYSCONFIG_PERIPHERAL_RESET_IOCONFIG_Msk;

  // declock IOCONFIG
  IOCFG_DISABLE_CLOCK();

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  Sets up a single pin, sets DIR bit (input/output) and its IOCONFIG register
 ** 
 ** @param  port - pointer to the GPIO peripheral port (PORTA, PORTB, etc)
 ** 
 ** @param  pin  - pin number (0-15 for PORTA-F, 0-7 for PORTG)
 ** 
 ** @param  dir  - enum, en_iocfg_dir_input or en_iocfg_dir_output
 ** 
 ** @param  reg  - un_iocfg_reg_t type - desired IOCONFIG register value for this pin
 **
 ** @return hal_status_t - Status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Iocfg_SetupPin(VOR_GPIO_Type* port, gpio_pin_number_t pin, en_iocfg_dir_t dir, un_iocfg_reg_t reg)
{
  uint32_t bank = ((uint32_t)port - (uint32_t)VOR_PORTA) / sizeof(VOR_GPIO_Type);

  // check for valid pin number
  if(pin >= PORTA_F_NUM_PINS){ 
    return hal_status_badParam;
  }
  if((pin >= PORTG_NUM_PINS) && (port == VOR_PORTG)){
    return hal_status_badParam;
  }

  // check for IOCONFIG clock enable
  if (0 == (VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE & CLK_ENABLE_IOCONFIG)){
    return hal_status_notInitialized;
  }

  // set direction (input/output, dontdare = input)
  port->DIR &= ~(1UL<<pin);
  if(en_iocfg_dir_output == dir){
    port->DIR |= 1UL<<pin;
  }

  // set IOCONFIG reg
  switch(bank)
  {
    case 0:
      VOR_IOCONFIG->PORTA[pin] = reg.regRaw;
      break;
    
    case 1:
      VOR_IOCONFIG->PORTB[pin] = reg.regRaw;
      break;
    
    case 2:
      VOR_IOCONFIG->PORTC[pin] = reg.regRaw;
      break;
    
    case 3:
      VOR_IOCONFIG->PORTD[pin] = reg.regRaw;
      break;
    
    case 4:
      VOR_IOCONFIG->PORTE[pin] = reg.regRaw;
      break;
    
    case 5:
      VOR_IOCONFIG->PORTF[pin] = reg.regRaw;
      break;
    
    case 6:
      VOR_IOCONFIG->PORTG[pin] = reg.regRaw;
      break;
    
    default:
      // not a valid bank
      return hal_status_badParam;
  }

  return hal_status_ok;
}

/*******************************************************************************
 **
 ** @brief  This driver contains a "pinmux" function for backwards compatibility
 ** 
 ** @param  port   - pointer to the GPIO peripheral port (PORTA, PORTB, etc)
 ** 
 ** @param  pin    - pin number (0-15 for PORTA-F, 0-7 for PORTG)
 ** 
 ** @param  funsel - function select 0-3
 **
 ** @return hal_status_t - Status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Iocfg_PinMux(VOR_GPIO_Type *port, gpio_pin_number_t pin, uint32_t funsel)
{
  uint32_t bank = ((uint32_t)port - (uint32_t)VOR_PORTA) / sizeof(VOR_GPIO_Type);

  // check for valid pin number
  if(pin >= 16){ 
    return hal_status_badParam;
  }
  if((pin >= 8) && (port == VOR_PORTG)){ 
    return hal_status_badParam;
  }

  // check for valid funsel
  if(funsel > FUNSEL3){ 
    return hal_status_badParam;
  }

  // check for IOCONFIG clock enable
  if (0 == (VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE & CLK_ENABLE_IOCONFIG)) { return hal_status_notInitialized; }

  switch(bank)
  {
    case 0:
      VOR_IOCONFIG->PORTA[pin] = (VOR_IOCONFIG->PORTA[pin] & ~((0x3)<<IOCONFIG_PORTA_FUNSEL_Pos))	| ((funsel)<<IOCONFIG_PORTA_FUNSEL_Pos);
      break;
    
    case 1:
      VOR_IOCONFIG->PORTB[pin] = (VOR_IOCONFIG->PORTB[pin] & ~((0x3)<<IOCONFIG_PORTB_FUNSEL_Pos))	| ((funsel)<<IOCONFIG_PORTB_FUNSEL_Pos);
      break;
    
    case 2:
      VOR_IOCONFIG->PORTC[pin] = (VOR_IOCONFIG->PORTC[pin] & ~((0x3)<<IOCONFIG_PORTC_FUNSEL_Pos))	| ((funsel)<<IOCONFIG_PORTC_FUNSEL_Pos);
      break;
    
    case 3:
      VOR_IOCONFIG->PORTD[pin] = (VOR_IOCONFIG->PORTD[pin] & ~((0x3)<<IOCONFIG_PORTD_FUNSEL_Pos))	| ((funsel)<<IOCONFIG_PORTD_FUNSEL_Pos);
      break;
    
    case 4:
      VOR_IOCONFIG->PORTE[pin] = (VOR_IOCONFIG->PORTE[pin] & ~((0x3)<<IOCONFIG_PORTE_FUNSEL_Pos))	| ((funsel)<<IOCONFIG_PORTE_FUNSEL_Pos);
      break;
    
    case 5:
      VOR_IOCONFIG->PORTF[pin] = (VOR_IOCONFIG->PORTF[pin] & ~((0x3)<<IOCONFIG_PORTF_FUNSEL_Pos))	| ((funsel)<<IOCONFIG_PORTF_FUNSEL_Pos);
      break;
    
    case 6:
      VOR_IOCONFIG->PORTG[pin] = (VOR_IOCONFIG->PORTG[pin] & ~((0x3)<<IOCONFIG_PORTG_FUNSEL_Pos))	| ((funsel)<<IOCONFIG_PORTG_FUNSEL_Pos);
      break;
    
    default:
      // not a valid bank
      return hal_status_badParam;
  }

  return hal_status_ok;
}


/*******************************************************************************
 **
 ** @brief  Set clock divider
 **
 ** @return hal_status_t Status of driver call
 **
 ******************************************************************************/
hal_status_t HAL_Iocfg_SetClkDiv (uint32_t clkNum, uint32_t divVal)
{
  // check for IOCONFIG clock enable
  if (0 == (VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE & CLK_ENABLE_IOCONFIG)){ 
    return hal_status_notInitialized; 
  }

  // check for valid divide value
  if(divVal > 255){ 
    // only 0-255 valid (8-bit)
    return hal_status_badParam;
  }

  switch(clkNum)
  {
    case 0:
      // CLKDIV0 is read only (0x1)
      //VOR_IOCONFIG->CLKDIV0 = divVal;
      return hal_status_badParam; 
      
    case 1:
      VOR_IOCONFIG->CLKDIV1 = divVal;
      break;
    
    case 2:
      VOR_IOCONFIG->CLKDIV2 = divVal;
      break;
    
    case 3:
      VOR_IOCONFIG->CLKDIV3 = divVal;
      break;
    
    case 4:
      VOR_IOCONFIG->CLKDIV4 = divVal;
      break;
    
    case 5:
      VOR_IOCONFIG->CLKDIV5 = divVal;
      break;
    
    case 6:
      VOR_IOCONFIG->CLKDIV6 = divVal;
      break;
    
    case 7:
      VOR_IOCONFIG->CLKDIV7 = divVal;
      break;
    
    default:
      return hal_status_badParam;
  }

  return hal_status_ok;
}

/*****************************************************************************/ 
/* End of file                                                               */ 
/*****************************************************************************/
