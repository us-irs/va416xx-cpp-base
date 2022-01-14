#include "va416xx_hal_i2c.h"

extern "C" void SPI0_RX_IRQHandler(void) {}

extern "C" void SPI1_RX_IRQHandler(void) {}

extern "C" void SPI2_RX_IRQHandler(void) {}
extern "C" void SPI3_RX_IRQHandler(void) {}

extern "C" void I2C0_SL_RX_IRQHandler(void) {
  //VOR_I2C0_SL_RX_IRQHandler();
}

