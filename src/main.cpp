#include <cstddef>
#include <cstdio>

#include "VORConfig.h"
#include "va416xx.h"
#include "va416xx_debug.h"
#include "va416xx_hal.h"
#include "va416xx_hal_gpio.h"

int main(void) {
  DBG_SetStdioOutput(en_stdio_rtt);
  SystemCoreClockUpdate();
  VOR_printf("-- Vorago VA416xx GCC demo project --\n");
  hal_status_t result = HAL_Init();
  if (result != hal_status_ok) {
    VOR_printf("HAL init failed with code %d", result);
  }
  VOR_printf("Toggling LED PG5 periodically\n");
  for (;;) {
    GPIO_TOG(VOR_PORTG, 5);
    for (size_t idx = 0; idx < 5000000; idx++) {
    }
  }
}
