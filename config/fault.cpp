#include <hardFault_handler.h>

extern "C" {

void HardFault_Handler() __attribute__((naked));
void HardFault_Handler() {
  VOREXC_HardFault_Handler();
}

}
