/**
 * Based on assembler implementation for Keil
 * 1. Set R0 to VTOR address
 * 2. Load VTOR
 * 3. Load initial MSP value
 * 4. Set SP value (assume MSP is selected)
 * 5. Load reset vector
 * 6. Branch to reset handler
 */
void vector_reset(void) __attribute__((naked));
void vector_reset(void) {
  __asm volatile(
      " ldr R0,=0xE000ED08                                        \n"
      " ldr r1, [r0]                                              \n"
      " ldr r0, [r1]                                              \n"
      " mov sp, r0                                                \n"
      " ldr r0, [r1, #4]                                          \n"
      " bx r0                                                     \n");
}
