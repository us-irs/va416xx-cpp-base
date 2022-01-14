// It is assumed that no interrupts are used for a basic application.
// The user can call the default interrupt handlers written by Vorago here or
// call a customized one.

// The interrupts listed here are the ones where Vorago has provided an implementation

extern "C" {

void I2C0_SL_IRQHandler(void) {}
void I2C0_SL_RX_IRQHandler(void) {}
void I2C0_SL_TX_IRQHandler(void) {}
void I2C1_SL_IRQHandler(void) {}
void I2C1_SL_RX_IRQHandler(void) {}
void I2C1_SL_TX_IRQHandler(void) {}

void UART0_TX_IRQHandler(void) {}
void UART0_RX_IRQHandler(void) {}
void UART1_TX_IRQHandler(void) {}
void UART1_RX_IRQHandler(void) {}
void UART2_TX_IRQHandler(void) {}
void UART2_RX_IRQHandler(void) {}

void SPI0_RX_IRQHandler(void) {}
void SPI1_RX_IRQHandler(void) {}
void SPI2_RX_IRQHandler(void) {}
void SPI3_RX_IRQHandler(void) {}

}
