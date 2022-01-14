target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(eth)
add_subdirectory(spw)

target_sources(${TARGET_NAME} PRIVATE
    va416xx_hal.c
    va416xx_debug.c
    va416xx_hal_uart.c
    va416xx_hal_timer.c
    va416xx_hal_spi.c
    va416xx_hal_i2c.c
    va416xx_hal_ioconfig.c
    va416xx_hal_irqrouter.c
    va416xx_hal_ethernet.c
    va416xx_hal_dma.c
    va416xx_hal_dac.c
    va416xx_hal_adc.c
    va416xx_hal_clkgen.c
    va416xx_hal_canbus.c
    va416xx_hal_adc_swcal.c
)
