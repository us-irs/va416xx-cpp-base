target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(${TARGET_NAME} PRIVATE
    va416xx_hal.c
    va416xx_hal_uart.c
    va416xx_hal_timer.c
    va416xx_hal_spi.c
    va416xx_hal_i2c.c
    va416xx_hal_ioconfig.c
)
