target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(${TARGET_NAME} PRIVATE
    circular_buffer.c
    segger_rtt.c
    segger_rtt_printf.c
    hardFault_handler.c
    utils.c
    spi_fram.c
    vectrst.c
)

if(GCC_USE_SEGGER_SYSCALLS)
    target_sources(${TARGET_NAME} PRIVATE
        segger_rtt_syscalls_gcc.c
    )
endif()