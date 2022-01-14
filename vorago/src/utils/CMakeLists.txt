target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(${TARGET_NAME} PRIVATE
    circular_buffer.c
    segger_rtt.c
    segger_rtt_printf.c
    utils.c
    segger_rtt_syscalls_gcc.c
)
