target_sources(${TARGET_NAME} PRIVATE
    irq.cpp
    fault.cpp
    event_handler_index.h
)

target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
