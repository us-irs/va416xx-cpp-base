target_sources(${TARGET_NAME} PRIVATE
    event_handler_index.c
)

target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
