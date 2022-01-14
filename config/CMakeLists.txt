target_sources(${TARGET_NAME} PRIVATE
    overrideWeak.cpp
    event_handler_index.h
)

target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
