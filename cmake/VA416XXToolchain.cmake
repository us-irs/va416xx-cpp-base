# Cross compile CMake file for the SOURCE project. Requires ARM cross compiler in path

if(NOT DEFINED ENV{CROSS_COMPILE})
    set(CROSS_COMPILE "arm-none-eabi")
    message(STATUS 
        "No CROSS_COMPILE environmental variable set, using default ARM linux "
        "cross compiler name ${CROSS_COMPILE}"
    )
else()
    set(CROSS_COMPILE "$ENV{CROSS_COMPILE}")
    message(STATUS 
        "Using environmental variable CROSS_COMPILE as cross-compiler: "
        "$ENV{CROSS_COMPILE}"
    )
endif()

set(CROSS_COMPILE_CC "${CROSS_COMPILE}-gcc")
set(CROSS_COMPILE_CXX "${CROSS_COMPILE}-g++")
set(CROSS_COMPILE_OBJCOPY "${CROSS_COMPILE}-objcopy")
set(CROSS_COMPILE_SIZE "${CROSS_COMPILE}-size")

# At the very least, cross compile gcc, g++ and objcopy have to be set!
find_program (CROSS_COMPILE_CC_FOUND ${CROSS_COMPILE_CC} REQUIRED)
find_program (CROSS_COMPILE_CXX_FOUND ${CROSS_COMPILE_CXX} REQUIRED)
find_program (CROSS_COMPILE_OBJCOPY_FOUND ${CROSS_COMPILE_OBJCOPY} REQUIRED)

# Size utility optional.
find_program (CMAKE_SIZE ${CROSS_COMPILE_SIZE})

set(CMAKE_CROSSCOMPILING TRUE)

# Define name of the target system
set(CMAKE_SYSTEM_NAME "Generic")
set(CMAKE_SYSTEM_PROCESSOR "arm")

# Define the compiler
set(CMAKE_C_COMPILER ${CROSS_COMPILE_CC})
set(CMAKE_CXX_COMPILER ${CROSS_COMPILE_CXX})
set(CMAKE_ASM_FLAGS "-x assembler-with-cpp")

set(ABI_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfloat-abi=hard
    -specs=nosys.specs
)

if(GCC_USE_NANO_LIB)
    list(APPEND ABI_FLAGS
        -specs=nano.specs
    )
    if(GCC_NANOLIB_SCAN_FLOAT)
        list(APPEND ABI_FLAGS
            -Wl,--undefined,_scanf_float
        )
    endif()
    if(GCC_NANOLIB_SCAN_FLOAT)
        list(APPEND ABI_FLAGS
            -Wl,--undefined,_printf_float
        )
    endif()
endif()

string (REPLACE ";" " " ABI_FLAGS "${ABI_FLAGS}")

set(CMAKE_C_FLAGS 
    ${ABI_FLAGS}
    CACHE STRING "C flags for the VA416XX"
)
set(CMAKE_CXX_FLAGS 
    "${CMAKE_C_FLAGS}" 
    CACHE STRING "C++ flags for the VA416XX"
)