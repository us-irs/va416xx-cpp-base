add_subdirectory(start)
add_subdirectory(common)
add_subdirectory(utils)

if(VOR_CUSTOM_USER_DIR)
    add_subdirectory(ADD_CUSTOM_USER_DIR)
else()
    add_subdirectory(user)
endif()