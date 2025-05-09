# Create an INTERFACE library for our C module.
add_library(usermod_esp32can INTERFACE)

# Add our source files to the lib
target_sources(usermod_esp32can INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/mod_can.c
)

# Add the current directory as an include directory.
target_include_directories(usermod_esp32can INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_definitions(usermod_esp32can INTERFACE)

# Link our INTERFACE library to the usermod target.
target_link_libraries(usermod INTERFACE usermod_esp32can)