# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(SRCS ${app_sources}
                       REQUIRES SX1262_Interface i2c_manager oled_display web_updater nvs_flash persistent_params logger web_terminal)
