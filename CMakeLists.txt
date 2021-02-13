# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(firmware)

idf_component_register(SRCS "gatts_table_creat_demo.c"
                    INCLUDE_DIRS ".")