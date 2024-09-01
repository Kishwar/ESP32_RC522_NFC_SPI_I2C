##
 # @file CMakeLists.txt
 #
 # @brief CMake file for project directory
 #
 # @author Kishwar Kumar
 # Contact: kumar.kishwar@gmail.com
 # 
 # @date 01/09/2024
 #
 ##

 cmake_minimum_required(VERSION 3.16.0)

 list(APPEND EXTRA_COMPONENT_DIRS "src"
                                  "lib/_core"
                                  "lib/_i2c"
                                  "lib/_spi")
                                 
 add_compile_options(-Wno-missing-field-initializers)
 
 include($ENV{IDF_PATH}/tools/cmake/project.cmake)
 
 project(ESP32_RC522_NFC_SPI_I2C)
