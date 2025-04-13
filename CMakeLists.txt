# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mpu-2-mqtt)

target_add_binary_data(mpu-2-mqtt.elf "main/mqtt_eclipseprojects_io.pem" TEXT)
