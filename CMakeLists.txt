cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(ARDUINO_SELECTED_VARIANT "esp32s3" CACHE STRING "Force Arduino variant to ESP32-S3")

project(D-BOT)
