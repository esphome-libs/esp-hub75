# SPDX-FileCopyrightText: 2026 Stuart Parmenter
# SPDX-License-Identifier: MIT

# ESP-IDF project for color_test example

cmake_minimum_required(VERSION 3.16)

# Add hub75 component from repository root
set(EXTRA_COMPONENT_DIRS "../../../components")

# Copy shared ESP32-P4 defaults if building for ESP32-P4
# This allows all examples to share common P4-specific settings
set(SHARED_P4_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/../../common/sdkconfig.defaults.esp32p4")
set(LOCAL_P4_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults.esp32p4")
if(EXISTS ${SHARED_P4_DEFAULTS})
    configure_file(${SHARED_P4_DEFAULTS} ${LOCAL_P4_DEFAULTS} COPYONLY)
    message(STATUS "Copied shared ESP32-P4 defaults from examples/common/")
endif()

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define the project
project(color_test)
