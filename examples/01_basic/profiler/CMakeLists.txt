# SPDX-FileCopyrightText: 2026 Stuart Parmenter
# SPDX-License-Identifier: MIT

# ESP-IDF project for drawing API profiler example
# NOTE: This example uses the Xtensa perfmon API and only works on
# Xtensa-based chips (ESP32, ESP32-S2, ESP32-S3).
# It will NOT work on RISC-V chips (ESP32-C3, ESP32-C6, ESP32-P4).

cmake_minimum_required(VERSION 3.16)

# Add hub75 component from repository root
set(EXTRA_COMPONENT_DIRS "../../../components")

# Enable drawing profiling in hub75 component (adds CCOUNT timing instrumentation)
add_compile_definitions(HUB75_PROFILE_DRAWING)

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define the project
project(profiler)
