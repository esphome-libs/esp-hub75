# ESP32 HUB75 DMA Driver
# CMakeLists.txt for ESP-IDF component

# Build REQUIRES list conditionally (esp_mm only exists in ESP-IDF 5.x for P4/C6)
# Use IDF_TARGET (CMake variable) not CONFIG_IDF_TARGET_* (Kconfig, not yet available)
set(HUB75_REQUIRES
    driver
    esp_timer
    esp_hw_support
    freertos
)
if(IDF_TARGET STREQUAL "esp32p4" OR IDF_TARGET STREQUAL "esp32c6")
    list(APPEND HUB75_REQUIRES esp_mm)
endif()

idf_component_register(
    SRCS
        "src/core/hub75_driver.cpp"
        "src/color/color_lut.cpp"
        "src/color/color_convert.cpp"
        "src/platforms/platform_detect.cpp"
        "src/platforms/platform_dma.cpp"
        "src/drivers/fm6126a.cpp"
        # Platform-specific sources added conditionally below
    INCLUDE_DIRS
        "include"
        "src"
    REQUIRES
        ${HUB75_REQUIRES}
)

# Platform-specific source files
if(CONFIG_IDF_TARGET_ESP32 OR CONFIG_IDF_TARGET_ESP32S2)
    target_sources(${COMPONENT_LIB} PRIVATE
        "src/platforms/i2s/i2s_dma.cpp"
    )
elseif(CONFIG_IDF_TARGET_ESP32S3)
    target_sources(${COMPONENT_LIB} PRIVATE
        "src/platforms/gdma/gdma_dma.cpp"
    )
elseif(CONFIG_IDF_TARGET_ESP32P4 OR CONFIG_IDF_TARGET_ESP32C6)
    # PARLIO peripheral (ESP32-P4 has clock gating, ESP32-C6 does not)
    target_sources(${COMPONENT_LIB} PRIVATE
        "src/platforms/parlio/parlio_dma.cpp"
    )
endif()

# Compiler flags (common to both GCC and Clang)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter

    # Treat specific warnings as errors (compile-time safety checks)
    -Werror=return-type           # Ensure all functions return values
    -Werror=switch                # Catch missing enum cases in switches
    -Werror=implicit-fallthrough  # Require explicit [[fallthrough]] in switches
    -Werror=null-dereference      # Catch obvious null pointer dereferences
    -Werror=overflow              # Catch compile-time detectable overflows
    -Werror=unused-variable       # Catch unused local variables (almost always bugs)
    -Werror=deprecated-declarations  # Prevent use of deprecated ESP-IDF APIs
)

# GCC-specific flags (ESP-IDF toolchain uses GCC)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${COMPONENT_LIB} PRIVATE
        -Werror=maybe-uninitialized      # GCC: catch potentially uninitialized variables
        -Werror=unused-but-set-variable  # GCC: catch variables assigned but never read
        -Werror=format=2                 # Strict format string checking (ESP_LOG* macros)
    )
endif()

# Clang-specific flags (for local development/IDE support)
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    target_compile_options(${COMPONENT_LIB} PRIVATE
        -Werror=uninitialized            # Clang: catch uninitialized variables (different from GCC)
        -Werror=format=2                 # Clang: strict format string checking
    )
endif()

# Enable debug logging for the library
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    LOG_LOCAL_LEVEL=ESP_LOG_DEBUG
)

# Require C++20 for [[likely]]/[[unlikely]] and other modern features
# Note: consteval validation is skipped on ESP-IDF 4.x (GCC 8.x) via version guards
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_20)
