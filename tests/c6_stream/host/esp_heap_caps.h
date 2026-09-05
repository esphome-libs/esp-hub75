#pragma once
#include <cstddef>
constexpr unsigned MALLOC_CAP_INTERNAL = 1;
constexpr unsigned MALLOC_CAP_DMA = 2;
constexpr unsigned MALLOC_CAP_8BIT = 4;
void *heap_caps_calloc(size_t count, size_t bytes, unsigned capabilities);
void *heap_caps_aligned_calloc(size_t alignment, size_t count, size_t bytes, unsigned capabilities);
void heap_caps_free(void *pointer);
