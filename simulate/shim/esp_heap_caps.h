#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#define MALLOC_CAP_SPIRAM   (1 << 10)
#define MALLOC_CAP_8BIT     (1 << 2)
#define MALLOC_CAP_DMA      (1 << 3)
#define MALLOC_CAP_INTERNAL (1 << 11)

#define heap_caps_malloc(size, caps)        malloc(size)
#define heap_caps_calloc(count, size, caps) calloc(count, size)
#define heap_caps_free(pointer)             free(pointer)
#define heap_caps_get_free_size(caps)       ((size_t)8 * 1024 * 1024)
