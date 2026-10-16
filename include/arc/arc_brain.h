#pragma once

#include "arc_types.h"
#include "arc_plasticity.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ARC001 (v1): frozen, 64-byte header. */
#define ARC_MAGIC        "ARC001"
#define ARC_VERSION      1
#define ARC_HEADER_SIZE  64

/* ARC002 (v2): plastic prior, 128-byte header. */
#define ARC_MAGIC_V2        "ARC002"
#define ARC_VERSION_V2      2
#define ARC_HEADER_SIZE_V2  128

/* Returns 2, 1, or 0 (unknown). */
int arc_brain_version(const uint8_t* data, size_t size);

arc_status arc_brain_validate_header(const uint8_t* header, size_t header_size,
                                      arc_geometry* geom_out, size_t* body_size_out);

arc_status arc_brain_get_plasticity(const uint8_t* header, size_t header_size,
                                     arc_plasticity_config* out);

arc_status arc_brain_load_body(const uint8_t* brain_data, size_t brain_size,
                                const arc_geometry* geom,
                                void* arena, size_t arena_size,
                                arc_engine* out);

#ifdef __cplusplus
}
#endif
