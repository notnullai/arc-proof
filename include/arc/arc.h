#pragma once

#include "arc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

arc_status arc_get_required_memory(const uint8_t* header, size_t header_size,
                                    arc_geometry* geom_out, size_t* bytes_out);

arc_status arc_load(const uint8_t* brain_data, size_t brain_size,
                     void* arena, size_t arena_size,
                     arc_engine** out);

arc_status arc_set_input(arc_engine* e, const uint32_t* words);
arc_status arc_step(arc_engine* e);
arc_status arc_step_with_overrides(arc_engine* e, uint32_t eligible_mask);
arc_status arc_get_motor(const arc_engine* e, uint32_t* out_words);
arc_status arc_get_groups(const arc_engine* e, uint8_t* out_groups);
arc_status arc_reset(arc_engine* e);

const arc_geometry* arc_geometry_of(const arc_engine* e);
const char* arc_strerror(arc_status s);
const char* arc_last_error(void);

#ifdef __cplusplus
}
#endif