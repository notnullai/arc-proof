#pragma once

#include "arc_types.h"
#include "arc_plasticity.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ARC_SNAPSHOT_MAGIC       "ARCSNAP1"
#define ARC_SNAPSHOT_VERSION     1
#define ARC_SNAPSHOT_HEADER_SIZE 64

/* FNV-1a 64-bit. Used as the prior hash in snapshots. */
uint64_t arc_fnv1a64(const uint8_t* data, size_t size);

/* Serialize the current learned state (weights + full plastic state + flip
   log + reward window) into buf. Frozen engines (plasticity == NULL)
   serialize weights only. Returns ARC_OK and sets *written. */
arc_status arc_snapshot_serialize(const arc_engine* e, uint64_t tick,
                                  uint8_t* buf, size_t buf_size,
                                  size_t* written);

/* Weight-only snapshot (masks + prior hash), for the fault-time guarantee.
   Small (~1 KB for hidden=64). Not resumable; load_candidate accepts it. */
arc_status arc_snapshot_serialize_weights(const arc_engine* e, uint64_t tick,
                                          uint8_t* buf, size_t buf_size,
                                          size_t* written);

/* Resume: load a snapshot into an already-loaded engine. Requires the
   snapshot's prior hash to match the engine's current prior hash. */
arc_status arc_snapshot_deserialize(arc_engine* e, const uint8_t* buf,
                                    size_t size);

/* Load a snapshot as the new prior: masks become the prior, plastic state is
   cleared, flip log restarts. Any prior hash accepted. */
arc_status arc_snapshot_load_candidate(arc_engine* e, const uint8_t* buf,
                                       size_t size);

/* Hard reset: restore the prior, discard all learned state, cooldown.
   Triggered by API or the safety layer on a fault. */
void arc_plasticity_hard_reset(arc_engine* e, uint32_t tick);

#ifdef __cplusplus
}
#endif
