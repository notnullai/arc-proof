/* arc_load robustness fuzz: the loader is the only untrusted-input parser
   (evaluators feed their own .arc002 models). Deterministic mutation
   suite: truncations, magic corruption, header field bombs, seeded
   bitflips. Success bar: every input returns a status, none crashes. */
#include "train/sim/drone_env.hpp"
#include "arc/arc.h"
#include "arc/arc_types.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>

using namespace arc::sim;

static uint32_t rng_state = 0x12345678u;
static uint32_t rng_next() {
    rng_state = rng_state * 1664525u + 1013904223u;
    return rng_state;
}

int main() {
    int failed = 0;

    DroneConfig dcfg;
    dcfg.fire_threshold = 1;
    DroneBrain bb(dcfg);
    if (bb.brain.empty()) { printf("FAIL: corpus build\n"); return 1; }
    const std::vector<uint8_t>& corpus = bb.brain;
    printf("corpus: %zu bytes\n", corpus.size());

    size_t need = 0;
    arc_geometry geom;
    if (arc_get_required_memory(corpus.data(), corpus.size(),
                                &geom, &need) != ARC_OK || need == 0) {
        printf("FAIL: required_memory on valid corpus\n");
        return 1;
    }
    std::vector<uint8_t> arena(need + 64, 0);

    /* The same suite runs through both loader entry points: arc_load and
       arc_load_inference (no plasticity state; how RAM-tight firmware loads). */
    typedef arc_status (*load_fn)(const uint8_t*, size_t, void*, size_t, arc_engine**);
    const load_fn loaders[2] = { arc_load, arc_load_inference };
    const char* const names[2] = { "arc_load", "arc_load_inference" };
    for (int li = 0; li < 2; ++li) {
    const load_fn load = loaders[li];
    rng_state = 0x12345678u;
    printf("-- %s\n", names[li]);

    /* 1. Empty input. */
    {
        arc_engine* e = nullptr;
        arc_status st = load(corpus.data(), 0, arena.data(), arena.size(), &e);
        if (st == ARC_OK) { printf("FAIL: empty input accepted\n"); failed++; }
        else printf("PASS: empty input rejected (%s)\n", arc_strerror(st));
    }

    /* 2. Magic corruption. */
    {
        std::vector<uint8_t> bad = corpus;
        bad[0] ^= 0xFF;
        arc_engine* e = nullptr;
        arc_status st = load(bad.data(), bad.size(), arena.data(),
                                 arena.size(), &e);
        if (st == ARC_OK) { printf("FAIL: bad magic accepted\n"); failed++; }
        else printf("PASS: bad magic rejected (%s)\n", arc_strerror(st));
    }

    /* 3. Truncations at every 16-byte step. */
    {
        int accepted = 0, ran = 0;
        for (size_t n = 0; n < corpus.size(); n += 16) {
            arc_engine* e = nullptr;
            (void)load(corpus.data(), n, arena.data(), arena.size(), &e);
            if (e) accepted++;
            ran++;
        }
        if (accepted != 0) {
            printf("FAIL: %d/%d truncations accepted\n", accepted, ran);
            failed++;
        } else {
            printf("PASS: %d truncations all rejected, no crash\n", ran);
        }
    }

    /* 4. Header field bombs (bytes 0..127: size/geometry fields).
       Contract: every input returns a status and never crashes. Fields the
       format defines as free (fire_threshold, leak_q16, reserved) may load;
       an accepted engine must still be usable (step + get_motor). */
    {
        int accepted = 0, ran = 0;
        for (int field = 0; field < 32; ++field) {
            std::vector<uint8_t> bad = corpus;
            const size_t off = 8 + (size_t)field * 4;
            if (off + 4 > bad.size()) break;
            bad[off + 0] = 0xFF; bad[off + 1] = 0xFF;
            bad[off + 2] = 0xFF; bad[off + 3] = 0x7F;
            arc_engine* e = nullptr;
            (void)load(bad.data(), bad.size(), arena.data(), arena.size(), &e);
            ran++;
            if (e) {
                accepted++;
                uint32_t motor = 0;
                (void)arc_step(e);
                (void)arc_get_motor(e, &motor);
            }
        }
        printf("PASS: %d header bombs handled (%d accepted as usable, no crash)\n",
               ran, accepted);
    }

    /* 5. Seeded bitflips: 2000 random single-bit mutations. */
    {
        int ok_count = 0;
        for (int i = 0; i < 2000; ++i) {
            std::vector<uint8_t> bad = corpus;
            const uint32_t pos = rng_next() % (uint32_t)bad.size();
            const uint32_t bit = rng_next() % 8u;
            bad[pos] ^= (uint8_t)(1u << bit);
            arc_engine* e = nullptr;
            if (load(bad.data(), bad.size(), arena.data(),
                         arena.size(), &e) == ARC_OK) {
                ok_count++;      /* tolerated only if engine is usable */
                if (e) {
                    uint32_t motor = 0;
                    (void)arc_step(e);
                    (void)arc_get_motor(e, &motor);
                }
            }
        }
        printf("PASS: 2000 bitflips completed (%d accepted as loadable)\n",
               ok_count);
    }

    /* 6. Whole-file garbage. */
    {
        std::vector<uint8_t> bad(512, 0xA5);
        arc_engine* e = nullptr;
        arc_status st = load(bad.data(), bad.size(), arena.data(),
                                 arena.size(), &e);
        if (st == ARC_OK) { printf("FAIL: garbage accepted\n"); failed++; }
        else printf("PASS: garbage rejected (%s)\n", arc_strerror(st));
    }
    }   /* loaders */

    if (failed) printf("FAIL\n");
    else printf("PASS\n");
    return failed ? 1 : 0;
}
