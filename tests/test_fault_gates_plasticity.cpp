#include "arc/arc_plasticity.h"
#include "arc/arc_types.h"
#include "test_plastic_util.hpp"
#include <cstdio>
#include <cstdint>
#include <vector>

int main() {
    int passed = 0, failed = 0;

    const int HID = 32;
    arc_plasticity_config cfg = pltest::default_cfg();
    cfg.plasticity_interval = 10;
    std::vector<int32_t> wp = {65536, 0, 0};

    pltest::Harness h(64, HID, 0, cfg, wp);
    if (!h) { printf("FAIL: harness\n"); return 1; }
    arc_engine* e = h.e;
    arc_plasticity* p = h.p;

    auto pass_at = [&](uint32_t tick) {
        for (int i = 0; i < (int)cfg.plasticity_interval; ++i)
            arc_plasticity_step(e, tick + (uint32_t)i);
    };

    /* Gated: traces decay, flips blocked. */
    *p->mod_state = 65536;
    p->E[0] = 100 * 65536;
    p->fault_gated = 1;
    const uint32_t before = p->flip_count;
    const int32_t e0 = p->E[0];
    pass_at(100);
    if (p->flip_count != before) { printf("FAIL: flip while fault-gated\n"); failed++; }
    else { printf("PASS: fault gate blocks flips\n"); passed++; }
    if (p->E[0] == e0) { printf("FAIL: traces frozen while gated\n"); failed++; }
    else { printf("PASS: traces still decay while gated\n"); passed++; }

    /* Ungated: flips resume. */
    p->E[0] = 100 * 65536;
    p->fault_gated = 0;
    pass_at(200);
    if (p->flip_count == before) { printf("FAIL: flips did not resume\n"); failed++; }
    else { printf("PASS: flips resume after fault clears\n"); passed++; }

    printf("\nFault-gate tests: %d passed, %d failed\n", passed, failed);
    return failed ? 1 : 0;
}
