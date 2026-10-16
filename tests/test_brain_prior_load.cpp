/* Trained prior load test: both published .arc002 priors must load via
   arc_load and produce deterministic, repeatable outputs from a fixed
   input sequence. Usage: test_brain_prior_load <model_a.arc002>
   <model_b.arc002> */
#include "arc/arc.h"
#include "arc/arc_types.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>

static std::vector<uint8_t> read_file(const char* path) {
    std::vector<uint8_t> out;
    FILE* f = fopen(path, "rb");
    if (!f) return out;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (n > 0 && n < (1 << 20)) {
        out.resize((size_t)n);
        if (fread(out.data(), 1, out.size(), f) != out.size()) out.clear();
    }
    fclose(f);
    return out;
}

static int check_one(const char* path) {
    std::vector<uint8_t> data = read_file(path);
    if (data.empty()) {
        printf("FAIL: cannot read %s\n", path);
        return 1;
    }

    arc_geometry geom;
    size_t need = 0;
    if (arc_get_required_memory(data.data(), data.size(), &geom, &need) != ARC_OK) {
        printf("FAIL: %s header rejected\n", path);
        return 1;
    }
    std::vector<uint8_t> arena(need + 64, 0);
    arc_engine* e = nullptr;
    if (arc_load(data.data(), data.size(), arena.data(), arena.size(), &e) != ARC_OK) {
        printf("FAIL: %s load rejected\n", path);
        return 1;
    }
    printf("%s: %zu bytes, geom %dx%dx%d\n", path, data.size(),
           geom.input_bits, geom.hidden_neurons, geom.motor_neurons);

    /* Fixed input sequence, run twice from reset; motor words must match. */
    std::vector<uint32_t> motors_a, motors_b;
    for (int run = 0; run < 2; ++run) {
        std::vector<uint32_t>& dst = (run == 0) ? motors_a : motors_b;
        (void)arc_reset(e);
        uint32_t seed = 0x600DF00Du;
        for (int t = 0; t < 64; ++t) {
            std::vector<uint32_t> words((size_t)geom.input_words, 0);
            for (int w = 0; w < geom.input_words; ++w) {
                seed = seed * 1664525u + 1013904223u;
                words[(size_t)w] = seed;
            }
            (void)arc_set_input(e, words.data());
            (void)arc_step(e);
            uint32_t m = 0;
            (void)arc_get_motor(e, &m);
            dst.push_back(m);
        }
    }
    if (motors_a != motors_b) {
        printf("FAIL: %s not deterministic across reset\n", path);
        return 1;
    }
    uint32_t nonzero = 0;
    for (size_t i = 0; i < motors_a.size(); ++i)
        if (motors_a[i] != 0) nonzero = 1;
    if (!nonzero) {
        printf("FAIL: %s produced only zero motors\n", path);
        return 1;
    }
    printf("PASS: %s deterministic over 64 ticks\n", path);
    return 0;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        printf("usage: %s <model_a.arc002> <model_b.arc002>\n", argv[0]);
        return 1;
    }
    int failed = 0;
    failed += check_one(argv[1]);
    failed += check_one(argv[2]);
    printf("%s\n", failed ? "FAIL" : "PASS");
    return failed ? 1 : 0;
}
