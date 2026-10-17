# One-command verification of the Arc proof suite (linux/x86_64):
#
#   docker build --platform linux/amd64 -t arc-proof .
#
# The build runs configure + compile + all 12 test suites on x86_64,
# the same 12 suites cross-built for ARM64 and run under qemu-user, and
# the QEMU Cortex-M3/M7 golden-twin checks. A green build is the proof.
# Re-run the suite in the built image with:
#
#   docker run --rm arc-proof
FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends \
        cmake ninja-build g++ qemu-system-arm gcc-arm-none-eabi ca-certificates \
        g++-aarch64-linux-gnu qemu-user-static \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /proof
COPY . .

RUN cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build \
    && ctest --test-dir build --output-on-failure

# ARM64: the same 12 suites, cross-built against lib/x64-linux-aarch64
# and executed under qemu-aarch64 (CMAKE_CROSSCOMPILING_EMULATOR).
RUN cmake -S . -B build-aarch64 -G Ninja -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_TOOLCHAIN_FILE=ci/aarch64-linux-gnu.cmake \
    && cmake --build build-aarch64 \
    && ctest --test-dir build-aarch64 --output-on-failure

# QEMU twin hashes: the same reflex loop that runs on target hardware,
# verified under emulation against the committed golden values.
RUN bash ci/run_mcu_qemu.sh

CMD ["ctest", "--test-dir", "build", "--output-on-failure"]
