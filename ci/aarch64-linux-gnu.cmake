# Cross toolchain: build the proof suite for Linux ARM64 against
# lib/x64-linux-aarch64/libarc_runtime.a and run every test under
# qemu-user (no ARM64 hardware, no hosted runner). Static link so the
# emulator needs no sysroot.
#
#   cmake -S . -B build-aarch64 -DCMAKE_TOOLCHAIN_FILE=ci/aarch64-linux-gnu.cmake
#   cmake --build build-aarch64
#   ctest --test-dir build-aarch64 --output-on-failure
#
# Requires: g++-aarch64-linux-gnu, qemu-user-static
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_EXE_LINKER_FLAGS_INIT "-static")
set(CMAKE_CROSSCOMPILING_EMULATOR qemu-aarch64-static)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
