cmake_minimum_required(VERSION 3.15)
project(ArcProof LANGUAGES C CXX)

enable_testing()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -fno-exceptions -fno-rtti -fno-threadsafe-statics -Wall -Wextra -Werror -ffp-contract=off")
elseif(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2 /Wall /WX /EHsc- /GR- /wd5045")
endif()

# Engine: prebuilt, binary-only. Built from the private Arc repository
# at commit fba7469 (Windows: MSVC 19.44 /x64; Linux: GCC 13 / x86_64).
add_library(arc_runtime STATIC IMPORTED)
if(WIN32)
    set_target_properties(arc_runtime PROPERTIES
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/x64-windows/arc_runtime.lib)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set_target_properties(arc_runtime PROPERTIES
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/x64-linux-aarch64/libarc_runtime.a)
else()
    set_target_properties(arc_runtime PROPERTIES
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/x64-linux/libarc_runtime.a)
endif()
target_include_directories(arc_runtime INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

function(add_proof_test name source)
    add_executable(${name} ${source})
    target_include_directories(${name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    if(MSVC)
        target_compile_options(${name} PRIVATE /wd4365 /wd4267 /wd4189 /wd4710 /wd4711 /wd4820 /wd4626 /wd5027 /wd5045 /wd4514 /wd4018 /wd4505 /wd4101 /wd4244 /wd4996 /wd4464 /wd5219)
    else()
        target_compile_options(${name} PRIVATE -Wno-sign-conversion -Wno-sign-compare -Wno-unused-variable -Wno-padded)
    endif()
    target_link_libraries(${name} arc_runtime)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

# Bit-exactness twin: the same reflex loop that runs on target hardware,
# compared against the committed golden hash.
add_proof_test(test_mcu_drone_loop_reference tests/test_mcu_drone_loop_reference.cpp)

# Safety contract: rate-limited setpoints, fault ramp, watchdog.
add_proof_test(test_safety tests/test_safety.cpp)
add_proof_test(test_loop_fault tests/test_loop_fault.cpp)
add_proof_test(test_fault_gates_plasticity tests/test_fault_gates_plasticity.cpp)

# Fault injection through the real loop: watchdog overrun, estop latch,
# sensor dropout, single late tick.
add_proof_test(test_fault_injection tests/test_fault_injection.cpp)

# ESC pulse mapping: input us -> exact FlexPWM counts.
add_proof_test(test_esc_pulse tests/test_esc_pulse.cpp)
target_include_directories(test_esc_pulse PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/platforms/teensy)

# Drone plant suites (split of the former 10k-tick episode):
# motor-loss survival (golden-pinned), tilt-fault response, invariants.
function(add_drone_proof_test name source)
    add_executable(${name} ${source} platforms/sim/hal_drone_sim.cpp)
    target_include_directories(${name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
        ${CMAKE_CURRENT_SOURCE_DIR}/train/sim
        ${CMAKE_CURRENT_SOURCE_DIR}/platforms/sim
    )
    if(MSVC)
        target_compile_options(${name} PRIVATE /wd4710 /wd4711 /wd4820 /wd4626 /wd5027 /wd5045 /wd4189 /wd4101 /wd4514)
    else()
        target_compile_options(${name} PRIVATE -Wno-sign-conversion -Wno-sign-compare -Wno-unused-variable -Wno-padded)
    endif()
    target_link_libraries(${name} arc_runtime)
    add_test(NAME ${name} COMMAND ${name})
endfunction()
add_drone_proof_test(test_motor_loss_survival tests/test_motor_loss_survival.cpp)
add_drone_proof_test(test_tilt_fault_response tests/test_tilt_fault_response.cpp)
add_drone_proof_test(test_drone_sim_env tests/test_drone_sim_env.cpp)

# Loader robustness: arc_load is the only untrusted-input parser.
add_executable(test_arc_load_fuzz
    tests/test_arc_load_fuzz.cpp
    platforms/sim/hal_drone_sim.cpp
)
target_include_directories(test_arc_load_fuzz PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tests
    ${CMAKE_CURRENT_SOURCE_DIR}/train/sim
    ${CMAKE_CURRENT_SOURCE_DIR}/platforms/sim
)
if(MSVC)
    target_compile_options(test_arc_load_fuzz PRIVATE /wd4710 /wd4711 /wd4820 /wd4626 /wd5027 /wd5045 /wd4189 /wd4101 /wd4514)
else()
    target_compile_options(test_arc_load_fuzz PRIVATE -Wno-sign-conversion -Wno-sign-compare -Wno-unused-variable -Wno-padded)
endif()
target_link_libraries(test_arc_load_fuzz arc_runtime)
add_test(NAME test_arc_load_fuzz COMMAND test_arc_load_fuzz)

# Trained prior determinism: the committed models load and replay identically.
add_executable(test_brain_prior_load tests/test_brain_prior_load.cpp)
target_include_directories(test_brain_prior_load PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tests
)
if(MSVC)
    target_compile_options(test_brain_prior_load PRIVATE /wd4365 /wd4267 /wd4189 /wd4710 /wd4711 /wd4820 /wd4626 /wd5027 /wd5045 /wd4514 /wd4996)
else()
    target_compile_options(test_brain_prior_load PRIVATE -Wno-sign-conversion -Wno-sign-compare -Wno-unused-variable -Wno-padded)
endif()
target_link_libraries(test_brain_prior_load arc_runtime)
add_test(NAME test_brain_prior_load COMMAND test_brain_prior_load
    ${CMAKE_CURRENT_SOURCE_DIR}/models/prior_car.arc002
    ${CMAKE_CURRENT_SOURCE_DIR}/models/prior_servo.arc002)
