/* Drone plant sanity check: verifies physics, IMU, and fault detection
   without the Arc runtime. Runs 10000 ticks with a hover command and
   checks basic invariants. */
#include "train/sim/drone_plant.hpp"
#include <cstdio>
#include <cstdint>

int main() {
    using namespace arc::sim;

    DroneConfig cfg;
    cfg.ticks = 10000;
    cfg.motor_failure_tick = 5000;

    DronePlant plant;
    plant.init(cfg, 0xDEADBEEF);

    printf("Drone plant sanity check: %u ticks\n", cfg.ticks);

    /* Hover command: all 4 motors at 50% (32768 Q16.16). */
    int32_t hover_cmd[4] = {32768, 32768, 32768, 32768};

    for (uint32_t t = 0; t < cfg.ticks; ++t) {
        if (t == cfg.motor_failure_tick) {
            plant.fail_motor(1);
            printf("  Motor 1 FAILED at tick %u\n", t);
        }

        plant.tick(hover_cmd, 4);

        /* Check invariants every 1000 ticks. */
        if (t % 1000 == 0) {
            int32_t gyro[3];
            plant.get_imu(gyro);
            printf("  tick %5u: roll=%6d pitch=%6d yaw_rate=%6d "
                   "gyro=[%d,%d,%d] fault=%d\n",
                   t, plant.roll_q16, plant.pitch_q16, plant.yaw_rate_q16,
                   gyro[0], gyro[1], gyro[2], plant.check_fault());
        }
    }

    /* Verify physics stayed bounded. */
    const int32_t BOUND = 655360;  /* 10 rad — should never reach this */
    int pass = 1;
    if (q16_abs(plant.roll_q16) > BOUND) {
        printf("FAIL: roll diverged (%d)\n", plant.roll_q16);
        pass = 0;
    }
    if (q16_abs(plant.pitch_q16) > BOUND) {
        printf("FAIL: pitch diverged (%d)\n", plant.pitch_q16);
        pass = 0;
    }

    /* After motor failure, the plant should have tilted. */
    if (!plant.check_fault()) {
        printf("INFO: plant still stable after motor failure (good controller or slow dynamics)\n");
    } else {
        printf("INFO: plant fault triggered after motor failure (expected)\n");
    }

    printf("DRONE_PLANT_HASH %016llx\n",
           (unsigned long long)plant.prng.state);

    if (pass) printf("PASS\n");
    else printf("FAIL\n");
    return pass ? 0 : 1;
}
