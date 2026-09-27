#ifndef NAXES_TIMED_RAMP_H
#define NAXES_TIMED_RAMP_H

// Pitch ladder for FasTimed at 100000 steps/s^2 and 16 MHz ticks.
// Each value is the step period in driver ticks. The chunk uses as many
// steps as fit in 65535 ticks. The rising list is coarse. The falling list
// steps the ramp map one ramp-step at a time, which is what a legal
// deceleration from the top period requires.
//
// kNaxesTimedUpHold / kNaxesTimedDownHold repeats make each direction last
// about a second, so both the rise and the fall are audible.
//
// The RampMap of these periods is independent of the platform's
// getMaxSpeedInTicks() as long as the periods stay well above it, so the
// same ladder is legal on AVR, Pico, SAM and ESP32. The PC test
// extras/tests/pc_based/test_29 renders it as test_29.wav.

static const uint16_t kNaxesTimedUpPeriod[] = {36000, 25000, 18000, 14000,
                                               11000, 9000,  7500,  6400};
static const uint16_t kNaxesTimedDownPeriod[] = {
    6500,  6600,  6700,  6800,  6900,  7100,  7200,  7400,  7500,  7700,  7900,
    8100,  8300,  8500,  8700,  9000,  9300,  9600,  10000, 10400, 10900, 11400,
    12000, 12700, 13600, 14700, 16100, 18000, 20700, 25400, 35900};

static const uint8_t kNaxesTimedUpHold = 36;
static const uint8_t kNaxesTimedDownHold = 8;

static const uint32_t kNaxesTimedAccel = 100000;

#endif
