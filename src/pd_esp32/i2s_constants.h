#ifndef PD_ESP32_I2S_CONSTANTS_H
#define PD_ESP32_I2S_CONSTANTS_H
#if defined(SUPPORT_ESP32_I2S)

// I2S timing: 16-bit stereo mode
// Sample rate: 250kHz
// Bits per frame: 16-bit L + 16-bit R = 32 bits
// BCLK = 250kHz × 32 bits = 8MHz
// Frame duration = 32 bits / 8MHz = 4µs
// At 16MHz stepper reference: 4µs × 16 = 64 ticks per frame
#define I2S_SAMPLE_RATE_HZ 250000UL
#define I2S_TICKS_PER_FRAME 64
#define I2S_BITS_PER_FRAME 32
#define I2S_BYTES_PER_FRAME 4

// All constants derived from I2S_BLOCK_DURATION_US (per design doc)
#define I2S_BLOCK_DURATION_US 500
#define I2S_BLOCK_COUNT 2
#define I2S_FRAMES_PER_BLOCK \
  (I2S_BLOCK_DURATION_US * I2S_SAMPLE_RATE_HZ / 1000000UL)
#define I2S_BLOCK_TICKS (I2S_FRAMES_PER_BLOCK * I2S_TICKS_PER_FRAME)
#define I2S_BYTES_PER_BLOCK (I2S_FRAMES_PER_BLOCK * I2S_BYTES_PER_FRAME)

// Total buffer (all 2 blocks combined)
#define I2S_TOTAL_FRAMES (I2S_BLOCK_COUNT * I2S_FRAMES_PER_BLOCK)
#define I2S_TOTAL_BYTES (I2S_BLOCK_COUNT * I2S_BYTES_PER_BLOCK)

// Max frequency enforced by addQueueEntry(), not by I2S driver
#define MAX_STEP_FREQ_HZ 200000UL

// Max pulses per block (for tracking array size)
// 500µs × 200kHz = 100, +1 for safety
#define I2S_MAX_PULSES_PER_BLOCK \
  ((I2S_BLOCK_DURATION_US * MAX_STEP_FREQ_HZ / 1000000UL) + 1)

#define I2S_DIRECT_MIN_SPEED_TICKS 80

// 100kHz step frequency corresponds to 10µs period, so 2µs pulse width is 20%
#define I2S_DEFAULT_PULSE_WIDTH_TICKS 32

// DMA: minimum for continuous streaming (per design doc)
#define I2S_DMA_DESC_NUM 2
#define I2S_DMA_FRAME_NUM I2S_FRAMES_PER_BLOCK

#endif  // SUPPORT_ESP32_I2S
#endif  // PD_ESP32_I2S_CONSTANTS_H
