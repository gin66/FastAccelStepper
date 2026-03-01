#ifndef PD_ESP32_I2S_CONSTANTS_H
#define PD_ESP32_I2S_CONSTANTS_H
#if defined(SUPPORT_ESP32_I2S)

// I2S timing: BCLK=4MHz, stereo 8-bit
// 250 kHz sample rate keeps MCLK=32 MHz (with mclk_multiple=128) below APB/2=40
// MHz. Frame = L-byte (8 BCLK) + R-byte (8 BCLK) = 4µs = 64 ticks at 16MHz Step
// pulse: both bytes 0xFF → full frame HIGH = 4µs pulse
#define I2S_SAMPLE_RATE_HZ 250000UL
#define I2S_TICKS_PER_FRAME 64
#define I2S_BYTES_PER_FRAME 2

// Triple buffering: 3 blocks of ~1ms each
// 1ms / 4µs = 250 frames per block
#define I2S_BLOCK_COUNT 3
#define I2S_FRAMES_PER_BLOCK 250
#define I2S_BYTES_PER_BLOCK (I2S_FRAMES_PER_BLOCK * I2S_BYTES_PER_FRAME)

// Total buffer (all 3 blocks combined)
#define I2S_TOTAL_FRAMES (I2S_BLOCK_COUNT * I2S_FRAMES_PER_BLOCK)
#define I2S_TOTAL_BYTES (I2S_BLOCK_COUNT * I2S_BYTES_PER_BLOCK)

// Min step period: 2 frames = 8µs = 128 ticks
#define I2S_MIN_SPEED_TICKS 128

// DMA: 16 descriptors x 500 frames = 8000 frames = 32ms total buffer
#define I2S_DMA_DESC_NUM 16
#define I2S_DMA_FRAME_NUM 500

// Extra task cycles to stream after queue empties (ensures DMA flushes)
#define I2S_DRAIN_TASKS 4

// Max pulses per block (for tracking array size)
#define I2S_MAX_PULSES_PER_BLOCK 128

#endif  // SUPPORT_ESP32_I2S
#endif  // PD_ESP32_I2S_CONSTANTS_H
