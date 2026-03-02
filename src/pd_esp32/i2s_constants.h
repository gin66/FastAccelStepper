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
#define I2S_TICKS_PER_FRAME 64  // Frame duration in 16MHz ticks (4µs)
#define I2S_BITS_PER_FRAME 32   // 16-bit L + 16-bit R
#define I2S_BYTES_PER_FRAME 4   // 16-bit stereo: 2 bytes L + 2 bytes R

// Triple buffering: 3 blocks of ~1ms each
// 1ms / 4µs = 250 frames per block = 1000 bytes per block
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
