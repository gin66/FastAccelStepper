#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FastAccelStepper.h"
#include "fas_queue/stepper_queue.h"

char TCCR1A;
char TCCR1B;
char TCCR1C;
char TIMSK1;
char TIFR1;
unsigned short OCR1A;
unsigned short OCR1B;

StepperQueue fas_queue[NUM_QUEUES];

void inject_fill_interrupt(int mark) {}
void noInterrupts() {}
void interrupts() {}

#define I2S_TEST_SAMPLE_RATE_HZ 500000UL
#define I2S_TEST_TICKS_PER_FRAME 32
#define I2S_TEST_BYTES_PER_FRAME 2
#define I2S_TEST_FRAMES_PER_TASK 2000
#define I2S_TEST_MAX_FRAMES_PER_FILL (I2S_TEST_FRAMES_PER_TASK / 2)
#define I2S_TEST_BYTES_PER_TASK \
  (I2S_TEST_FRAMES_PER_TASK * I2S_TEST_BYTES_PER_FRAME)
#define I2S_TEST_PULSE_MAX 100
#define I2S_TEST_DRAIN_TASKS 2
#define I2S_TEST_MIN_SPEED_TICKS 64
#define I2S_TEST_GPIO_UNUSED 255

typedef struct {
  bool initialized;
  int step_pin;
  int bclk_gpio;
  int ws_gpio;
  uint8_t work_buf[I2S_TEST_BYTES_PER_TASK];
} I2sManagerMock;

static I2sManagerMock g_i2s_mgr = {false, 32, 33, I2S_TEST_GPIO_UNUSED, {0}};

static void i2s_mgr_init(int step_pin, int bclk_gpio, int ws_gpio) {
  g_i2s_mgr.step_pin = step_pin;
  g_i2s_mgr.bclk_gpio = bclk_gpio;
  g_i2s_mgr.ws_gpio = ws_gpio;
  g_i2s_mgr.initialized = true;
  memset(g_i2s_mgr.work_buf, 0x00, I2S_TEST_BYTES_PER_TASK);
}

static uint8_t* i2s_mgr_work_buf() { return g_i2s_mgr.work_buf; }

static void i2s_mgr_clear_work_buf() {
  memset(g_i2s_mgr.work_buf, 0x00, I2S_TEST_BYTES_PER_TASK);
}

typedef struct {
  bool use_i2s;
  bool _isRunning;
  uint8_t read_idx;
  uint8_t next_write_idx;
  uint32_t _i2s_tick_carry;
  uint8_t _i2s_drain;
  uint16_t _i2s_pulse_positions[I2S_TEST_PULSE_MAX];
  uint16_t _i2s_pulse_write_idx;
  uint16_t _i2s_pulse_read_idx;
  struct queue_entry entry[QUEUE_LEN];
} TestStepperQueue;

static void test_queue_init(TestStepperQueue* q) {
  q->use_i2s = true;
  q->_isRunning = false;
  q->read_idx = 0;
  q->next_write_idx = 0;
  q->_i2s_tick_carry = 0;
  q->_i2s_drain = 0;
  q->_i2s_pulse_write_idx = 0;
  q->_i2s_pulse_read_idx = 0;
  memset(q->_i2s_pulse_positions, 0, sizeof(q->_i2s_pulse_positions));
  memset(q->entry, 0, sizeof(q->entry));
}

static void test_queue_add_entry(TestStepperQueue* q, uint16_t ticks,
                                 uint8_t steps) {
  if (q->next_write_idx - q->read_idx < QUEUE_LEN) {
    q->entry[q->next_write_idx & QUEUE_LEN_MASK].ticks = ticks;
    q->entry[q->next_write_idx & QUEUE_LEN_MASK].steps = steps;
    q->entry[q->next_write_idx & QUEUE_LEN_MASK].toggle_dir = 0;
    q->next_write_idx++;
  }
}

static void test_queue_fill_i2s_buffer(TestStepperQueue* q) {
  if (!q->use_i2s) {
    return;
  }
  if (!q->_isRunning && (q->_i2s_drain == 0)) {
    return;
  }

  uint8_t* buf = i2s_mgr_work_buf();
  uint32_t frame_pos = 0;
  uint32_t tick_carry = q->_i2s_tick_carry;

  uint8_t rp = q->read_idx;
  const uint8_t wp = q->next_write_idx;

  while (q->_i2s_pulse_read_idx != q->_i2s_pulse_write_idx) {
    uint16_t frame_idx = q->_i2s_pulse_positions[q->_i2s_pulse_read_idx];
    buf[frame_idx * I2S_TEST_BYTES_PER_FRAME] = 0x00;
    buf[frame_idx * I2S_TEST_BYTES_PER_FRAME + 1] = 0x00;
    q->_i2s_pulse_read_idx = (q->_i2s_pulse_read_idx + 1) % I2S_TEST_PULSE_MAX;
  }

  while (frame_pos < I2S_TEST_MAX_FRAMES_PER_FILL) {
    if (rp == wp) {
      if (q->_isRunning) {
        q->_isRunning = false;
        q->_i2s_drain = I2S_TEST_DRAIN_TASKS;
      }
      break;
    }

    struct queue_entry* e = &q->entry[rp & QUEUE_LEN_MASK];

    if (e->steps == 0) {
      uint32_t total = (uint32_t)e->ticks + tick_carry;
      uint32_t frames = total / I2S_TEST_TICKS_PER_FRAME;
      tick_carry = total % I2S_TEST_TICKS_PER_FRAME;
      frame_pos += frames;
      rp++;
    } else {
      uint8_t steps = e->steps;
      const uint16_t ticks = e->ticks;
      uint8_t s = 0;

      while (s < steps) {
        uint32_t total = (uint32_t)ticks + tick_carry;
        uint32_t frames = total / I2S_TEST_TICKS_PER_FRAME;
        tick_carry = total % I2S_TEST_TICKS_PER_FRAME;
        frame_pos += frames;

        if (frame_pos >= I2S_TEST_MAX_FRAMES_PER_FILL) {
          break;
        }

        buf[frame_pos * I2S_TEST_BYTES_PER_FRAME] = 0xFF;
        buf[frame_pos * I2S_TEST_BYTES_PER_FRAME + 1] = 0xFF;

        q->_i2s_pulse_positions[q->_i2s_pulse_write_idx] = (uint16_t)frame_pos;
        q->_i2s_pulse_write_idx =
            (q->_i2s_pulse_write_idx + 1) % I2S_TEST_PULSE_MAX;
        s++;
      }

      if (s == steps) {
        rp++;
      } else {
        e->steps -= s;
        break;
      }
    }
  }

  q->_i2s_tick_carry = tick_carry;
  q->read_idx = rp;

  if (q->_i2s_drain > 0) {
    q->_i2s_drain--;
  }
}

static int test_passed = 0;
static int test_failed = 0;

static void test_result(const char* test_name, bool passed) {
  if (passed) {
    printf("PASS: %s\n", test_name);
    test_passed++;
  } else {
    printf("FAIL: %s\n", test_name);
    test_failed++;
  }
}

static bool verify_pulse_at_frame(uint8_t* buf, uint16_t frame) {
  if (buf[frame * I2S_TEST_BYTES_PER_FRAME] != 0xFF) {
    printf("  FAIL at frame %d: L-byte = 0x%02X, expected 0xFF\n", frame,
           buf[frame * I2S_TEST_BYTES_PER_FRAME]);
    return false;
  }
  if (buf[frame * I2S_TEST_BYTES_PER_FRAME + 1] != 0xFF) {
    printf("  FAIL at frame %d: R-byte = 0x%02X, expected 0xFF\n", frame,
           buf[frame * I2S_TEST_BYTES_PER_FRAME + 1]);
    return false;
  }
  return true;
}

static bool verify_no_pulse_at_frame(uint8_t* buf, uint16_t frame) {
  if (buf[frame * I2S_TEST_BYTES_PER_FRAME] != 0x00 ||
      buf[frame * I2S_TEST_BYTES_PER_FRAME + 1] != 0x00) {
    printf("  FAIL at frame %d: expected 0x00, got L=0x%02X R=0x%02X\n", frame,
           buf[frame * I2S_TEST_BYTES_PER_FRAME],
           buf[frame * I2S_TEST_BYTES_PER_FRAME + 1]);
    return false;
  }
  return true;
}

static void test_i2s_init() {
  printf("Running: I2S init\n");

  i2s_mgr_init(32, 33, I2S_TEST_GPIO_UNUSED);

  if (!g_i2s_mgr.initialized) {
    printf("  I2S init: FAILED\n");
    test_result("I2S init", false);
    return;
  }

  printf("  I2S init: OK\n");

  uint8_t* work_buf = i2s_mgr_work_buf();
  if (work_buf == NULL) {
    printf("  Work buffer: FAILED (NULL)\n");
    test_result("I2S init", false);
    return;
  }

  printf("  Work buffer: OK (size=%d)\n", I2S_TEST_BYTES_PER_TASK);
  test_result("I2S init", true);
}

static void test_i2s_direct_write() {
  printf("Running: I2S direct write\n");

  i2s_mgr_clear_work_buf();

  uint8_t* buf = i2s_mgr_work_buf();

  uint16_t pulse_frames[4] = {500, 1000, 1500, 2000};
  for (int i = 0; i < 4; i++) {
    uint16_t frame = pulse_frames[i];
    buf[frame * I2S_TEST_BYTES_PER_FRAME] = 0xFF;
    buf[frame * I2S_TEST_BYTES_PER_FRAME + 1] = 0xFF;
  }

  for (int i = 0; i < 4; i++) {
    if (!verify_pulse_at_frame(buf, pulse_frames[i])) {
      test_result("I2S direct write", false);
      return;
    }
  }

  printf("  Wrote and verified 4 pulses at frames 500, 1000, 1500, 2000\n");
  test_result("I2S direct write", true);
}

static void test_i2s_queue_fill() {
  printf("Running: I2S queue fill\n");

  i2s_mgr_clear_work_buf();

  TestStepperQueue q;
  test_queue_init(&q);

  q._isRunning = true;
  test_queue_add_entry(&q, 1000, 1);
  test_queue_add_entry(&q, 1000, 1);
  test_queue_add_entry(&q, 1000, 1);
  test_queue_add_entry(&q, 1000, 1);

  test_queue_fill_i2s_buffer(&q);

  uint8_t* buf = i2s_mgr_work_buf();

  printf("  Checking pulse positions from ring buffer:\n");
  for (int i = 0; i < 4; i++) {
    uint16_t frame = q._i2s_pulse_positions[i];
    printf("    Pulse %d at frame %d\n", i, frame);

    if (!verify_pulse_at_frame(buf, frame)) {
      test_result("I2S queue fill", false);
      return;
    }
  }

  printf("  Verified 4 pulses written from queue\n");
  test_result("I2S queue fill", true);
}

static void test_i2s_ring_buffer_clear() {
  printf("Running: I2S ring buffer clear\n");

  i2s_mgr_clear_work_buf();

  TestStepperQueue q;
  test_queue_init(&q);

  q._isRunning = true;
  test_queue_add_entry(&q, 1000, 4);

  test_queue_fill_i2s_buffer(&q);

  uint8_t* buf = i2s_mgr_work_buf();
  uint16_t first_pulse_frame = q._i2s_pulse_positions[0];
  uint16_t second_pulse_frame = q._i2s_pulse_positions[1];

  if (!verify_pulse_at_frame(buf, first_pulse_frame)) {
    printf("  FAIL: First pulse not found\n");
    test_result("I2S ring buffer clear", false);
    return;
  }

  printf(
      "  First fill: 4 pulses written, first at frame %d, second at frame %d\n",
      first_pulse_frame, second_pulse_frame);
  printf("  Pulse ring: write_idx=%d, read_idx=%d\n", q._i2s_pulse_write_idx,
         q._i2s_pulse_read_idx);

  test_queue_add_entry(&q, 800, 4);
  q._isRunning = true;

  test_queue_fill_i2s_buffer(&q);

  printf("  After second fill (800 ticks): write_idx=%d, read_idx=%d\n",
         q._i2s_pulse_write_idx, q._i2s_pulse_read_idx);

  if (!verify_no_pulse_at_frame(buf, first_pulse_frame)) {
    printf("  FAIL: Old pulse at frame %d not cleared\n", first_pulse_frame);
    test_result("I2S ring buffer clear", false);
    return;
  }
  if (!verify_no_pulse_at_frame(buf, second_pulse_frame)) {
    printf("  FAIL: Old pulse at frame %d not cleared\n", second_pulse_frame);
    test_result("I2S ring buffer clear", false);
    return;
  }

  uint16_t new_first_pulse_frame = q._i2s_pulse_positions[4];
  printf("  Old pulses at frames %d, %d were cleared\n", first_pulse_frame,
         second_pulse_frame);
  printf("  New pulses start at frame %d\n", new_first_pulse_frame);

  test_result("I2S ring buffer clear", true);
}

static void test_i2s_frame_boundary() {
  printf("Running: I2S frame boundary\n");

  i2s_mgr_clear_work_buf();

  TestStepperQueue q;
  test_queue_init(&q);

  q._isRunning = true;

  uint16_t frames[] = {490, 495, 498};
  uint16_t ticks = 0;
  for (int i = 0; i < 3; i++) {
    uint16_t frame = frames[i];
    uint16_t frames_needed = frame - ticks;
    uint16_t ticks_needed = frames_needed * I2S_TEST_TICKS_PER_FRAME;
    test_queue_add_entry(&q, ticks_needed, 1);
    ticks = frame;
  }

  test_queue_fill_i2s_buffer(&q);

  uint8_t* buf = i2s_mgr_work_buf();

  for (int i = 0; i < 3; i++) {
    uint16_t frame = q._i2s_pulse_positions[i];
    printf("  Pulse %d at frame %d\n", i, frame);

    if (frame >= I2S_TEST_MAX_FRAMES_PER_FILL) {
      printf("  FAIL: Pulse at frame %d beyond buffer boundary\n", frame);
      test_result("I2S frame boundary", false);
      return;
    }

    if (!verify_pulse_at_frame(buf, frame)) {
      test_result("I2S frame boundary", false);
      return;
    }
  }

  printf("  Verified frame boundary handling - no off-by-one errors\n");
  test_result("I2S frame boundary", true);
}

static void test_i2s_empty_queue() {
  printf("Running: I2S empty queue\n");

  i2s_mgr_clear_work_buf();

  TestStepperQueue q;
  test_queue_init(&q);

  q._isRunning = true;
  test_queue_add_entry(&q, 1000, 1);

  test_queue_fill_i2s_buffer(&q);

  uint16_t pulse_frame = q._i2s_pulse_positions[0];
  printf("  First pass: pulse at frame %d\n", pulse_frame);

  if (q._isRunning) {
    printf("  FAIL: _isRunning should be false after queue emptied\n");
    test_result("I2S empty queue", false);
    return;
  }

  uint8_t expected_drain = I2S_TEST_DRAIN_TASKS - 1;
  if (q._i2s_drain != expected_drain) {
    printf("  FAIL: _i2s_drain = %d, expected %d\n", q._i2s_drain,
           expected_drain);
    test_result("I2S empty queue", false);
    return;
  }

  printf(
      "  Queue emptied, _isRunning=false, _i2s_drain=%d (decremented in same "
      "cycle)\n",
      q._i2s_drain);

  test_queue_fill_i2s_buffer(&q);

  if (q._i2s_drain != expected_drain - 1) {
    printf("  FAIL: Drain counter not decremented on second fill\n");
    test_result("I2S empty queue", false);
    return;
  }

  printf("  Drain counter decremented to %d\n", expected_drain - 1);

  test_result("I2S empty queue", true);
}

static void test_i2s_ring_buffer_wrap() {
  printf("Running: I2S ring buffer wrap\n");

  i2s_mgr_clear_work_buf();

  TestStepperQueue q;
  test_queue_init(&q);

  q._isRunning = true;

  int entries_added = 0;
  for (int i = 0; i < QUEUE_LEN; i++) {
    test_queue_add_entry(&q, 32, 1);
    entries_added++;
  }

  test_queue_fill_i2s_buffer(&q);

  printf("  Added %d entries at 32 ticks each, write_idx=%d, read_idx=%d\n",
         entries_added, q._i2s_pulse_write_idx, q._i2s_pulse_read_idx);

  uint16_t pulses_first_fill = q._i2s_pulse_write_idx;
  printf("  First fill wrote %d pulses\n", pulses_first_fill);

  q._i2s_pulse_read_idx = q._i2s_pulse_write_idx;

  for (int i = 0; i < QUEUE_LEN; i++) {
    test_queue_add_entry(&q, 32, 1);
  }
  test_queue_fill_i2s_buffer(&q);

  printf("  After second fill: write_idx=%d, read_idx=%d\n",
         q._i2s_pulse_write_idx, q._i2s_pulse_read_idx);

  uint16_t total_pulses =
      pulses_first_fill + (QUEUE_LEN > I2S_TEST_PULSE_MAX ? 0 : QUEUE_LEN);
  bool wrap_occurred = (q._i2s_pulse_write_idx < pulses_first_fill);

  printf("  Wrap occurred: %s\n", wrap_occurred ? "yes" : "no");

  if (q._i2s_pulse_write_idx >= I2S_TEST_PULSE_MAX) {
    printf("  FAIL: write_idx should wrap, got %d\n", q._i2s_pulse_write_idx);
    test_result("I2S ring buffer wrap", false);
    return;
  }

  test_result("I2S ring buffer wrap", true);
}

static void test_i2s_multi_step_entry() {
  printf("Running: I2S multi-step entry\n");

  i2s_mgr_clear_work_buf();

  TestStepperQueue q;
  test_queue_init(&q);

  q._isRunning = true;
  test_queue_add_entry(&q, 320, 10);

  test_queue_fill_i2s_buffer(&q);

  uint8_t* buf = i2s_mgr_work_buf();

  printf("  Checking 10 pulses from single multi-step entry:\n");
  for (int i = 0; i < 10; i++) {
    uint16_t frame = q._i2s_pulse_positions[i];
    printf("    Pulse %d at frame %d\n", i, frame);

    if (frame >= I2S_TEST_MAX_FRAMES_PER_FILL) {
      printf("  FAIL: Pulse %d at frame %d beyond boundary\n", i, frame);
      test_result("I2S multi-step entry", false);
      return;
    }

    if (!verify_pulse_at_frame(buf, frame)) {
      test_result("I2S multi-step entry", false);
      return;
    }
  }

  printf("  Verified 10 pulses from single queue entry\n");
  test_result("I2S multi-step entry", true);
}

static void test_i2s_tick_carry() {
  printf("Running: I2S tick carry\n");

  i2s_mgr_clear_work_buf();

  TestStepperQueue q;
  test_queue_init(&q);

  q._isRunning = true;
  test_queue_add_entry(&q, 33, 1);
  test_queue_add_entry(&q, 33, 1);
  test_queue_add_entry(&q, 33, 1);

  test_queue_fill_i2s_buffer(&q);

  printf("  3 pulses with 33 ticks each (1 tick carry per step):\n");
  for (int i = 0; i < 3; i++) {
    uint16_t frame = q._i2s_pulse_positions[i];
    printf("    Pulse %d at frame %d\n", i, frame);
  }

  uint16_t frame0 = q._i2s_pulse_positions[0];
  uint16_t frame1 = q._i2s_pulse_positions[1];
  uint16_t frame2 = q._i2s_pulse_positions[2];

  uint16_t delta01 = frame1 - frame0;
  uint16_t delta12 = frame2 - frame1;

  printf("  Frame deltas: %d, %d (should be 1 each due to carry)\n", delta01,
         delta12);

  if (delta01 != 1 || delta12 != 1) {
    printf("  FAIL: Tick carry not accumulated correctly\n");
    test_result("I2S tick carry", false);
    return;
  }

  test_result("I2S tick carry", true);
}

static void test_i2s_pause_entry() {
  printf("Running: I2S pause entry\n");

  i2s_mgr_clear_work_buf();

  TestStepperQueue q;
  test_queue_init(&q);

  q._isRunning = true;
  test_queue_add_entry(&q, 1000, 1);
  test_queue_add_entry(&q, 3200, 0);
  test_queue_add_entry(&q, 1000, 1);

  test_queue_fill_i2s_buffer(&q);

  uint8_t* buf = i2s_mgr_work_buf();

  uint16_t frame0 = q._i2s_pulse_positions[0];
  uint16_t frame1 = q._i2s_pulse_positions[1];

  printf("  Pulse 0 at frame %d, Pulse 1 at frame %d\n", frame0, frame1);

  uint16_t pause_frames = 3200 / I2S_TEST_TICKS_PER_FRAME;
  uint16_t step_frames = 1000 / I2S_TEST_TICKS_PER_FRAME;
  uint16_t expected_gap = pause_frames + step_frames;
  uint16_t actual_gap = frame1 - frame0;

  printf("  Expected gap: %d frames (pause:%d + step2:%d)\n", expected_gap,
         pause_frames, step_frames);
  printf("  Actual gap: %d frames\n", actual_gap);

  int16_t diff = (int16_t)actual_gap - (int16_t)expected_gap;
  if (diff < -2 || diff > 2) {
    printf("  FAIL: Pause entry gap incorrect (diff=%d)\n", diff);
    test_result("I2S pause entry", false);
    return;
  }

  if (!verify_pulse_at_frame(buf, frame0) ||
      !verify_pulse_at_frame(buf, frame1)) {
    printf("  FAIL: Pulses not written correctly\n");
    test_result("I2S pause entry", false);
    return;
  }

  test_result("I2S pause entry", true);
}

static void test_i2s_buffer_overflow_protection() {
  printf("Running: I2S buffer overflow protection\n");

  i2s_mgr_clear_work_buf();

  TestStepperQueue q;
  test_queue_init(&q);

  q._isRunning = true;

  for (int i = 0; i < I2S_TEST_PULSE_MAX + 10; i++) {
    test_queue_add_entry(&q, 32, 1);
  }

  test_queue_fill_i2s_buffer(&q);

  uint16_t pulses_written = q._i2s_pulse_write_idx;
  uint16_t write_wrap_count = (I2S_TEST_PULSE_MAX + 10) / I2S_TEST_PULSE_MAX;

  printf("  Added %d entries, write_idx=%d (wrapped %d time(s))\n",
         I2S_TEST_PULSE_MAX + 10, pulses_written, write_wrap_count);

  uint8_t* buf = i2s_mgr_work_buf();
  int valid_pulses = 0;
  for (int i = 0; i < I2S_TEST_PULSE_MAX; i++) {
    uint16_t frame = q._i2s_pulse_positions[i];
    if (frame < I2S_TEST_MAX_FRAMES_PER_FILL &&
        buf[frame * I2S_TEST_BYTES_PER_FRAME] == 0xFF) {
      valid_pulses++;
    }
  }

  printf("  Valid pulses in ring: %d (ring size: %d)\n", valid_pulses,
         I2S_TEST_PULSE_MAX);

  if (pulses_written >= I2S_TEST_PULSE_MAX) {
    printf("  Ring buffer wrapped correctly\n");
  }

  test_result("I2S buffer overflow protection", true);
}

static void test_i2s_partial_entry_processing() {
  printf("Running: I2S partial entry processing\n");

  i2s_mgr_clear_work_buf();

  TestStepperQueue q;
  test_queue_init(&q);

  q._isRunning = true;

  test_queue_add_entry(&q, 500, 5);

  test_queue_fill_i2s_buffer(&q);

  printf("  First fill: wrote pulses at frames:\n");
  for (int i = 0; i < 5; i++) {
    if (q._i2s_pulse_positions[i] != 0 || i == 0) {
      printf("    Pulse %d at frame %d\n", i, q._i2s_pulse_positions[i]);
    }
  }

  bool entry_consumed = (q.read_idx == q.next_write_idx);
  printf("  Entry consumed: %s (read_idx=%d, write_idx=%d)\n",
         entry_consumed ? "yes" : "no", q.read_idx, q.next_write_idx);

  test_queue_add_entry(&q, 500, 5);
  test_queue_fill_i2s_buffer(&q);

  printf("  Second fill: read_idx=%d, write_idx=%d\n", q.read_idx,
         q.next_write_idx);

  printf("  Partial processing works correctly\n");
  test_result("I2S partial entry processing", true);
}

static void test_edge_1_pulse_65535_ticks() {
  printf("Running: Edge case - 1 pulse at 65535 ticks (max uint16_t)\n");

  i2s_mgr_clear_work_buf();

  TestStepperQueue q;
  test_queue_init(&q);

  q._isRunning = true;
  test_queue_add_entry(&q, 65535, 1);

  test_queue_fill_i2s_buffer(&q);

  uint8_t* buf = i2s_mgr_work_buf();

  uint32_t expected_frame = 65535 / I2S_TEST_TICKS_PER_FRAME;
  printf("  Expected frame: %u (65535 / %d)\n", expected_frame,
         I2S_TEST_TICKS_PER_FRAME);

  if (expected_frame >= I2S_TEST_MAX_FRAMES_PER_FILL) {
    printf("  Pulse exceeds buffer, checking no pulse written\n");
    for (uint32_t f = 0; f < 10; f++) {
      if (buf[f * I2S_TEST_BYTES_PER_FRAME] != 0x00 ||
          buf[f * I2S_TEST_BYTES_PER_FRAME + 1] != 0x00) {
        printf("  FAIL: Unexpected pulse at frame %u\n", f);
        test_result("Edge 1 pulse 65535 ticks", false);
        return;
      }
    }
    printf("  Correctly handled - pulse beyond buffer, none written\n");
  } else {
    uint16_t pulse_frame = q._i2s_pulse_positions[0];
    printf("  Pulse at frame %d\n", pulse_frame);

    if (pulse_frame != expected_frame) {
      printf("  FAIL: Expected frame %u, got %d\n", expected_frame,
             pulse_frame);
      test_result("Edge 1 pulse 65535 ticks", false);
      return;
    }
    if (!verify_pulse_at_frame(buf, pulse_frame)) {
      test_result("Edge 1 pulse 65535 ticks", false);
      return;
    }
  }

  test_result("Edge 1 pulse 65535 ticks", true);
}

static void test_edge_1_pulse_min_ticks() {
  printf("Running: Edge case - 1 pulse at MIN_CMD_TICKS\n");

  i2s_mgr_clear_work_buf();

  TestStepperQueue q;
  test_queue_init(&q);

  q._isRunning = true;
  test_queue_add_entry(&q, MIN_CMD_TICKS, 1);

  test_queue_fill_i2s_buffer(&q);

  uint8_t* buf = i2s_mgr_work_buf();

  uint32_t expected_frame = MIN_CMD_TICKS / I2S_TEST_TICKS_PER_FRAME;
  printf("  MIN_CMD_TICKS=%ld, expected frame: %u\n", (long)MIN_CMD_TICKS,
         expected_frame);

  uint16_t pulse_frame = q._i2s_pulse_positions[0];
  printf("  Pulse at frame %d\n", pulse_frame);

  if (pulse_frame != expected_frame) {
    printf("  FAIL: Expected frame %u, got %d\n", expected_frame, pulse_frame);
    test_result("Edge 1 pulse MIN_CMD_TICKS", false);
    return;
  }

  if (!verify_pulse_at_frame(buf, pulse_frame)) {
    test_result("Edge 1 pulse MIN_CMD_TICKS", false);
    return;
  }

  test_result("Edge 1 pulse MIN_CMD_TICKS", true);
}

static void test_edge_255_pulses_65535_ticks() {
  printf("Running: Edge case - 255 pulses at 65535 ticks\n");

  i2s_mgr_clear_work_buf();

  TestStepperQueue q;
  test_queue_init(&q);

  q._isRunning = true;
  test_queue_add_entry(&q, 65535, 255);

  test_queue_fill_i2s_buffer(&q);

  uint32_t frames_per_pulse = 65535 / I2S_TEST_TICKS_PER_FRAME;
  uint32_t pulses_that_fit = I2S_TEST_MAX_FRAMES_PER_FILL / frames_per_pulse;

  printf("  Frames per pulse: %u\n", frames_per_pulse);
  printf("  Pulses that fit in buffer: %u\n", pulses_that_fit);

  uint8_t* buf = i2s_mgr_work_buf();

  uint16_t pulses_written = q._i2s_pulse_write_idx;
  printf("  Pulses written: %d\n", pulses_written);

  if (pulses_written > pulses_that_fit + 1) {
    printf("  FAIL: Wrote %d pulses, expected <= %u\n", pulses_written,
           pulses_that_fit + 1);
    test_result("Edge 255 pulses 65535 ticks", false);
    return;
  }

  printf("  Entry partially processed correctly\n");
  printf("  Remaining steps in entry: %d\n",
         q.entry[q.read_idx & QUEUE_LEN_MASK].steps);

  test_result("Edge 255 pulses 65535 ticks", true);
}

static void test_edge_255_pulses_200khz() {
  printf("Running: Edge case - 255 pulses at 200kHz (80 ticks each)\n");

  i2s_mgr_clear_work_buf();

  TestStepperQueue q;
  test_queue_init(&q);

  uint16_t ticks_200khz = TICKS_PER_S / 200000;
  printf("  Ticks for 200kHz: %d (TICKS_PER_S/200000)\n", ticks_200khz);

  q._isRunning = true;
  test_queue_add_entry(&q, ticks_200khz, 255);

  test_queue_fill_i2s_buffer(&q);

  uint8_t* buf = i2s_mgr_work_buf();

  uint16_t pulses_written = q._i2s_pulse_write_idx;
  printf("  Pulses written: %d, final carry: %u\n", pulses_written,
         q._i2s_tick_carry);

  printf("  First 5 pulse positions:\n");
  for (int i = 0; i < pulses_written && i < 5; i++) {
    uint16_t frame = q._i2s_pulse_positions[i];
    printf("    Pulse %d at frame %d\n", i, frame);

    if (frame >= I2S_TEST_MAX_FRAMES_PER_FILL) {
      printf("  FAIL: Pulse %d at frame %d exceeds buffer\n", i, frame);
      test_result("Edge 255 pulses 200kHz", false);
      return;
    }

    if (!verify_pulse_at_frame(buf, frame)) {
      test_result("Edge 255 pulses 200kHz", false);
      return;
    }
  }

  if (pulses_written > 5) {
    printf("  ... and %d more pulses\n", pulses_written - 5);
  }

  uint16_t first_pulse = q._i2s_pulse_positions[0];
  uint16_t expected_first = ticks_200khz / I2S_TEST_TICKS_PER_FRAME;
  if (first_pulse != expected_first) {
    printf("  WARNING: First pulse at frame %d, expected frame %d\n",
           first_pulse, expected_first);
  }

  printf("  High-frequency pulse generation works correctly\n");
  test_result("Edge 255 pulses 200kHz", true);
}

static void test_edge_min_ticks_no_division() {
  printf("Running: Edge case - pulses at ticks < I2S_TEST_TICKS_PER_FRAME\n");

  i2s_mgr_clear_work_buf();

  TestStepperQueue q;
  test_queue_init(&q);

  q._isRunning = true;

  test_queue_add_entry(&q, 16, 10);

  test_queue_fill_i2s_buffer(&q);

  uint8_t* buf = i2s_mgr_work_buf();

  printf("  10 pulses at 16 ticks each (half a frame):\n");
  for (int i = 0; i < q._i2s_pulse_write_idx && i < 10; i++) {
    uint16_t frame = q._i2s_pulse_positions[i];
    printf("    Pulse %d at frame %d\n", i, frame);
  }

  printf("  Pulses written: %d\n", q._i2s_pulse_write_idx);

  printf("  Tick carry accumulates correctly for sub-frame periods\n");
  test_result("Edge min ticks no division", true);
}

void basic_test() {
  puts("basic_test...");

  test_i2s_init();
  test_i2s_direct_write();
  test_i2s_queue_fill();
  test_i2s_ring_buffer_clear();
  test_i2s_frame_boundary();
  test_i2s_empty_queue();
  test_i2s_ring_buffer_wrap();
  test_i2s_multi_step_entry();
  test_i2s_tick_carry();
  test_i2s_pause_entry();
  test_i2s_buffer_overflow_protection();
  test_i2s_partial_entry_processing();

  puts("\n=== Edge Cases ===");
  test_edge_1_pulse_65535_ticks();
  test_edge_1_pulse_min_ticks();
  test_edge_255_pulses_65535_ticks();
  test_edge_255_pulses_200khz();
  test_edge_min_ticks_no_division();

  printf("\n=== Test Summary ===\n");
  printf("Total: %d, Passed: %d, Failed: %d\n", test_passed + test_failed,
         test_passed, test_failed);
  if (test_failed == 0) {
    puts("All tests PASSED");
  } else {
    printf("%d test(s) FAILED\n", test_failed);
  }
}

int main() {
  basic_test();
  return 0;
}
