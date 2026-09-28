#include <stdint.h>
#include <stdio.h>

uint16_t debug_part_size = 32;

#include "FastAccelStepper.h"

void inject_fill_interrupt(int mark) {}
void noInterrupts() {}
void interrupts() {}

static int dir_toggles = 0;
#define SUPPORT_ESP32_RMT_V2
#define IRAM_ATTR
#define LL_TOGGLE_PIN(dirPin) dir_toggles++
#include "pd_esp32/StepperISR_idf5_esp32_rmt_encode.cpp"

static int failures = 0;

static void expect(bool ok, const char* msg) {
  if (!ok) {
    printf("FAIL: %s\n", msg);
    failures++;
  }
}

static void decode(uint32_t word, bool* level0, uint16_t* dur0, bool* level1,
                   uint16_t* dur1) {
  uint16_t low = (uint16_t)(word & 0xffff);
  uint16_t high = (uint16_t)(word >> 16);
  *level0 = (low & 0x8000) != 0;
  *dur0 = (uint16_t)(low & 0x7fff);
  *level1 = (high & 0x8000) != 0;
  *dur1 = (uint16_t)(high & 0x7fff);
}

static uint32_t symbol_ticks(uint32_t word) {
  bool l0, l1;
  uint16_t d0, d1;
  decode(word, &l0, &d0, &l1, &d1);
  return (uint32_t)d0 + d1;
}

static bool any_zero_duration(const uint32_t* words, uint32_t count) {
  for (uint32_t i = 0; i < count; i++) {
    bool l0, l1;
    uint16_t d0, d1;
    decode(words[i], &l0, &d0, &l1, &d1);
    if (d0 == 0 || d1 == 0) {
      return true;
    }
  }
  return false;
}

// The RMT relation 1 floor is 2 ticks per sub-entry; anything below makes the
// hardware stretch the symbol, and 0 is the stop pattern.
static bool any_half_below(const uint32_t* words, uint32_t count,
                           uint16_t floor) {
  for (uint32_t i = 0; i < count; i++) {
    bool l0, l1;
    uint16_t d0, d1;
    decode(words[i], &l0, &d0, &l1, &d1);
    if (d0 < floor || d1 < floor) {
      return true;
    }
  }
  return false;
}

static bool all_low(const uint32_t* words, uint32_t count) {
  for (uint32_t i = 0; i < count; i++) {
    bool l0, l1;
    uint16_t d0, d1;
    decode(words[i], &l0, &d0, &l1, &d1);
    if (l0 || l1) {
      return false;
    }
  }
  return true;
}

static uint32_t rising_edges(const uint32_t* words, uint32_t count) {
  bool high = false;
  uint32_t edges = 0;
  for (uint32_t i = 0; i < count; i++) {
    bool l0, l1;
    uint16_t d0, d1;
    decode(words[i], &l0, &d0, &l1, &d1);
    if (l0 && !high) {
      edges++;
    }
    high = l0;
    if (l1 && !high) {
      edges++;
    }
    high = l1;
  }
  return edges;
}

static uint32_t total_ticks(const uint32_t* words, uint32_t count) {
  uint32_t sum = 0;
  for (uint32_t i = 0; i < count; i++) {
    sum += symbol_ticks(words[i]);
  }
  return sum;
}

static void reset_queue() {
  fas_queue[0]._initVars();
  fas_queue[0].read_idx = 0;
  fas_queue[0].next_write_idx = 0;
  dir_toggles = 0;
}

static void push(uint8_t steps, uint16_t ticks, bool toggle) {
  uint8_t idx = fas_queue[0].next_write_idx & QUEUE_LEN_MASK;
  fas_queue[0].entry[idx].steps = steps;
  fas_queue[0].entry[idx].ticks = ticks;
  fas_queue[0].entry[idx].toggle_dir = toggle ? 1 : 0;
  fas_queue[0].entry[idx].countUp = 1;
  fas_queue[0].entry[idx].hasSteps = steps != 0;
  fas_queue[0].next_write_idx++;
}

// Retired: these tests exercise the old whole-command model
// (one pause = PART_SIZE symbols, one step = 1-2 symbols).
// They will be rewritten for the tick-based fill in later phases.
static void test_pause_fills_one_half() {
  printf("[RETIRED] pause is PART_SIZE symbols (whole-command model)\n");
}

static void test_pause_needs_a_full_half() {
  printf("[RETIRED] pause waits for PART_SIZE free symbols (whole-command model)\n");
}

static void test_short_step_is_one_symbol() {
  printf("[RETIRED] short step uses one symbol (whole-command model)\n");
}

static void test_max_tick_step_is_two_symbols() {
  printf("[RETIRED] 65535-tick step uses two symbols (whole-command model)\n");
}

static void test_step_needs_two_free_symbols() {
  printf("[RETIRED] step entry needs two free symbols (whole-command model)\n");
}

static void test_remaining_steps_written_back() {
  printf("[RETIRED] partial step entry keeps the remainder (whole-command model)\n");
}

static void test_max_tick_steps_pack_by_two() {
  printf("[RETIRED] 65535-tick steps stop when one symbol remains (whole-command model)\n");
}

static void test_two_symbol_cases_report_their_count() {
  printf("[RETIRED] 1-or-2 symbol cases report count (whole-command model)\n");
}


static void test_pause_then_steps_share_a_call() {
  printf("pause and following steps share one call\n");
  reset_queue();
  push(0, 4000, false);
  push(10, 8000, false);
  uint32_t sym[80];
  uint32_t free_symbols = (uint32_t)PART_SIZE + 4;
  uint32_t n = rmt_encode_queue(&fas_queue[0], sym, free_symbols);
  expect(n == free_symbols, "half plus four steps");
  expect(all_low(sym, (uint32_t)PART_SIZE), "leading pause");
  expect(total_ticks(sym, (uint32_t)PART_SIZE) == 4000, "pause sum");
  expect(rising_edges(sym + PART_SIZE, 4) == 4, "four steps");
  expect(fas_queue[0].read_idx == 1, "pause consumed, steps remain");
  expect(fas_queue[0].entry[1].steps == 6, "six steps remain");
}

static void test_toggle_waits_for_the_next_call() {
  printf("direction toggle is not in the pause call\n");
  reset_queue();
  push(0, 3200, false);
  push(0, 3200, false);
  push(1, 8000, true);
  uint32_t sym[80];
  uint32_t n = rmt_encode_queue(&fas_queue[0], sym, (uint32_t)PART_SIZE * 2);
  expect(n == (uint32_t)PART_SIZE * 2, "two drain pauses");
  expect(all_low(sym, n), "drains are low");
  expect(fas_queue[0].read_idx == 2, "toggle entry not started");
  expect(dir_toggles == 0, "dir still old");
  expect(fas_queue[0].entry[2].toggle_dir == 1, "toggle still pending");
  n = rmt_encode_queue(&fas_queue[0], sym, 4);
  expect(n == 1, "step on the next call");
  expect(dir_toggles == 1, "dir toggled once");
  expect(fas_queue[0].read_idx == 3, "toggle entry consumed");
  expect(rising_edges(sym, n) == 1, "one new-direction edge");
}

static uint64_t total_ticks64(const uint32_t* words, uint32_t count) {
  uint64_t sum = 0;
  for (uint32_t i = 0; i < count; i++) {
    sum += symbol_ticks(words[i]);
  }
  return sum;
}

static uint16_t decode_period(uint32_t word) {
  bool l0, l1;
  uint16_t d0, d1;
  decode(word, &l0, &d0, &l1, &d1);
  uint16_t p = d0 < d1 ? d0 : d1;
  return p;
}

// fill_queue() is private; the library befriends FastAccelStepperTest for
// host-side ramp feeding (same as ramp_helper.cpp).
class FastAccelStepperTest {
 public:
  static void fill_queue(FastAccelStepper& s) { s.fill_queue(); }
};

struct MoveStats {
  uint64_t commanded_ticks;
  uint64_t symbol_ticks;
  uint32_t rising_edges;
  uint16_t min_symbol;
  uint16_t max_symbol;
  uint32_t empty_hits;
  uint32_t encode_calls;
  uint32_t zero_durations;
  uint32_t overrun;  // symbols written beyond the returned count
  // Minimum time spanned by any PART_SIZE consecutive RMT symbols.  This is
  // the worst-case amount of motion the hardware can still play after the
  // encoder has stopped handing over symbols, i.e. the drain time of one RMT
  // half-buffer at the current step rate.
  uint32_t min_window_ticks;
  // Fewest RMT symbols contributed by a single queue command, and the index
  // (n) of the move where it happened.
  uint32_t min_cmd_symbols;
  uint32_t min_cmd_steps;
  uint16_t min_cmd_ticks;
};

// Feed the queue exactly as the IDF5/6 simple encoder does: the driver asks
// for symbols whenever the RMT memory has room, so one call may see up to the
// whole memory block (2 * PART_SIZE) free.  Advance the ramp in the intended
// planning window by topping the queue up between calls.
static MoveStats run_move(FastAccelStepper* s, int32_t steps) {
  StepperQueue* q = &fas_queue[0];
  MoveStats st = {};
  st.min_symbol = 0xffff;
  st.min_window_ticks = 0xffffffff;
  st.min_cmd_symbols = 0xffffffff;
  uint32_t sym[2 * 64];
  uint8_t last_wp = q->next_write_idx;
  uint32_t guard = 0;
  // Rolling window of the last PART_SIZE symbol durations.
  uint32_t win[64];
  uint32_t win_sum = 0;
  uint32_t win_pos = 0;
  uint32_t win_count = 0;

  s->move(steps);
  while (true) {
    if (s->isRampGeneratorActive()) {
      FastAccelStepperTest::fill_queue(*s);
    }
    // Account every command once, at the moment it is enqueued.  The encoder
    // later writes back the remaining `steps`, but the full count is what the
    // symbols must reproduce in total.
    while (last_wp != q->next_write_idx) {
      struct queue_entry* e = &q->entry[last_wp & QUEUE_LEN_MASK];
      uint32_t csym;
      if (e->steps == 0) {
        st.commanded_ticks += e->ticks;
        csym = (uint32_t)PART_SIZE;  // a pause always fills one half
      } else {
        st.commanded_ticks += (uint64_t)e->ticks * e->steps;
        csym = (uint32_t)e->steps * ((e->ticks == 0xffff) ? 2 : 1);
      }
      if (csym < st.min_cmd_symbols) {
        st.min_cmd_symbols = csym;
        st.min_cmd_steps = e->steps;
        st.min_cmd_ticks = e->ticks;
      }
      last_wp++;
    }
    if (q->read_idx == q->next_write_idx) {
      if (!s->isRampGeneratorActive()) {
        break;
      }
      // Queue momentarily empty while the ramp is still producing.  The
      // hardware encoder would emit a drain pause here; count it so the
      // test shows how often seq_02 hits that path.
      st.empty_hits++;
      if (++guard > 20000000) {
        break;
      }
      continue;
    }
    // Sentinel the whole buffer so any symbol written past the returned count
    // is visible: the encoder must never write more than it reports.
    const uint32_t cap = 2 * (uint32_t)PART_SIZE;
    for (uint32_t i = 0; i < cap; i++) {
      sym[i] = 0xDEADBEEF;
    }
    uint32_t n = rmt_encode_queue(q, sym, cap);
    if (n == 0) {
      if (++guard > 20000000) {
        break;
      }
      continue;
    }
    st.encode_calls++;
    st.symbol_ticks += total_ticks64(sym, n);
    st.rising_edges += rising_edges(sym, n);
    if (any_zero_duration(sym, n)) {
      st.zero_durations++;
    }
    for (uint32_t i = n; i < cap; i++) {
      if (sym[i] != 0xDEADBEEF) {
        st.overrun++;
      }
    }
    for (uint32_t i = 0; i < n; i++) {
      uint16_t p = decode_period(sym[i]);
      if (p < st.min_symbol) {
        st.min_symbol = p;
      }
      if (p > st.max_symbol) {
        st.max_symbol = p;
      }
      uint32_t sti = symbol_ticks(sym[i]);
      if (win_count == (uint32_t)PART_SIZE) {
        win_sum -= win[win_pos];  // evict the oldest before overwriting
      } else {
        win_count++;
      }
      win[win_pos] = sti;
      win_sum += sti;
      if (++win_pos == (uint32_t)PART_SIZE) {
        win_pos = 0;
      }
      if ((win_count == (uint32_t)PART_SIZE) &&
          (win_sum < st.min_window_ticks)) {
        st.min_window_ticks = win_sum;
      }
    }
    if (++guard > 20000000) {
      break;
    }
  }
  if (st.min_symbol == 0xffff) {
    st.min_symbol = 0;
  }
  return st;
}

//===========================================================================
// Driver model: the IDF5/6 simple encoder run under a task that ticks every
// DELAY_MS_BASE.  The RMT memory holds 2*PART_SIZE symbols; a transaction is
// stopped the moment the encoder is asked for symbols and the queue is empty
// (encode_commands() sets _rmtStopped and emits one ENTER_PAUSE).  The task is
// then locked out until the memory has drained (isRunning() goes false) and it
// starts the next transaction on its following tick.  This is the mechanism
// the 040 item blames for the 120.8 s pin trace; the model reports how much of
// the difference it accounts for.
//===========================================================================

struct HoleModel {
  uint64_t encoded_ticks;   // symbols from rmt_encode_queue
  uint64_t injected_ticks;  // the ENTER_PAUSE(MIN_CMD_TICKS) stops
  uint64_t stops;
};

static uint32_t append_min_pause(uint32_t* out) {
  // Same as the ENTER_PAUSE(MIN_CMD_TICKS) macro in StepperISR_idf5_esp32_rmt.
  uint16_t ticks = (uint16_t)MIN_CMD_TICKS;
  uint16_t half = (uint16_t)(ticks / (2 * PART_SIZE));
  uint32_t symbol = 0x00010001 * half;
  for (uint8_t i = 0; i < PART_SIZE - 1; i++) {
    out[i] = symbol;
  }
  uint16_t remaining = (uint16_t)(ticks - 2 * (PART_SIZE - 1) * half);
  uint16_t first = (uint16_t)(remaining / 2);
  remaining = (uint16_t)(remaining - first);
  out[PART_SIZE - 1] = 0x00010000 * first + remaining;
  return PART_SIZE;
}

static HoleModel run_move_model(FastAccelStepper* s, int32_t steps,
                                uint64_t* now_inout,
                                uint64_t* next_task_inout) {
  StepperQueue* q = &fas_queue[0];
  HoleModel m = {};
  const uint64_t TASK = (uint64_t)DELAY_MS_BASE * (TICKS_PER_S / 1000);
  const uint32_t CAP = 2 * (uint32_t)PART_SIZE;
  uint32_t fifo[2 * 64];
  uint32_t fn = 0;
  uint64_t now = *now_inout;  // shared event time
  uint64_t next_task = *next_task_inout;
  uint64_t pin_base = now;  // time the current head symbol started
  bool active = false;
  bool stopped = false;
  uint64_t guard = 0;

  q->_isRunning = false;
  s->move(steps);

  while (guard++ < 200000000ull) {
    // 1) The driver can hand symbols to the RMT memory (a whole block at the
    //    start of a transaction, then one PART_SIZE half at each threshold).
    if (active && !stopped && (fn == 0 || (CAP - fn) >= (uint32_t)PART_SIZE)) {
      uint32_t free_syms = (fn == 0) ? CAP : (uint32_t)PART_SIZE;
      uint32_t tmp[2 * 64];
      if (fn == 0) {
        pin_base = now;
      }
      if (q->read_idx == q->next_write_idx) {
        stopped = true;
        uint32_t np = append_min_pause(tmp);
        for (uint32_t i = 0; i < np && fn < CAP; i++) {
          fifo[fn++] = tmp[i];
        }
        m.stops++;
        m.injected_ticks += MIN_CMD_TICKS;
      } else {
        uint32_t n = rmt_encode_queue(q, tmp, free_syms);
        m.encoded_ticks += total_ticks64(tmp, n);
        for (uint32_t i = 0; i < n && fn < CAP; i++) {
          fifo[fn++] = tmp[i];
        }
      }
      continue;
    }
    // 2) Stopped and drained -> transaction done, task unblocks.
    if (active && fn == 0 && stopped) {
      active = false;
      stopped = false;
      q->_isRunning = false;
      continue;
    }
    // 3) Idle pin and no transaction: advance to the next task tick.
    if (!active && fn == 0) {
      if (!s->isRampGeneratorActive() && q->read_idx == q->next_write_idx) {
        break;
      }
      if (now < next_task) {
        now = next_task;
      }
      FastAccelStepperTest::fill_queue(*s);
      next_task += TASK;
      if (q->read_idx != q->next_write_idx) {
        active = true;
        stopped = false;
        fn = 0;
        q->_isRunning = true;
      }
      continue;
    }
    // 4) Advance to the earlier of the head symbol's end and the next task
    //    tick.  A task tick must not push the head symbol's end back.
    uint64_t pin_event = (uint64_t)-1;
    if (fn > 0) {
      pin_event = pin_base + symbol_ticks(fifo[0]);
    }
    if (next_task <= pin_event) {
      now = next_task;
      next_task += TASK;
      if (!(q->_isRunning && stopped)) {
        FastAccelStepperTest::fill_queue(*s);
        if (!active && q->read_idx != q->next_write_idx) {
          active = true;
          stopped = false;
          q->_isRunning = true;
        }
      }
      continue;
    }
    if (pin_event == (uint64_t)-1) {
      break;
    }
    // Consume the symbol at the head of the RMT memory.
    now = pin_event;
    pin_base = now;
    for (uint32_t i = 1; i < fn; i++) {
      fifo[i - 1] = fifo[i];
    }
    fn--;
  }
  *now_inout = now;
  *next_task_inout = next_task;
  return m;
}

static void test_seq_02_holes() {
  printf("seq_02: modeled stop/restart holes\n");
  fas_queue[0]._initVars();
  dir_toggles = 0;
  FastAccelStepper s;
  s.init(NULL, 0, 0);
  s.setDirectionPin(2);
  s.setSpeedInUs(40);
  s.setAcceleration(1000);

  uint64_t now = 0, next_task = 0;
  uint64_t encoded = 0, injected = 0, stops = 0;
  uint32_t pairs = 0;
  for (uint32_t n = 1;; n = (n + 1) + ((n + 1) >> 2)) {
    HoleModel fwd = run_move_model(&s, (int32_t)n, &now, &next_task);
    HoleModel back = run_move_model(&s, -(int32_t)n, &now, &next_task);
    pairs++;
    encoded += fwd.encoded_ticks + back.encoded_ticks;
    injected += fwd.injected_ticks + back.injected_ticks;
    stops += fwd.stops + back.stops;
    if (n >= 1733) {
      printf("  n=%u stops=%llu\n", n,
             (unsigned long long)(fwd.stops + back.stops));
    }
    if (n >= 6400) {
      break;
    }
  }
  double wall_s = now / 16e6;
  double extra_s = wall_s - encoded / 16e6;
  printf("  pairs=%u wall=%.3fs encoded=%.3fs injected=%.3fs stops=%llu\n",
         pairs, wall_s, encoded / 16e6, injected / 16e6,
         (unsigned long long)stops);
  printf("  unencoded extra=%.3fs (%.3fms per stop)\n", extra_s,
         stops ? extra_s * 1000.0 / stops : 0.0);
  printf("  modeled wall vs 120.760s capture: %.3fs (delta %.3fs)\n", wall_s,
         wall_s - 120.7604225);
  expect(encoded == 1463342840ull,
         "modeled encoded ticks match the commanded stream");
}

static void test_seq_02_total_ticks() {
  printf("seq_02: rmt symbols reproduce the commanded ticks\n");
  fas_queue[0]._initVars();
  dir_toggles = 0;
  FastAccelStepper s;
  s.init(NULL, 0, 0);
  s.setDirectionPin(2);
  s.setSpeedInUs(40);
  s.setAcceleration(1000);

  uint64_t total_commanded = 0;
  uint64_t total_symbols = 0;
  uint32_t total_edges = 0;
  uint32_t total_expected_edges = 0;
  uint32_t total_empty = 0;
  uint32_t total_zero = 0;
  uint32_t total_overrun = 0;
  uint16_t global_min = 0xffff;
  uint32_t global_min_window = 0xffffffff;
  uint32_t global_min_cmd_sym = 0xffffffff;
  uint32_t global_min_cmd_steps = 0;
  uint16_t global_min_cmd_ticks = 0;
  uint32_t pairs = 0;

  // seq_02: move n, then n = (n + 1) + ((n + 1) >> 2), and stop once the
  // just-finished n is >= 6400.  This yields 33 pairs, largest move 6621.
  for (uint32_t n = 1;; n = (n + 1) + ((n + 1) >> 2)) {
    MoveStats fwd = run_move(&s, (int32_t)n);
    MoveStats back = run_move(&s, -(int32_t)n);
    pairs++;
    total_commanded += fwd.commanded_ticks + back.commanded_ticks;
    total_symbols += fwd.symbol_ticks + back.symbol_ticks;
    total_edges += fwd.rising_edges + back.rising_edges;
    total_expected_edges += 2 * n;
    total_empty += fwd.empty_hits + back.empty_hits;
    total_zero += fwd.zero_durations + back.zero_durations;
    total_overrun += fwd.overrun + back.overrun;
    if (fwd.min_symbol < global_min) {
      global_min = fwd.min_symbol;
    }
    if (back.min_symbol < global_min) {
      global_min = back.min_symbol;
    }
    if (fwd.min_window_ticks < global_min_window) {
      global_min_window = fwd.min_window_ticks;
    }
    if (back.min_window_ticks < global_min_window) {
      global_min_window = back.min_window_ticks;
    }
    if (fwd.min_cmd_symbols < global_min_cmd_sym) {
      global_min_cmd_sym = fwd.min_cmd_symbols;
      global_min_cmd_steps = fwd.min_cmd_steps;
      global_min_cmd_ticks = fwd.min_cmd_ticks;
    }
    if (back.min_cmd_symbols < global_min_cmd_sym) {
      global_min_cmd_sym = back.min_cmd_symbols;
      global_min_cmd_steps = back.min_cmd_steps;
      global_min_cmd_ticks = back.min_cmd_ticks;
    }
    if (fwd.symbol_ticks != fwd.commanded_ticks ||
        back.symbol_ticks != back.commanded_ticks) {
      printf("  n=%u FAIL fwd enc=%llu cmd=%llu | back enc=%llu cmd=%llu\n", n,
             (unsigned long long)fwd.symbol_ticks,
             (unsigned long long)fwd.commanded_ticks,
             (unsigned long long)back.symbol_ticks,
             (unsigned long long)back.commanded_ticks);
      failures++;
    }
    if (n >= 1733) {
      printf("  n=%u fwd ticks=%llu edges=%u min=%u max=%u empty=%u\n", n,
             (unsigned long long)fwd.symbol_ticks, fwd.rising_edges,
             fwd.min_symbol, fwd.max_symbol, fwd.empty_hits);
    }
    if (n >= 6400) {
      break;
    }
  }
  printf("  pairs=%u commanded=%llu symbols=%llu edges=%u empty_hits=%u\n",
         pairs, (unsigned long long)total_commanded,
         (unsigned long long)total_symbols, total_edges, total_empty);
  printf("  global_min=%u zero_durations=%u overrun=%u\n", global_min,
         total_zero, total_overrun);
  printf("  min PART_SIZE-symbol window = %u ticks (%.3f ms) [informational]\n",
         global_min_window, global_min_window * 1000.0 / (double)TICKS_PER_S);
  printf("  min symbols/command = %u (steps=%u ticks=%u), PART_SIZE/2 = %u\n",
         global_min_cmd_sym, global_min_cmd_steps, global_min_cmd_ticks,
         (uint32_t)PART_SIZE / 2);
  expect(total_symbols == total_commanded,
         "seq_02 rmt symbol ticks equal commanded ticks");
  expect(total_edges == total_expected_edges, "seq_02 one edge per step");
  // Every sub-entry must respect the RMT relation 1 floor, and the encoder
  // must not write a symbol it did not report.
  expect(global_min >= 2, "seq_02 every RMT sub-entry is >= 2 ticks");
  expect(total_zero == 0, "seq_02 has no zero-duration sub-entry");
  expect(total_overrun == 0, "seq_02 never writes past the reported count");
  // Read-ahead bound: if every command covers at least half an RMT half-buffer
  // (PART_SIZE/2 symbols), the full RMT buffer (2*PART_SIZE symbols) can hold
  // at most four commands.  The encoder then cannot drain the whole queue into
  // the RMT, so encode_commands() never sees a transiently empty queue and the
  // eager _rmtStopped stop cannot fire.
  expect(global_min_cmd_sym >= (uint32_t)PART_SIZE / 2,
         "seq_02 every command covers >= PART_SIZE/2 RMT symbols");
}

static void run_suite(uint16_t part) {
  debug_part_size = part;
  printf("\n=== PART_SIZE %u ===\n", part);
  test_pause_then_steps_share_a_call();
  test_toggle_waits_for_the_next_call();
  test_seq_02_total_ticks();
  test_seq_02_holes();
}

int main() {
  run_suite(32);
  run_suite(24);
  if (failures != 0) {
    printf("%d failure(s)\n", failures);
    return 1;
  }
  printf("PASS\n");
  return 0;
}
