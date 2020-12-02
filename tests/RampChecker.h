class RampChecker {
 public:
  uint64_t total_ticks;
  uint32_t last_dt;
  uint32_t min_dt;
  bool increase_ok;
  bool flat_ok;
  bool decrease_ok;
  bool first;
  uint32_t accelerate_till;
  uint32_t coast_till;
  uint32_t pos;
  uint32_t ticks_since_last_step = 0xffffffff;

RampChecker() {
  total_ticks = 0;
  last_dt = ~0;
  min_dt = ~0;
  first = true;
  increase_ok = true;
  decrease_ok = false;
  coast_till = 0;
  accelerate_till = 0;
  pos = 0;
}
void check_section(struct queue_entry *e) {
  uint8_t steps = e->steps;
  if (!first) {
    assert(!e->toggle_dir);
  }
  if (steps == 0) {
    // Just a pause
    ticks_since_last_step += e->ticks;
    total_ticks += e->ticks;
    printf("process pause %d\n", e->ticks);
    return;
  }
  pos += steps;
  uint32_t start_dt = e->ticks;
  total_ticks += steps * start_dt;
  if (ticks_since_last_step < 0xffff0000) {
    start_dt += ticks_since_last_step;
  } else {
    start_dt = ticks_since_last_step;
  }

  ticks_since_last_step = 0;
  min_dt = min(min_dt, start_dt);
  float accel = 0;
  if (!first) {
    accel = (16000000.0 / start_dt - 16000000.0 / last_dt) /
            (1.0 / 16000000.0 * start_dt);
  }
  printf(
      "process command in ramp checker @%.6fs: steps = %d last = %d start = %d "
      " min_dt "
      "= %d   accel=%.6f inc=%s dec=%s\n",
      total_ticks / 16000000.0, steps, last_dt, start_dt, min_dt, accel,
      increase_ok ? "ALLOW" : "NO", decrease_ok ? "ALLOW" : "NO");

  assert(steps * start_dt >= 0);

  if (last_dt > start_dt) {
    assert(increase_ok);
    accelerate_till = total_ticks;
    decrease_ok = true;
  } else if (last_dt < start_dt) {
    if (increase_ok) {
      coast_till = total_ticks;
    }
    assert(decrease_ok);
    increase_ok = false;
  }

  last_dt = start_dt;

  first = false;
}
};
