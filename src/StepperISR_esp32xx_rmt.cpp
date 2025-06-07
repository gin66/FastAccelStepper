#include "StepperISR.h"
#if defined(SUPPORT_ESP32_RMT)

// #define TEST_MODE

#include "fas_arch/test_probe.h"

// The following concept is in use:
//
//    The rmt buffer is split into two parts.
//    Each part will hold one command (or part of).
//
//    This way, the threshold interrupt occurs after the first part
//    and the end interrupt after the second part.
//    After the two parts an end marker is placed.
//
// Of these 32 bits, the low 16-bit entry is sent first and the high entry
// second.
// Every 16 bit entry defines with MSB the output level and the lower 15 bits
// the ticks.
//
// Important difference of esp32c3/esp32s3 (compared to esp32):
// Esp32c3 and Esp32s3 technical reference manuals state for relation 1:
//    3*T_APB + 5*T_RMT_CLK < period*T_CLK_DIV
// and relation 2, if period[14:0] == 0:
//    6*T_APB + 12*T_RMT_CLK < period*T_CLK_DIV
//
// Relation 2 is not applicable, because our end marker is 0x00000000
//
// Relation 1 means:
//    T_APB = 1/80MHz = 12.5ns
//    T_CLK_DIV = 1/16MHz = 62.5ns
//    T_RMT_CLK = 1/80MHz = 12.5ns
//    => period > (3*12.5ns + 5*12.5ns)/(62.5ns) = 1.6
//
void IRAM_ATTR rmt_fill_buffer(StepperQueue *q, bool fill_part_one,
                               uint32_t *data) {
  // Process command
  uint8_t rp = q->read_idx;
  struct queue_entry *e_curr = &q->entry[rp & QUEUE_LEN_MASK];
  if (e_curr->toggle_dir) {
    // the command requests dir pin toggle
    // This is ok only, if the ongoing command does not contain steps
    if (q->lastChunkContainsSteps) {
      // So we need a pause. change the finished read entry into a pause
      q->lastChunkContainsSteps = false;
      for (uint8_t i = 0; i < PART_SIZE; i++) {
        // two pauses à n ticks to achieve MIN_CMD_TICKS
        *data++ = 0x00010001 *
                  ((MIN_CMD_TICKS + 2 * PART_SIZE - 1) / (2 * PART_SIZE));
      }
      return;
    }
    // The ongoing command does not contain steps, so change dir here should be
    // ok. But we need the gpio_ll functions, which are always
    // inlined...hopefully.
    LL_TOGGLE_PIN(q->dirPin);
    // and delete the request
    e_curr->toggle_dir = 0;
  }

  uint8_t steps = e_curr->steps;
  uint16_t ticks = e_curr->ticks;
  //  if (steps != 0) {
  //  	PROBE_1_TOGGLE;
  //}
  if (steps == 0) {
    uint32_t last_entry;
    q->lastChunkContainsSteps = false;
    for (uint8_t i = 0; i < PART_SIZE - 1; i++) {
      // two pauses à 4 ticks
      *data++ = 0x00020002;
      ticks -= 4;
    }
    uint16_t ticks_l = ticks >> 1;
    uint16_t ticks_r = ticks - ticks_l;
    last_entry = ticks_l;
    last_entry <<= 16;
    last_entry |= ticks_r;
    *data++ = last_entry;
  } else {
    q->lastChunkContainsSteps = true;
    if (ticks == 0xffff) {
      // special treatment for this case, because an rmt entry can only cover up
      // to 0xfffe ticks every step must be minimum split into two rmt entries,
      // so at max PART/2 steps can be done.
      if (steps < PART_SIZE / 2) {
        for (uint8_t i = 1; i < steps; i++) {
          // steps-1 iterations
          *data++ = 0x40007fff | 0x8000;
          *data++ = 0x20002000;
        }
        *data++ = 0x40007fff | 0x8000;
        uint16_t delta = PART_SIZE - 2 * steps;
        delta <<= 5;
        *data++ = 0x20002000 - delta;
        // 2*(steps - 1) + 1 already stored => 2*steps - 1
        // and after this for loop one entry added => 2*steps
        for (uint8_t i = 2 * steps; i < PART_SIZE; i++) {
          *data++ = 0x00100010;
        }
        steps = 0;
      } else {
        steps -= PART_SIZE / 2;
        for (uint8_t i = 0; i < PART_SIZE / 2; i++) {
          *data++ = 0x40007fff | 0x8000;
          *data++ = 0x20002000;
        }
      }
    } else {
      uint8_t steps_to_do = steps;
      if (steps > PART_SIZE) {
        steps_to_do = PART_SIZE;
        if (steps_to_do > PART_SIZE) {
          steps_to_do >>= 1;
        }
      }
      // deduct this buffer's step from total
      steps -= steps_to_do;
      if (steps_to_do < PART_SIZE) {
        // We need to fill up the partition.
        // Worst case would be one step in the buffer.
        // In order to get threshold interrupt early enough,
        // the first step should be stretched to maximum.
        // The minimum period is 80ticks @200kHz.
        // Minimum command time is 3200 ticks.
        // Consequently, 80 ticks will come with minimum 40 steps.
        // Split into two rmt parts, so 20 steps each.
        // For PART_SIZE 23 or 31, minimum period is 2*2*PART_SIZE = 92 or
        // 124ticks. A single step with 80ticks cannot be stretched to
        // PART_SIZE.
        // => we need to stretch eventually two steps.
        uint8_t i = 0;
        while (true) {
          uint8_t extend_to_i = PART_SIZE - steps_to_do;  // extend_to_i >= 1
          // if steps_to_do = PART_SIZE-1, then extend_to_i = 1
          if (i >= extend_to_i) {
            // we have already extended enough
            break;
          }
          steps_to_do--;  // one step less to do
          uint16_t ticks_high = ticks >> 1;
          uint16_t ticks_low = ticks - ticks_high;
          while (true) {
            // need i+1, because low entry is still needed
            if ((ticks_high >= 8) && (i + 1 < extend_to_i)) {
              *data++ = 0x80028002;
              i++;
              ticks_high -= 4;
            } else {
              uint32_t entry = ticks_high>>1;
              entry <<= 16;
              entry |= ticks_high - (ticks_high>>1);
              *data++ = 0x80008000 | entry;
              i++;
              break;
            }
          }
          while (true) {
            if ((ticks_low >= 8) && (i < extend_to_i)) {
              *data++ = 0x00020002;
              i++;
              ticks_low -= 4;
            } else {
              // #325: esp32c3 has frozen with one step and ticks ~38600
              //       now the last entry is balanced instead of long pause and then 2 ticks
              uint32_t entry = ticks_low>>1;
              entry <<= 16;
              entry |= ticks_low - (ticks_low>>1);
              *data++ = entry;
              i++;
              break;
            }
          }
        }
        // Now add remaining steps, if any
        if (steps_to_do > 0) {
          uint16_t ticks_high = ticks >> 1;
          uint16_t ticks_low = ticks - ticks_high;
          // ticks_l <= ticks_r
          uint32_t rmt_entry = ticks_low;
          rmt_entry <<= 16;
          rmt_entry |= ticks_high | 0x8000;  // with step
          for (uint8_t i = 1; i <= steps_to_do; i++) {
            *data++ = rmt_entry;
          }
        }
      } else {
        // either >= 2*PART_SIZE or = PART_SIZE
        // every entry one step
        uint16_t ticks_high = ticks >> 1;
        uint16_t ticks_low = ticks - ticks_high;
        uint32_t rmt_entry = ticks_low;
        rmt_entry <<= 16;
        rmt_entry |= ticks_high | 0x8000;  // with step
        for (uint8_t i = 0; i < PART_SIZE; i++) {
          *data++ = rmt_entry;
        }
      }
    }
  }
#if defined(SUPPORT_ESP32_RMT_TICK_LOST)
  // No tick lost mentioned for esp32s3 and esp32c3
  if (!fill_part_one) {
    // Note: When enabling the continuous transmission mode by setting
    // RMT_REG_TX_CONTI_MODE, the transmitter will transmit the data on the
    // channel continuously, that is, from the first byte to the last one,
    // then from the first to the last again, and so on. In this mode, there
    // will be an idle level lasting one clk_div cycle between N and N+1
    // transmissions.
    data--;
    *data -= 1;
  }
#endif

  // Data is complete
  if (steps == 0) {
    // The command has been completed
    if (e_curr->repeat_entry == 0) {
      q->read_idx = rp + 1;
      PROBE_4_TOGGLE;
    }
  } else {
    e_curr->steps = steps;
  }
  return;
}
#if (ESP_IDF_VERSION_MAJOR == 4)
void IRAM_ATTR rmt_apply_command(StepperQueue *q, bool fill_part_one,
                                 uint32_t *data) {
  if (!fill_part_one) {
    data += PART_SIZE;
  }
  uint8_t rp = q->read_idx;
  if (rp == q->next_write_idx) {
    // no command in queue
    if (fill_part_one) {
      q->lastChunkContainsSteps = false;
      for (uint8_t i = 0; i < PART_SIZE; i++) {
        // make a pause with approx. 1ms
        //    258 ticks * 2 * 31 = 15996 @ 16MHz
        //    347 ticks * 2 * 23 = 15962 @ 16MHz
        *data++ = 0x00010001 * (16000 / 2 / PART_SIZE);
      }
    } else {
      q->stop_rmt(false);
    }
    return;
  }
  rmt_fill_buffer(q, fill_part_one, data);
}
#endif

#endif
