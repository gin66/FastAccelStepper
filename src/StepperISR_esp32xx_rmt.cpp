#include "StepperISR.h"
#if defined(SUPPORT_ESP32_RMT)

// #define TEST_MODE

#include "fas_arch/test_probe.h"

// The following concept is in use:
//
//    The buffer of 64Bytes is split into two parts à 31 words.
//    Each part will hold one command (or part of).
//    After the 2*31 words an end marker is placed.
//    The threshold is set to 31.
//
//    This way, the threshold interrupt occurs after the first part
//    and the end interrupt after the second part.
//
// Of these 32 bits, the low 16-bit entry is sent first and the high entry
// second.
// Every 16 bit entry defines with MSB the output level and the lower 15 bits
// the ticks.
#define PART_SIZE (RMT_SIZE / 2 - 1)

void IRAM_ATTR rmt_apply_command(StepperQueue *q, bool fill_part_one,
                                    uint32_t *data) {
  if (!fill_part_one) {
    data += PART_SIZE;
  }
  uint8_t rp = q->read_idx;
  if (rp == q->next_write_idx) {
    // no command in queue
    if (fill_part_one) {
      q->bufferContainsSteps[0] = false;
      for (uint8_t i = 0; i < PART_SIZE; i++) {
        // make a pause with approx. 1ms
        //    258 ticks * 2 * 31 = 15996 @ 16MHz
        *data++ = 0x01020102;
      }
    } else {
      q->stop_rmt(false);
    }
    return;
  }
  // Process command
  struct queue_entry *e_curr = &q->entry[rp & QUEUE_LEN_MASK];

  if (e_curr->toggle_dir) {
    // the command requests dir pin toggle
    // This is ok only, if the ongoing command does not contain steps
    if (q->bufferContainsSteps[fill_part_one ? 1 : 0]) {
      // So we need a pause. change the finished read entry into a pause
      q->bufferContainsSteps[fill_part_one ? 0 : 1] = false;
      for (uint8_t i = 0; i < PART_SIZE; i++) {
        // two pauses à n ticks to achieve MIN_CMD_TICKS
        *data++ = 0x00010001 * ((MIN_CMD_TICKS + 61) / 62);
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
  //  	PROBE_2_TOGGLE;
  //}
  uint32_t last_entry;
  if (steps == 0) {
    q->bufferContainsSteps[fill_part_one ? 0 : 1] = false;
    for (uint8_t i = 0; i < PART_SIZE - 1; i++) {
      // two pauses à 3 ticks. the 2 for debugging
      *data++ = 0x00010002;
      ticks -= 3;
    }
    uint16_t ticks_l = ticks >> 1;
    uint16_t ticks_r = ticks - ticks_l;
    last_entry = ticks_l;
    last_entry <<= 16;
    last_entry |= ticks_r;
  } else {
    q->bufferContainsSteps[fill_part_one ? 0 : 1] = true;
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
        for (uint8_t i = 2 * steps; i < PART_SIZE - 1; i++) {
          *data++ = 0x00100010;
        }
        last_entry = 0x00100010;
        steps = 0;
      } else {
        steps -= PART_SIZE / 2;
        for (uint8_t i = 0; i < PART_SIZE / 2 - 1; i++) {
          *data++ = 0x40007fff | 0x8000;
          *data++ = 0x20002000;
        }
        *data++ = 0x40007fff | 0x8000;
        last_entry = 0x20002000;
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
        // the first step should be stretched to maximum
        // The minimum period is 80ticks @200kHz
        // For PART_SIZE <= 31. This is possible, but high _and_ low
        // may need stretching.
        uint8_t extend_to = PART_SIZE - steps_to_do + 1;  // extend_to >= 2
        uint16_t ticks_high = ticks >> 1;
        uint16_t ticks_low = ticks - ticks_high;
        while (ticks_high > 0) {
          if ((ticks_high > 4) && (extend_to > 2)) {
            *data++ = 0x80018001;
            ticks_high -= 2;
            extend_to--;
          } else {
            *data++ = 0x80018000 | (ticks_high - 1);
            ticks_high = 0;
            extend_to--;
          }
        }
        while (extend_to > 1) {
          *data++ = 0x00010001;
          ticks_low -= 2;
          extend_to--;
        }
        last_entry = 0x00010000 | (ticks_low - 1);
        // Now add remaining steps, if any
        if (steps_to_do > 1) {
          uint16_t ticks_l = ticks >> 1;
          uint16_t ticks_r = ticks - ticks_l;
          // ticks_l <= ticks_r
          uint32_t rmt_entry = ticks_l;
          rmt_entry <<= 16;
          rmt_entry |= ticks_r | 0x8000;  // with step
          for (uint8_t i = 2; i <= steps_to_do; i++) {
            *data++ = last_entry;
            last_entry = rmt_entry;
          }
          last_entry = rmt_entry;
        }
      } else {
        // either >= 2*PART_SIZE or = PART_SIZE
        // every entry one step
        uint16_t ticks_l = ticks >> 1;
        uint16_t ticks_r = ticks - ticks_l;
        uint32_t rmt_entry = ticks_l;
        rmt_entry <<= 16;
        rmt_entry |= ticks_r | 0x8000;  // with step
        for (uint8_t i = 0; i < PART_SIZE - 1; i++) {
          *data++ = rmt_entry;
        }
        last_entry = rmt_entry;
      }
    }
  }
  if (!fill_part_one) {
    // Note: When enabling the continuous transmission mode by setting
    // RMT_REG_TX_CONTI_MODE, the transmitter will transmit the data on the
    // channel continuously, that is, from the first byte to the last one,
    // then from the first to the last again, and so on. In this mode, there
    // will be an idle level lasting one clk_div cycle between N and N+1
    // transmissions.
    last_entry -= 1;
  }
  *data = last_entry;

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
}
#endif
