#include <stdint.h>

#include "fas_queue/stepper_queue.h"

#if defined(NEED_GENERIC_GET_CURRENT_POSITION)
int32_t StepperQueue::getCurrentPosition() {
  fasDisableInterrupts();
  uint32_t pos = (uint32_t)queue_end.pos;
  uint8_t rp = read_idx;
  bool is_empty = (rp == next_write_idx);
  struct queue_entry* e = &entry[rp & QUEUE_LEN_MASK];
  uint16_t pos_last16 = e->end_pos_last16;
  uint8_t steps = e->steps;
  fasEnableInterrupts();
  if (!is_empty) {
    int16_t adjust = 0;

    uint16_t pos16 = pos & 0xffff;
    uint8_t transition = ((pos16 >> 12) & 0x0c) | (pos_last16 >> 14);
    switch (transition) {
      case 0:   // 00 00
      case 5:   // 01 01
      case 10:  // 10 10
      case 15:  // 11 11
        break;
      case 1:   // 00 01
      case 6:   // 01 10
      case 11:  // 10 11
      case 12:  // 11 00
        pos += 0x4000;
        break;
      case 4:   // 01 00
      case 9:   // 10 01
      case 14:  // 11 10
      case 3:   // 00 11
        pos -= 0x4000;
        break;
      case 2:   // 00 10
      case 7:   // 01 11
      case 8:   // 10 00
      case 13:  // 11 01
        break;  // TODO: ERROR
    }
    pos = (int32_t)((pos & 0xffff0000) | pos_last16);

    if (steps != 0) {
      if (e->countUp) {
        adjust = -steps;
      } else {
        adjust = steps;
      }
      pos += adjust;
    }
  }
  return pos;
}
#endif
