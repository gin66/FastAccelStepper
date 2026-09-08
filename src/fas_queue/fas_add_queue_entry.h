AqeResultCode FastAccelStepper::addQueueEntry(
    const struct stepper_command_s* cmd, bool start) {
  StepperQueue* q = _queue();
  if (cmd == NULL) {
    return q->addQueueEntry(NULL, start);
  }
  if (cmd->ticks < q->max_speed_in_ticks) {
    return AQE_ERROR_TICKS_TOO_LOW;
  }

  if (_dirPin != PIN_UNDEFINED) {
    if (!isQueueRunning()) {
      if (_engine != NULL) {
        if (_engine->isDirPinBusy(_dirPin, _queue_num)) {
          return AQE_DIR_PIN_IS_BUSY;
        }
      }
    }
  } else {
    if (!cmd->count_up) {
      return AQE_ERROR_NO_DIR_PIN_TO_TOGGLE;
    }
  }

  AqeResultCode res = AQE_OK;
  if (_autoEnable) {
    fasDisableInterrupts();
    uint16_t delay_counter = _auto_disable_delay_counter;
    fasEnableInterrupts();
    if (delay_counter == 0) {
      // outputs are disabled
      if (!enableOutputs()) {
        return AQE_WAIT_FOR_ENABLE_PIN_ACTIVE;
      }
      // if on delay is defined, fill queue if required amount of pauses before
      // the first step
      if (_on_delay_ticks > 0) {
        uint32_t delay = _on_delay_ticks;
        // this delay sets count_up appropriately. If this is shorter than
        // dir_change_delay_ticks, then extend accordingly
        if ((delay < _dir_change_delay_ticks) &&
            (q->queue_end.count_up != cmd->count_up)) {
          delay = _dir_change_delay_ticks;
        }
        while (delay > 0) {
          uint32_t ticks = delay >> 1;
          uint16_t ticks_u16 = ticks;
          if (ticks > 65535) {
            ticks_u16 = 65535;
          } else if (ticks < 32768) {
            ticks_u16 = delay;
          }
          struct stepper_command_s start_cmd = {
              .ticks = ticks_u16, .steps = 0, .count_up = cmd->count_up};
          q->addQueueEntry(&start_cmd, false);
          delay -= ticks_u16;
        }
        res = q->addQueueEntry(NULL, start);
        if (res != AQE_OK) {
          return res;
        }
      }
    }
  }
  bool dir_change_needed =
      (_dirPin != PIN_UNDEFINED) && (q->queue_end.count_up != cmd->count_up);

  if (_dirPin & PIN_EXTERNAL_FLAG) {
    if (dir_change_needed) {
      if (!handleExternalDirectionPin(q, cmd->count_up)) {
        struct stepper_command_s pause_cmd = {
            .ticks = US_TO_TICKS((uint16_t)2000),
            .steps = 0,
            .count_up = cmd->count_up};
        res = q->addQueueEntry(&pause_cmd, start);
        if (res == AQE_OK) {
          res = AQE_DIR_PIN_2MS_PAUSE_ADDED;
        }
        return res;
      }
    }
  } else if (dir_change_needed && (cmd->steps != 0)) {
#if defined(BEFORE_DIR_CHANGE_DELAY_TICKS)
    uint16_t before_delay = BEFORE_DIR_CHANGE_DELAY_TICKS(q);
#else
    uint16_t before_delay = 0;
#endif
    uint16_t after_delay = _dir_change_delay_ticks;

#if defined(AFTER_DIR_CHANGE_DELAY_TICKS)
    after_delay = fas_max(AFTER_DIR_CHANGE_DELAY_TICKS(q), after_delay);
#endif

    if (q->_nr_of_pauses != 0 && q->_last_pause_ticks >= before_delay) {
      before_delay = 0;
    }

    uint8_t commands_needed = 1;
    if (before_delay > 0) {
      commands_needed++;
    }
    if (after_delay > 0) {
      commands_needed++;
    }
    if (q->queueEntries() >= QUEUE_LEN - commands_needed) {
      return AQE_DIR_PIN_IS_BUSY;
    }

    if (before_delay > 0) {
      struct stepper_command_s before_cmd = {
          .ticks = (uint16_t)fas_max(before_delay, MIN_CMD_TICKS),
          .steps = 0,
          .count_up = q->queue_end.count_up }; // delay with old value
      res = q->addQueueEntry(&before_cmd, start);
      if (res != AQE_OK) {
        return res;
      }
    }

    if (after_delay > 0) {
      struct stepper_command_s after_cmd = {
          .ticks = (uint16_t)fas_max(after_delay, MIN_CMD_TICKS),
          .steps = 0,
          .count_up = cmd->count_up};
      res = q->addQueueEntry(&after_cmd, start);
      if (res != AQE_OK) {
        return res;
      }
    }
  }
  res = q->addQueueEntry(cmd, start);
  if (res == AQE_OK) {
    if (_autoEnable) {
      fasDisableInterrupts();
      _auto_disable_delay_counter = _off_delay_count;
      fasEnableInterrupts();
    }
  }

  return res;
}
