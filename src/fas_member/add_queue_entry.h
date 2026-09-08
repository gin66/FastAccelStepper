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
  } else if (dir_change_needed) {
    res = q->addDirChangePauseToQueue(cmd, start, _dir_change_delay_ticks);
    if (res != AQE_OK) {
      return res;
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
