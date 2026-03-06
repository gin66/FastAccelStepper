#pragma once

#include <stdint.h>

#define MAX_SEQUENCE_COMMANDS 64

enum MoveCommandType {
  MOVE_TYPE_MOVE,
  MOVE_TYPE_MOVE_TO,
  MOVE_TYPE_RUN_FORWARD,
  MOVE_TYPE_RUN_BACKWARD,
  MOVE_TYPE_STOP,
  MOVE_TYPE_FORCE_STOP,
  MOVE_TYPE_WAIT,
  MOVE_TYPE_WAIT_FOR_STOP,
  MOVE_TYPE_SET_SPEED,
  MOVE_TYPE_SET_ACCEL,
  MOVE_TYPE_SET_POSITION,
  MOVE_TYPE_ENABLE,
  MOVE_TYPE_DISABLE,
  MOVE_TYPE_KEEP_RUNNING
};

struct MoveCommand {
  MoveCommandType type;
  uint8_t stepper_id;
  int32_t value1;
  int32_t value2;
  uint32_t delay_ms;

  void setDefaults() {
    type = MOVE_TYPE_STOP;
    stepper_id = 0;
    value1 = 0;
    value2 = 0;
    delay_ms = 0;
  }
};

struct MoveSequence {
  uint8_t id;
  char name[32];
  bool loop;
  uint8_t num_commands;
  MoveCommand commands[MAX_SEQUENCE_COMMANDS];
  bool is_executing;
  uint8_t current_command_index;

  void setDefaults() {
    id = 255;
    name[0] = '\0';
    loop = false;
    num_commands = 0;
    is_executing = false;
    current_command_index = 0;
    for (int i = 0; i < MAX_SEQUENCE_COMMANDS; i++) {
      commands[i].setDefaults();
    }
  }
};
