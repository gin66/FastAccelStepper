#include "fas_queue/stepper_queue.h"
#if defined(SUPPORT_ESP32_PULSE_COUNTER) && (ESP_IDF_VERSION_MAJOR == 5)

// pcnt_new_channel() with a real GPIO still calls gpio_config() on that pin
// (input + pull-up, output off). Step and dir pins are already driven by
// RMT/MCPWM/GPIO; that drop of output routing stops the pulse driver.
// Channel is created with virtual IOs, then only input is appended via the
// GPIO matrix (same approach as the MCPWM/PCNT driver, without touching
// func_out_sel).

struct pcnt_unit_t {
  /*pcnt_group_t*/ void* group;
  portMUX_TYPE spinlock;
  int unit_id;
};

struct pcnt_chan_t {
  pcnt_unit_t* unit;
  int channel_id;
};

static void pcnt_listen_on_output_pin(int gpio_num, int signal_idx) {
  gpio_ll_input_enable(&GPIO, (gpio_num_t)gpio_num);
  esp_rom_gpio_connect_in_signal((uint32_t)gpio_num, (uint32_t)signal_idx,
                                 false);
}

static void pcnt_abort_attach(pcnt_channel_handle_t chan,
                              pcnt_unit_handle_t unit, bool enabled) {
  if (enabled) {
    pcnt_unit_disable(unit);
  }
  if (chan) {
    pcnt_del_channel(chan);
  }
  if (unit) {
    pcnt_del_unit(unit);
  }
}

bool FastAccelStepper::attachToPulseCounter(uint8_t unused_pcnt_unit,
                                            int16_t low_value,
                                            int16_t high_value,
                                            uint8_t dir_pin) {
  (void)unused_pcnt_unit;

  int low_limit = low_value;
  int high_limit = high_value;
  if ((low_limit == 0) && (high_limit == 0)) {
    low_limit = -32768;
    high_limit = 32767;
  }

  pcnt_unit_config_t config = {.low_limit = low_limit,
                               .high_limit = high_limit,
                               .intr_priority = 0,
                               .flags = {.accum_count = 0}};
  pcnt_unit_handle_t punit = NULL;
  if (pcnt_new_unit(&config, &punit) != ESP_OK) {
    return false;
  }

  pcnt_chan_config_t chan_config = {.edge_gpio_num = -1,
                                    .level_gpio_num = -1,
                                    .flags = {
                                        .invert_edge_input = 0,
                                        .invert_level_input = 0,
                                        .virt_edge_io_level = 0,
                                        .virt_level_io_level = 0,
                                        .io_loop_back = 0,
                                    }};

  pcnt_channel_level_action_t level_high = PCNT_CHANNEL_LEVEL_ACTION_KEEP;
  pcnt_channel_level_action_t level_low = PCNT_CHANNEL_LEVEL_ACTION_KEEP;

  if (dir_pin == PIN_UNDEFINED) {
    dir_pin = getDirectionPin();
  }
  bool use_dir =
      (dir_pin != PIN_UNDEFINED) && ((dir_pin & PIN_EXTERNAL_FLAG) == 0);
  if (use_dir) {
    if (directionPinHighCountsUp()) {
      level_low = PCNT_CHANNEL_LEVEL_ACTION_INVERSE;
    } else {
      level_high = PCNT_CHANNEL_LEVEL_ACTION_INVERSE;
    }
  }

  pcnt_channel_handle_t pcnt_chan = NULL;
  if (pcnt_new_channel(punit, &chan_config, &pcnt_chan) != ESP_OK) {
    pcnt_abort_attach(NULL, punit, false);
    return false;
  }

  int unit_id = punit->unit_id;
  int channel_id = pcnt_chan->channel_id;
  if ((unit_id < 0) || (unit_id >= SUPPORT_ESP32_PULSE_COUNTER) ||
      (channel_id < 0) || (channel_id >= 2)) {
    pcnt_abort_attach(pcnt_chan, punit, false);
    return false;
  }

  if (pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_HOLD) != ESP_OK) {
    pcnt_abort_attach(pcnt_chan, punit, false);
    return false;
  }
  if (pcnt_channel_set_level_action(pcnt_chan, level_high, level_low) !=
      ESP_OK) {
    pcnt_abort_attach(pcnt_chan, punit, false);
    return false;
  }

  uint8_t step_pin = getStepPin();
  int pulse_sig = pcnt_periph_signals.groups[0]
                      .units[unit_id]
                      .channels[channel_id]
                      .pulse_sig;
  pcnt_listen_on_output_pin(step_pin, pulse_sig);
  if (use_dir) {
    int control_sig = pcnt_periph_signals.groups[0]
                          .units[unit_id]
                          .channels[channel_id]
                          .control_sig;
    pcnt_listen_on_output_pin(dir_pin, control_sig);
  }

  if (pcnt_unit_enable(punit) != ESP_OK) {
    pcnt_abort_attach(pcnt_chan, punit, false);
    return false;
  }
  if (pcnt_unit_clear_count(punit) != ESP_OK) {
    pcnt_abort_attach(pcnt_chan, punit, true);
    return false;
  }
  if (pcnt_unit_start(punit) != ESP_OK) {
    pcnt_abort_attach(pcnt_chan, punit, true);
    return false;
  }

  _attached_pulse_unit = punit;
  return true;
}

void FastAccelStepper::clearPulseCounter() {
  if (pulseCounterAttached()) {
    pcnt_unit_clear_count(_attached_pulse_unit);
  }
}

int16_t FastAccelStepper::readPulseCounter() {
  int value = 0;
  if (pulseCounterAttached()) {
    pcnt_unit_get_count(_attached_pulse_unit, &value);
  }
  return value;
}

#endif
