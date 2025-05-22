#include "StepperISR.h"
#if defined(SUPPORT_ESP32_PULSE_COUNTER) && (ESP_IDF_VERSION_MAJOR == 5)

// Why the hell, does espressif think, that the unit and channel id are not
// needed ? Without unit/channel ID, the needed parameter for
// gpio_matrix_in/gpio_iomux_in cannot be derived.
//
// Here we declare the private pcnt_chan_t structure, which is not save.
struct pcnt_unit_t {
  /*pcnt_group_t*/ void *group;
  portMUX_TYPE spinlock;
  int unit_id;
  // remainder of struct not needed
};
struct pcnt_chan_t {
  pcnt_unit_t *unit;
  int channel_id;
  // remainder of struct not needed
};

bool FastAccelStepper::attachToPulseCounter(uint8_t unused_pcnt_unit,
                                            int16_t low_value,
                                            int16_t high_value,
                                            uint8_t dir_pin) {
  pcnt_unit_config_t config = {.low_limit = low_value,
                               .high_limit = high_value,
                               .intr_priority = 0,
                               .flags = {.accum_count = 0}};
  pcnt_unit_handle_t punit = NULL;
  esp_err_t rc;
  rc = pcnt_new_unit(&config, &punit);
  if (rc != ESP_OK) {
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

  if (dir_pin != PIN_UNDEFINED && (dir_pin & PIN_EXTERNAL_FLAG) == 0) {
    chan_config.level_gpio_num = dir_pin;
    if (directionPinHighCountsUp()) {
      level_low = PCNT_CHANNEL_LEVEL_ACTION_INVERSE;
    } else {
      level_high = PCNT_CHANNEL_LEVEL_ACTION_INVERSE;
    }
  }

  pcnt_channel_handle_t pcnt_chan = NULL;
  rc = pcnt_new_channel(punit, &chan_config, &pcnt_chan);
  if (rc != ESP_OK) {
    pcnt_del_unit(punit);
    return false;
  }

  int unit_id = punit->unit_id;
  int channel_id = pcnt_chan->channel_id;
  if ((unit_id < 0) || (unit_id >= SUPPORT_ESP32_PULSE_COUNTER)) {
    // perhaps the pcnt_chan_t-structure is changed !?
    pcnt_del_channel(pcnt_chan);
    pcnt_del_unit(punit);
    return false;
  }

  rc =
      pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_HOLD);
  if (rc != ESP_OK) {
    return false;
  }
  rc = pcnt_channel_set_level_action(pcnt_chan, level_high, level_low);
  if (rc != ESP_OK) {
    return false;
  }

  rc = pcnt_unit_enable(punit);
  if (rc != ESP_OK) {
    return false;
  }
  rc = pcnt_unit_clear_count(punit);
  if (rc != ESP_OK) {
    return false;
  }
  rc = pcnt_unit_start(punit);
  if (rc != ESP_OK) {
    return false;
  }

  uint8_t step_pin = getStepPin();
#ifdef TRACE
  printf("pins = %d/%d unit_id=%d channel_id=%d\n", step_pin, dir_pin, unit_id,
         channel_id);
#endif
  int signal = pcnt_periph_signals.groups[0]
                   .units[unit_id]
                   .channels[channel_id]
                   .pulse_sig;
  gpio_matrix_in(step_pin, signal, 0);
  gpio_iomux_in(step_pin, signal);
  if (dir_pin != PIN_UNDEFINED && (dir_pin & PIN_EXTERNAL_FLAG) == 0) {
    pinMode(dir_pin, OUTPUT);
    int control = pcnt_periph_signals.groups[0]
                      .units[unit_id]
                      .channels[channel_id]
                      .control_sig;
    gpio_iomux_out(dir_pin, 0x100, false);
    gpio_matrix_in(dir_pin, control, 0);
    gpio_iomux_in(dir_pin, control);
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
