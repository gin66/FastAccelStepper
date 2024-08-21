#include "StepperISR.h"
#if defined(SUPPORT_ESP32_PULSE_COUNTER) && (ESP_IDF_VERSION_MAJOR == 5)

uint32_t sig_idx[SUPPORT_ESP32_PULSE_COUNTER] = {
	PCNT_SIG_CH0_IN0_IDX, PCNT_SIG_CH0_IN1_IDX,
	PCNT_SIG_CH0_IN2_IDX, PCNT_SIG_CH0_IN3_IDX,
#if SUPPORT_ESP32_PULSE_COUNTER == 8
	PCNT_SIG_CH0_IN4_IDX, PCNT_SIG_CH0_IN5_IDX,
	PCNT_SIG_CH0_IN6_IDX, PCNT_SIG_CH0_IN7_IDX,
#endif
};
uint32_t ctrl_idx[SUPPORT_ESP32_PULSE_COUNTER] = {
	PCNT_CTRL_CH0_IN0_IDX, PCNT_CTRL_CH0_IN1_IDX,
	PCNT_CTRL_CH0_IN2_IDX, PCNT_CTRL_CH0_IN3_IDX,
#if SUPPORT_ESP32_PULSE_COUNTER == 8
	PCNT_CTRL_CH0_IN4_IDX, PCNT_CTRL_CH0_IN5_IDX,
	PCNT_CTRL_CH0_IN6_IDX, PCNT_CTRL_CH0_IN7_IDX
#endif
};

// Why the hell, does espressif think, that the channel id is not needed ?
// Without channel ID, the needed parameter for gpio_iomux_in cannot be derived.
//
// Here we declare the private pcnt_chan_t structure, which is not save.
struct pcnt_chan_t {
	pcnt_unit_t *unit;
	int channel_id;
	// remainder of struct not needed
};

bool FastAccelStepper::attachToPulseCounter(uint8_t unused_pcnt_unit,
		                                    int16_t low_value,
                                            int16_t high_value) {
  pcnt_unit_config_t config = { 
	  .low_limit = low_value, 
	  .high_limit = high_value, 
	  .intr_priority = 0, 
	  .flags = {
		  .accum_count = 0
	  } 
  };
  pcnt_unit_handle_t punit = NULL;
  esp_err_t rc;
  rc = pcnt_new_unit(&config, &punit);
  if (rc != ESP_OK) {
	  return false;
  }

  uint8_t step_pin = getStepPin();
  pcnt_chan_config_t chan_config = {
	  .edge_gpio_num = step_pin,
	  .level_gpio_num = -1,
	  .flags = {
		  .invert_edge_input = 0,
		  .invert_level_input = 0,
		  .virt_edge_io_level = 0,
		  .virt_level_io_level = 0,
		  .io_loop_back = 0,
	  }
  };

  pcnt_channel_level_action_t level_high = PCNT_CHANNEL_LEVEL_ACTION_KEEP;
  pcnt_channel_level_action_t level_low = PCNT_CHANNEL_LEVEL_ACTION_KEEP;
  uint8_t dir_pin = getDirectionPin();
  if (dir_pin != PIN_UNDEFINED) {
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

  int channel_id = pcnt_chan->channel_id;
  if ((channel_id < 0) || (channel_id >= SUPPORT_ESP32_PULSE_COUNTER)) {
	  // perhaps the pcnt_chan_t-structure is changed !?
	  pcnt_del_channel(pcnt_chan);
	  pcnt_del_unit(punit);
	  return false;
  }

  pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
  pcnt_channel_set_level_action(pcnt_chan, level_high, level_low);

  pcnt_unit_enable(punit);

  detachFromPin();
  reAttachToPin();
  gpio_iomux_in(step_pin, sig_idx[channel_id]);
  if (dir_pin != PIN_UNDEFINED) {
    gpio_iomux_out(dir_pin, 0x100, false);
    gpio_iomux_in(dir_pin, ctrl_idx[channel_id]);
    pinMode(dir_pin, OUTPUT);
  }

  pcnt_unit_clear_count(punit);
  pcnt_unit_start(punit);

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

