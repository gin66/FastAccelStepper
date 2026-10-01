#!/usr/bin/env python3
"""
config/channel_configs.py — Channel configuration presets for Saleae test harness.

Defines all supported channel configuration modes:
  - 8ch_step_only: 8 channels, Step only (no Dir)
  - 7ch_shared_dir: 7 channels, shared Dir line
  - 4ch_rmt: 4 steppers, all using RMT driver
  - 4ch_mcpwm: 4 steppers, all using MCPWM/PCNT driver
  - 2rmt_2i2s: 2 steppers on RMT + 2 steppers on I2S Direct
  - 4ch_i2s_extender: 4-8 steppers on I2S Extender
  - 6ch_i2s_mux: 6 steppers on I2S Mux
  - mixed: Arbitrary mix of RMT + MCPWM + I2S
"""

# Channel configuration presets
CHANNEL_CONFIGS = {
    "8ch_step_only": {
        "description": "8 channels, Step only (no Dir). Each stepper has its own Step channel.",
        "stepper_count": 8,
        "driver": "rmt_v2",
        "steppers": [
            {"name": "A", "step_pin": 2, "dir_pin": None},
            {"name": "B", "step_pin": 4, "dir_pin": None},
            {"name": "C", "step_pin": 17, "dir_pin": None},
            {"name": "D", "step_pin": 18, "dir_pin": None},
            {"name": "E", "step_pin": 21, "dir_pin": None},
            {"name": "F", "step_pin": 22, "dir_pin": None},
            {"name": "G", "step_pin": 23, "dir_pin": None},
            {"name": "H", "step_pin": 24, "dir_pin": None},
        ],
    },
    "4ch_rmt": {
        "description": "4 steppers, all using RMT driver (Step + Dir each).",
        "stepper_count": 4,
        "driver": "rmt_v2",
        "steppers": [
            {"name": "A", "step_pin": 2, "dir_pin": 0},
            {"name": "B", "step_pin": 4, "dir_pin": 16},
            {"name": "C", "step_pin": 17, "dir_pin": 5},
            {"name": "D", "step_pin": 18, "dir_pin": 19},
        ],
    },
    "4ch_mcpwm": {
        "description": "4 steppers, all using MCPWM/PCNT driver.",
        "stepper_count": 4,
        "driver": "mcpwm_pcnt",
        "steppers": [
            {"name": "A", "step_pin": 25, "dir_pin": 26},
            {"name": "B", "step_pin": 27, "dir_pin": 32},
            {"name": "C", "step_pin": 33, "dir_pin": 14},
            {"name": "D", "step_pin": 15, "dir_pin": 4},
        ],
    },
    "2rmt_2i2s": {
        "description": "2 steppers on RMT + 2 steppers on I2S Direct.",
        "stepper_count": 4,
        "driver": "mixed",
        "steppers": [
            {"name": "A", "step_pin": 2, "dir_pin": 0, "driver": "rmt_v2"},
            {"name": "B", "step_pin": 4, "dir_pin": 16, "driver": "rmt_v2"},
            {"name": "C", "step_pin": 17, "dir_pin": None, "driver": "i2s_direct"},
            {"name": "D", "step_pin": 18, "dir_pin": None, "driver": "i2s_direct"},
        ],
    },
    "mixed": {
        "description": "Arbitrary mix of RMT + MCPWM + I2S. Configurable per-stepper.",
        "stepper_count": 4,
        "driver": "mixed",
        "steppers": [
            {"name": "A", "step_pin": 2, "dir_pin": 0, "driver": "rmt_v2"},
            {"name": "B", "step_pin": 4, "dir_pin": 16, "driver": "mcpwm_pcnt"},
            {"name": "C", "step_pin": 17, "dir_pin": 5, "driver": "i2s_direct"},
            {"name": "D", "step_pin": 18, "dir_pin": 19, "driver": "i2s_mux"},
        ],
    },
}


def get_config(config_name):
    """Get channel configuration by name."""
    return CHANNEL_CONFIGS.get(config_name)


def list_configs():
    """List all available channel configurations."""
    return list(CHANNEL_CONFIGS.keys())