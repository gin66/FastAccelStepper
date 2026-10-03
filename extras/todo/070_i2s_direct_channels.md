# 175 i2s_direct has 2 channels on ESP32, not 3

## Priority

**MEDIUM** — documentation/constant bug.  The constant overstates the number
of I2S channels by one, so anything sizing a queue array from `NUM_QUEUES`
over-allocates by one.

## Finding

Measured `--mode scale --driver i2s_direct --pin-mode nodir` on the connected
board:

- **n=1 and n=2 pass, n=3..8 are refused**, and the refusal is the IDF's own
  -- `E (129) i2s_common: i2s_new_channel(902)`, i.e. `ESP_ERR_NO_MEM` from
  the peripheral rather than a policy limit.
- `SOC_I2S_NUM` is 2 on the ESP32 and `I2sManager::create()` allocates one
  TX channel per manager (`i2s_new_channel(..., &chan, NULL)`,
  `i2s_manager.cpp:31`), so two is the ceiling and the constant overstates it
  by one.

This is a *different kind* of wrong from the MCPWM defect (171).  There the
constant counted allocations correctly and only the health was bad.  Here the
allocation figure itself is wrong, so anything sizing a queue array from
`NUM_QUEUES` over-allocates by one.

It was found only because R7 stopped trusting the constant: `DRIVER_MAXS` had
carried `i2s_direct: 3` through every previous run, unchanged and untested,
because `i2s_direct` had only ever been measured as a *partner* in a `sync`
combination with a stepper on another driver.

The failure mode is graceful — `I2sManager::create()` returns nullptr and the
harness reports `refused` with the peripheral's own error — so this is a
capacity and documentation bug, not a crash.
