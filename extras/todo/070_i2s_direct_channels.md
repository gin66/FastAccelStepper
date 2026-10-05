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

## 2026-10-05 — confirmed on both I2S SDKs, refusal signature clarified

`--mode scale --driver i2s_direct --pin-mode nodir` on both rows that have I2S
queues, from the full release matrix. `n=1` and `n=2` pass, `n=3…8` are refused
— the same bound, now measured on IDF 5.5.3 **and** 6.1.0 in one run rather than
on one board in one sitting.

The refusal carries more information than the earlier note recorded. Full
stderr from `n=3`:

```
W (389) i2s_platform: i2s controller 0 has been occupied by i2s_driver
W (389) i2s_platform: i2s controller 0 has been occupied by i2s_driver
W (389) i2s_platform: i2s controller 1 has been occupied by i2s_driver
E (399) i2s_common: i2s_new_channel(1032): no available channel found
ERR connect step 2 n=2 drv=i2s_direct nodir=1
```

Three points, none of which change the conclusion:

- The **error** is still inside `i2s_new_channel`, so the earlier reading holds;
  only the line number moved (902 → 1032 on 5.5.3, 1254 on 6.1.0) and the
  message is now explicit: `no available channel found`, i.e. `ESP_ERR_NO_MEM`
  rather than a bare error code.
- The **`W` lines name the cause directly** — both controllers, 0 and 1, are
  occupied. Two channels on the ESP32, two allocated, no third. That is the
  whole finding in three lines of IDF's own diagnostics.
- `ERR connect step 2` says the refusal happened at **step index 2**, i.e. the
  third stepper, so the firmware's bound and the peripheral's agree exactly.
  There is no off-by-one between what the library counts and what the hardware
  has.

`SOC_I2S_NUM = 2` stands. The constant still overstates it by one.
