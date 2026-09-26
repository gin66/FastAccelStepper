# Ramp generator

# Introduction

The ramp generator increases the speed linearly with the acceleration.
If e.g. a=1000 steps/s², then after 1s the speed is 1000 steps/s.

The full ramp consists of three phases: acceleration+coasting+deceleration.
If the number of steps are less than the required steps for acceleration and deceleration, then there will not be any coasting phase.

For illustration here an ascii chart of the ramp

```
Speed
    |       -------------------
    |      /                   \
    |     /                     \
    |    /                       \
    0---/                         \------
        |<->|<--------------->|<->|
          Ta        Tc          Td
    -----------------------------------> Time

Ta ... acceleration time.
Tc ... coasting time.
Td ... deceleration time.
```

The total ramp time is Ta + Tc + Td = T 

Due to acceleration = deceleration, the related times Ta and Td are same.
Since Ta = Td:

    T = 2*Ta + Tc

The steps executed in coasting are simply:

    steps_coasting = v * Tc

The steps executed during acceleration and deceleration are:

    steps_acceleration = 0.5 * a * Ta²

The total steps are:

    steps = steps_acceleration + steps_coasting + steps_deceleration
            = 2 * steps_acceleration + steps_coasting
            = a * Ta² + v * Tc
            = a * Ta² + (a * Ta) * Tc
            = a * Ta * (Ta + Tc)
            = a * Ta * (T - Ta)
This can be rearranged to calculate the required acceleration to perform `steps` during the total ramp time T and acceleration time Ta:

    a = steps / (Ta * (T - Ta))

# Calculation example

The stepper motor should perform 32000 steps in 10s.

The required acceleration and speed for different acceleration times are calculated as this:

    Ta = 1s => a = 32000 / (1 * 9) steps/s² = 3556 steps/s²
               v = a*Ta = 3556 steps/s => 281us/step
    Ta = 2s => a = 32000 / (2 * 8) steps/s² = 2000 steps/s²
               v = a*Ta = 4000 steps/s => 250us/step
    Ta = 3s => a = 32000 / (3 * 7) steps/s² = 1524 steps/s²
               v = a*Ta = 4571 steps/s => 218us/step
    Ta = 4s => a = 32000 / (4 * 6) steps/s² = 1333 steps/s²
               v = a*Ta = 5333 steps/s => 188us/step
    Ta = 5s => a = 32000 / (5 * 5) steps/s² = 1280 steps/s²
               v = a*Ta = 6400 steps/s => 156us/step

As commands for the StepperDemo:
```
	M1 A3556 V281 R32000
	M1 A2000 V250 R32000
	M1 A1524 V218 R32000
	M1 A1333 V188 R32000
	M1 A1280 V156 R32000
```

# Special case

The minimum command time for a bundle of steps is `MIN_CMD_TICKS`. The ramp generator need to make sure,
that any command issued to the command queue meets this requirement.

With high acceleration this can get problematic coming from or going to stand still.

The required condition to meet this requirement is:

    steps * (1/v) ≥ T_CMD

From stand still the speed after `MIN_CMD_TICKS` is:

    v_start = a * T_CMD

So the related condition for steps is:

    steps * (1 / (a * T_CMD)) ≥ T_CMD
    steps ≥ a * T_CMD²

Based on this there can be deducted two problems:

1. Any move command with less steps cannot be fulfilled with this acceleration
   
    => Remedy: Run these steps at maximum allowed speed based on `MIN_CMD_TICKS`

2. While ramping down the last issued command may violate this condition

    => Remedy: Run these steps at maximum allowed speed based on `MIN_CMD_TICKS`.
       The problem is to distinguish this case from a legitimate overshoot situation.
