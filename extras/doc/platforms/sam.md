# Atmel SAM Due Platform

[Back to README](../../../README.md) | [Platform index](../hardware.md)

This is supported by clazarowitz.

## Usage limits

* allows up to 50000 generated steps per second
* supports up to six stepper motors using Step/Direction/Enable Control (Direction and Enable is optional)
* Steppers' command queue depth: 32

Tested with max two stepper motors with 50 kHz step rate by clazarowitz.

## Implementation

**Note:** The SAM platform cannot be tested in CI. An audit against the SAM3X8E
datasheet has identified
[6 open issues](../ai_improvements/05_sam_platform_audit.md) (4 critical, 2
minor), including invalid pin mappings and a wrong timer register value. Any
changes to the SAM code must be cross-checked against that document.
