#
# SimAVR test for the examples/naxes multi-axis sketch.
#
# Builds the naxes sketch for the ATmega328p (two step channels, OC1A / OC1B
# -> NAXES_HW = 2), drives it under the atmega328p with run_avr and traces the
# StepA / StepB / DirA / DirB pins, then VALIDATES THE GEOMETRY of the
# reconstructed helix/hexagon/square/origin curve with detect_geometry.py.
#
# The eval.awk + judge.awk path (single-axis position/timing vs expect.txt) is
# insufficient for a multi-axis curve; the detector is the judge here.
#
# Run from the top-level harness:
#   make -C extras/tests/simavr_based
# Or standalone:
#   make -C test_naves
#

SRC=$(shell find ../../../../src -type f) $(wildcard src/*)

#
# The sketch is autonomous (no serial input), so there is no SIM_TEST_INPUT
# build flag. Trace the two step channels (Timer 1) and their direction pins
# on the ATmega328p, matching the pins in examples/naxes/StepperPins_naxes_avr.h.
DUT=atmega328p
DEVICE=atmega328p
TRACES=-at StepISR=trace@0x25/0x08     # PB3
TRACES+=-at FillISR=trace@0x25/0x10    # PB4
TRACES+=-at StepA=trace@0x25/0x02      # OC1A PB1 9
TRACES+=-at StepB=trace@0x25/0x04      # OC1B PB2 10
TRACES+=-at DirA=trace@0x2b/0x20       # Pin 5  PD5
TRACES+=-at DirB=trace@0x2b/0x80       # Pin 7  PD7
TRACES+=-at EnableA=trace@0x2b/0x40    # Pin 6  PD6
TRACES+=-at EnableB=trace@0x25/0x01    # Pin 0  PB0

FIRMWARE=".pio/build/$(DUT)/firmware.elf"

DIR=$(shell env pwd)

ifndef SILENCE
	SILENCE=0
endif

test: .tested

# The geometry detector is the judge: exit 0 (all checks pass) gates .tested.
.tested: x.vcd detect_geometry.py
	rm -f .tested
	python3 detect_geometry.py x.vcd
	touch .tested

# Build the firmware and run it under the simulator, capturing the Step/Dir
# pin trace to x.vcd.
x.vcd: $(SRC) ../run_avr platformio.ini src/.naxes
	env pio run -e $(DUT) || ~/.platformio/penv/bin/pio run -e $(DUT) || ~/.local/bin/pio run -e $(DUT)
	../run_avr $(FIRMWARE) -m $(DEVICE) -o x.vcd $(TRACES)

# Link the whole example (including the per-architecture pin headers) into
# src/ so PlatformIO compiles naxes.ino. Each make line is its own shell, so
# the symlinks are created with absolute targets and the stamp lives in src/.
PRJ_ROOT=$(shell git rev-parse --show-toplevel)
NAXES_SRC=$(PRJ_ROOT)/examples/naxes

src/.naxes: $(wildcard $(NAXES_SRC)/*)
	mkdir -p src
	for f in $(NAXES_SRC)/*; do ln -sf "$$f" src/; done
	ln -s .dir $@

clean:
	rm -fR .pio .tested x.vcd result.txt
