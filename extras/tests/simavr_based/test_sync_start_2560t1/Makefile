# Sync-start test for ATmega2560 Timer1.
#
# Verify that synchronizedStart() produces synchronous pulses on
# StepA (Timer1 OC1A) and StepB (Timer1 OC1B).

SRC=$(shell find ../../../../src -type f) $(wildcard src/*)

DUT=$(shell gawk '/env:/{print(substr($$1,6,length($$1)-6))}' platformio.ini)

# 2560 Timer1 traces: OC1A (PB5 pin 11), OC1B (PB6 pin 12)
TRACES=-at StepA=trace@0x025/0x20  # OC1A PB5  pin 11
TRACES+=-at StepB=trace@0x025/0x40  # OC1B PB6  pin 12

DEVICE=atmega2560

FIRMWARE=".pio/build/$(DUT)/firmware.elf"
DIR=$(shell env pwd)

test: .tested

.tested: result.txt expect.txt ../judge_sync.awk
	echo DUT=$(DUT)
	rm -f .tested
	gawk -f ../judge_sync.awk x.vcd expect.txt
	test -f .tested

result.txt: x.vcd
	gawk -f ../eval.awk x.vcd > result.txt
	cat expect.txt

x.vcd: $(SRC) ../run_avr platformio.ini FastAccelStepper
	~/.platformio/penv/bin/pio run -e $(DUT) || ~/.local/bin/pio run -e $(DUT) || env pio run -e $(DUT)
	../run_avr $(FIRMWARE) -m $(DEVICE) -o x.vcd $(TRACES)

clean:
	rm -fR .pio .tested x.vcd result.txt
