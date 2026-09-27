# Sync-start test for ATmega328P (Timer1).
#
# Verify that synchronizedStart() produces synchronous pulses on
# StepA (OC1A) and StepB (OC1B).

SRC=$(shell find ../../src -type f) $(wildcard src/*)

DUT=$(shell gawk '/env:/{print(substr($$1,6,length($$1)-6))}' platformio.ini)

# 328P uses Timer1 (OC1A + OC1B)
# trace addresses are for the AVR port
TRACES=-at StepA=trace@0x25/0x02  # OC1A PB1  pin 9  328p
TRACES+=-at StepB=trace@0x25/0x04  # OC1B PB2  pin 10 328p

DEVICE=$(DUT)

ifeq ($(DUT),atmega328p)
TRACES=-at StepA=trace@0x25/0x02  # OC1A PB1  pin 9
TRACES+=-at StepB=trace@0x25/0x04  # OC1B PB2  pin 10
else ifeq ($(DUT),atmega328)
TRACES=-at StepA=trace@0x25/0x02  # OC1A PB1  pin 9
TRACES+=-at StepB=trace@0x25/0x04  # OC1B PB2  pin 10
else ifeq ($(DUT),atmega168p)
TRACES=-at StepA=trace@0x25/0x02  # OC1A PB1  pin 9
TRACES+=-at StepB=trace@0x25/0x04  # OC1B PB2  pin 10
else ifeq ($(DUT),atmega168)
TRACES=-at StepA=trace@0x25/0x02  # OC1A PB1  pin 9
TRACES+=-at StepB=trace@0x25/0x04  # OC1B PB2  pin 10
endif

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
	pio run -e $(DUT)
	../run_avr $(FIRMWARE) -m $(DEVICE) -o x.vcd $(TRACES)

clean:
	rm -fR .pio .tested x.vcd result.txt