# ESP32 hardware tests (classic ESP32, serial DEV below).
#
# Usage:
#   make                              compile default firmware + test M1 and M9
#   make compile_arduino_espidf4      Arduino 2.0.17 / IDF 4.4.7  (esp32_V6_8_1)
#   make compile_arduino_3            Arduino 3.1.1 / IDF 5.3.2   (esp32_pioarduino_V53_03_11)
#   make compile_arduino_latest       Arduino 2.0.17 on PIO 7.1.2 (esp32_V7_1_2)
#   make compile_idf5                 native ESP-IDF 5.3.1        (esp32_idf_V6_9_0)
#   make compile_idf6                 native ESP-IDF 6.1.0        (esp32_idf_V7_1_2)
#   make compile_idf4                 native ESP-IDF 4.4.3        (esp32_idf_V5_3_0, often cannot run)
#   make test                         run seq_* on M1 and M9 (whatever is already flashed)
#   make test_idf5_3                  Driver-Types config: M1=RMT, M2=I2S, M3=MCPWM
#   make test_all                     flash+test the usual set (skips ones already stamped)
#   make clean                        drop logs and test_all stamps
#
# `pio run -e` names must exist in extras/ci/platformio.ini
# (from extras/ci/build_matrix.yaml). PIO version != ESP-IDF version:
#   https://github.com/platformio/platform-espressif32/releases
#
# Default DONT_CARE motors (make test):
#   Arduino 2.x / IDF 4: M1=MCPWM, M9=RMT
#   Arduino 3.x / IDF 5: M1=RMT,   M9=I2S   (MCPWM is M3 with STEPPER_CONFIG=1)

.PHONY: all test test_all test_idf5_3 clean compile help \
	compile_arduino_espidf4 compile_arduino_3 compile_arduino_latest \
	compile_idf4 compile_idf5 compile_idf6 \
	compile_esp32_V6_8_1 compile_esp32_pioarduino_V53_03_11 compile_esp32_V7_1_2 \
	compile_esp32_idf_V5_3_0 compile_esp32_idf_V6_9_0 compile_esp32_idf_V7_1_2

# Test scripts use uv shebang with pyserial (no grabserial needed).

# macos:
DEV ?= cu.usbserial-0001

all: compile test

test: M1.test M9.test

# ESP-IDF 5.3 classic ESP32 (MCPWM/PCNT+RMT+I2S): test M1..M3 with Driver-Types config
test_idf5_3: M1.idf5_3_test M2.idf5_3_test M3.idf5_3_test

%.idf5_3_test:
	STEPPER_CONFIG=1 MOTOR=$* ./seq_04.py $(DEV)
	STEPPER_CONFIG=1 MOTOR=$* ./seq_05.py $(DEV)
	STEPPER_CONFIG=1 MOTOR=$* ./seq_06.py $(DEV)
	STEPPER_CONFIG=1 MOTOR=$* ./seq_01a.py $(DEV)
	STEPPER_CONFIG=1 MOTOR=$* ./seq_01b.py $(DEV)
	STEPPER_CONFIG=1 MOTOR=$* ./seq_02.py $(DEV)
	STEPPER_CONFIG=1 MOTOR=$* ./seq_07a.py $(DEV)
	STEPPER_CONFIG=1 MOTOR=$* ./seq_07b.py $(DEV)
	STEPPER_CONFIG=1 MOTOR=$* ./seq_07c.py $(DEV)
	STEPPER_CONFIG=1 MOTOR=$* ./seq_03.py $(DEV)

%.test:
	MOTOR=$* ./seq_04.py $(DEV)
	MOTOR=$* ./seq_05.py $(DEV)
	MOTOR=$* ./seq_06.py $(DEV)
	MOTOR=$* ./seq_01a.py $(DEV)
	MOTOR=$* ./seq_01b.py $(DEV)
	MOTOR=$* ./seq_02.py $(DEV)
	MOTOR=$* ./seq_07a.py $(DEV)
	MOTOR=$* ./seq_07b.py $(DEV)
	MOTOR=$* ./seq_07c.py $(DEV)
	MOTOR=$* ./seq_03.py $(DEV)

#test_all: .test_arduino_espidf4 .test_arduino_3 .test_idf5 .test_idf6 .test_arduino_latest
test_all: .test_arduino_espidf4 .test_arduino_3 .test_idf5 .test_arduino_latest

.test_arduino_espidf4:
	$(MAKE) compile_arduino_espidf4 && $(MAKE) test && touch .test_arduino_espidf4

.test_arduino_3:
	$(MAKE) compile_arduino_3 && $(MAKE) test && touch .test_arduino_3

# Cannot install/run
#.test_idf4:
#	$(MAKE) compile_idf4 && $(MAKE) test && touch .test_idf4

.test_arduino_latest:
	$(MAKE) compile_arduino_latest && $(MAKE) test && touch .test_arduino_latest

.test_idf5:
	$(MAKE) compile_idf5 && $(MAKE) test && touch .test_idf5

.test_idf6:
	$(MAKE) compile_idf6 && $(MAKE) test && touch .test_idf6

compile: compile_arduino_espidf4

# Arduino 2.0.17 (IDF 4.4.7): 6*MCPWM + 8*RMT
compile_arduino_espidf4 compile_esp32_V6_8_1:
	(cd ../../../pio_dirs/StepperDemo; rm -fR .pio; pio run -e esp32_V6_8_1 -t upload --upload-port /dev/$(DEV))

# Arduino 3.1.1 (IDF 5.3.2): 6*MCPWM + 8*RMT + I2S
compile_arduino_3 compile_esp32_pioarduino_V53_03_11:
	(cd ../../../pio_dirs/StepperDemo; rm -fR .pio; pio run -e esp32_pioarduino_V53_03_11 -t upload --upload-port /dev/$(DEV))

# Arduino 2.0.17 on latest PIO platform 7.1.2 (still IDF 4.4.7)
compile_arduino_latest compile_esp32_V7_1_2:
	(cd ../../../pio_dirs/StepperDemo; rm -fR .pio; pio run -e esp32_V7_1_2 -t upload --upload-port /dev/$(DEV))

# native ESP-IDF 4.4.3
compile_idf4 compile_esp32_idf_V5_3_0:
	(cd ../../scripts; ./build-pio-dirs.sh)
	(cd ../../../pio_espidf/StepperDemo; rm -fR .pio; pio run -e esp32_idf_V5_3_0 -t upload --upload-port /dev/$(DEV))

# native ESP-IDF 5.3.1: 6*MCPWM + 8*RMT + I2S
compile_idf5 compile_esp32_idf_V6_9_0:
	(cd ../../scripts; ./build-pio-dirs.sh)
	(cd ../../../pio_espidf/StepperDemo; rm -fR .pio; pio run -e esp32_idf_V6_9_0 -t upload --upload-port /dev/$(DEV))

# native ESP-IDF 6.1.0 (MCPWM/I2S not enabled in the library yet)
compile_idf6 compile_esp32_idf_V7_1_2:
	(cd ../../scripts; ./build-pio-dirs.sh)
	(cd ../../../pio_espidf/StepperDemo; rm -fR .pio; pio run -e esp32_idf_V7_1_2 -t upload --upload-port /dev/$(DEV))

help:
	@echo "make compile_arduino_espidf4   Arduino 2.0.17 / IDF 4.4.7   pio -e esp32_V6_8_1"
	@echo "make compile_arduino_3         Arduino 3.1.1  / IDF 5.3.2   pio -e esp32_pioarduino_V53_03_11"
	@echo "make compile_arduino_latest    Arduino 2.0.17 / PIO 7.1.2   pio -e esp32_V7_1_2"
	@echo "make compile_idf4              ESP-IDF 4.4.3                pio -e esp32_idf_V5_3_0"
	@echo "make compile_idf5              ESP-IDF 5.3.1                pio -e esp32_idf_V6_9_0"
	@echo "make compile_idf6              ESP-IDF 6.1.0                pio -e esp32_idf_V7_1_2"
	@echo "make test                      M1+M9 on the firmware already on the board"
	@echo "make test_idf5_3               M1..M3 with STEPPER_CONFIG=1 (RMT/I2S/MCPWM)"
	@echo "make test_all                  compile+test: arduino_espidf4, arduino_3, idf5, arduino_latest"

clean:
	rm -f seq*.log .test_*
