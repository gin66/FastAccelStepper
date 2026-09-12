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
#   make test_idf5_3                  IDF 5.3 Driver-Types config: M1=RMT, M2=I2S, M3=MCPWM
#   make test_idf6_1                  IDF 6.1 Driver-Types config: M1=RMT, M2=I2S (MCPWM off)
#   make test_all                     flash+test the usual set (skips ones already stamped)
#   make clean                        drop seq_*.log, test_all.log, and stamps
#
# `pio run -e` names must exist in extras/ci/platformio.ini
# (from extras/ci/build_matrix.yaml). PIO version != ESP-IDF version:
#   https://github.com/platformio/platform-espressif32/releases
#
# Default DONT_CARE motors (make test):
#   Arduino 2.x / IDF 4: M1=MCPWM, M9=RMT
#   Arduino 3.x / IDF 5: M1=RMT,   M9=I2S   (MCPWM is M3 with STEPPER_CONFIG=1)

.PHONY: all test test_all test_all_begin test_idf5_3 test_idf6_1 clean compile help \
	compile_arduino_espidf4 compile_arduino_3 compile_arduino_latest \
	compile_idf4 compile_idf5 compile_idf6 \
	compile_esp32_V6_8_1 compile_esp32_pioarduino_V53_03_11 compile_esp32_V7_1_2 \
	compile_esp32_idf_V5_3_0 compile_esp32_idf_V6_9_0 compile_esp32_idf_V7_1_2

# Test scripts use uv shebang with pyserial (no grabserial needed).

# macos:
DEV ?= cu.usbserial-0001
# Firmware under test; test_all sets this per config. Standalone `make test` = already flashed.
DUT ?= flashed

all: compile test

test: M1.test M9.test

# $1 = seq_xx.py. Logs "arduino_espidf4  M1 seq_04 passed" (or FAILED).
define RUN_SEQ
	DUT=$(DUT) MOTOR=$* ./$(1) $(DEV) && printf '%s  %s  %s %s passed\n' "$$(date '+%Y-%m-%d %H:%M:%S')" "$(DUT)" "$*" "$(1:.py=)" | tee -a test_all.log || { printf '%s  %s  %s %s FAILED\n' "$$(date '+%Y-%m-%d %H:%M:%S')" "$(DUT)" "$*" "$(1:.py=)" | tee -a test_all.log; exit 1; }
endef
define RUN_SEQ_CFG1
	DUT=$(DUT) STEPPER_CONFIG=1 MOTOR=$* ./$(1) $(DEV) && printf '%s  %s  %s %s config=1 passed\n' "$$(date '+%Y-%m-%d %H:%M:%S')" "$(DUT)" "$*" "$(1:.py=)" | tee -a test_all.log || { printf '%s  %s  %s %s config=1 FAILED\n' "$$(date '+%Y-%m-%d %H:%M:%S')" "$(DUT)" "$*" "$(1:.py=)" | tee -a test_all.log; exit 1; }
endef

# ESP-IDF 5.3 classic ESP32 (MCPWM/PCNT+RMT+I2S): test M1..M3 with Driver-Types config
test_idf5_3: DUT := driver_types
test_idf5_3: M1.idf5_3_test M2.idf5_3_test M3.idf5_3_test

%.idf5_3_test:
	$(call RUN_SEQ_CFG1,seq_04.py)
	$(call RUN_SEQ_CFG1,seq_05.py)
	$(call RUN_SEQ_CFG1,seq_06.py)
	$(call RUN_SEQ_CFG1,seq_01a.py)
	$(call RUN_SEQ_CFG1,seq_01b.py)
	$(call RUN_SEQ_CFG1,seq_02.py)
	$(call RUN_SEQ_CFG1,seq_07a.py)
	$(call RUN_SEQ_CFG1,seq_07b.py)
	$(call RUN_SEQ_CFG1,seq_07c.py)
	$(call RUN_SEQ_CFG1,seq_03.py)

# ESP-IDF 6.1.0 (esp32_idf_V7_1_2): MCPWM not enabled yet, so the
# Driver-Types config drives M1=RMT, M2=I2S. Run it with STEPPER_CONFIG=1.
test_idf6_1: DUT := idf6_cfg1
test_idf6_1: M1.idf6_1_test M2.idf6_1_test

%.idf6_1_test:
	$(call RUN_SEQ_CFG1,seq_04.py)
	$(call RUN_SEQ_CFG1,seq_05.py)
	$(call RUN_SEQ_CFG1,seq_06.py)
	$(call RUN_SEQ_CFG1,seq_01a.py)
	$(call RUN_SEQ_CFG1,seq_01b.py)
	$(call RUN_SEQ_CFG1,seq_02.py)
	$(call RUN_SEQ_CFG1,seq_07a.py)
	$(call RUN_SEQ_CFG1,seq_07b.py)
	$(call RUN_SEQ_CFG1,seq_07c.py)
	$(call RUN_SEQ_CFG1,seq_03.py)

%.test:
	$(call RUN_SEQ,seq_04.py)
	$(call RUN_SEQ,seq_05.py)
	$(call RUN_SEQ,seq_06.py)
	$(call RUN_SEQ,seq_01a.py)
	$(call RUN_SEQ,seq_01b.py)
	$(call RUN_SEQ,seq_02.py)
	$(call RUN_SEQ,seq_07a.py)
	$(call RUN_SEQ,seq_07b.py)
	$(call RUN_SEQ,seq_07c.py)
	$(call RUN_SEQ,seq_03.py)

test_all: test_all_begin .test_arduino_espidf4 .test_arduino_3 .test_idf5 .test_idf6_1 .test_arduino_latest
	@printf '%s  DONE  test_all\n' "$$(date '+%Y-%m-%d %H:%M:%S')" | tee -a test_all.log

test_all_begin:
	@printf '\n# test_all %s\n' "$$(date '+%Y-%m-%d %H:%M:%S')" | tee -a test_all.log
	@for s in arduino_espidf4 arduino_3 idf5 idf6_1 arduino_latest; do \
		if [ -e .test_$$s ]; then printf '# skip %s (stamp exists)\n' "$$s" | tee -a test_all.log; fi; \
	done

# $1=dut name  $2=pio env  $3=human description (logged before compile/test)
define RECORD_DUT
	printf '%s  DUT %-18s  pio=%s  %s\n' "$$(date '+%Y-%m-%d %H:%M:%S')" "$(1)" "$(2)" "$(3)" | tee -a test_all.log
endef
define RECORD_PASS
	printf '%s  PASS %-18s  pio=%s\n' "$$(date '+%Y-%m-%d %H:%M:%S')" "$(1)" "$(2)" | tee -a test_all.log
endef

.test_arduino_espidf4:
	$(call RECORD_DUT,arduino_espidf4,esp32_V6_8_1,Arduino 2.0.17 / IDF 4.4.7)
	$(MAKE) compile_arduino_espidf4 && $(MAKE) test DUT=arduino_espidf4
	$(call RECORD_PASS,arduino_espidf4,esp32_V6_8_1)
	touch .test_arduino_espidf4

.test_arduino_3:
	$(call RECORD_DUT,arduino_3,esp32_pioarduino_V53_03_11,Arduino 3.1.1 / IDF 5.3.2)
	$(MAKE) compile_arduino_3 && $(MAKE) test DUT=arduino_3
	$(call RECORD_PASS,arduino_3,esp32_pioarduino_V53_03_11)
	touch .test_arduino_3

# Cannot install/run
#.test_idf4:
#	$(call RECORD_DUT,idf4,esp32_idf_V5_3_0,ESP-IDF 4.4.3)
#	$(MAKE) compile_idf4 && $(MAKE) test DUT=idf4
#	$(call RECORD_PASS,idf4,esp32_idf_V5_3_0)
#	touch .test_idf4

.test_arduino_latest:
	$(call RECORD_DUT,arduino_latest,esp32_V7_1_2,Arduino 2.0.17 / PIO 7.1.2)
	$(MAKE) compile_arduino_latest && $(MAKE) test DUT=arduino_latest
	$(call RECORD_PASS,arduino_latest,esp32_V7_1_2)
	touch .test_arduino_latest

.test_idf5:
	$(call RECORD_DUT,idf5,esp32_idf_V6_9_0,ESP-IDF 5.3.1)
	$(MAKE) compile_idf5 && $(MAKE) test DUT=idf5
	$(call RECORD_PASS,idf5,esp32_idf_V6_9_0)
	touch .test_idf5

.test_idf6_1:
	$(call RECORD_DUT,idf6_1,esp32_idf_V7_1_2,ESP-IDF 6.1.0 / Driver-Types M1=RMT M2=I2S)
	$(MAKE) compile_idf6 && $(MAKE) test_idf6_1
	$(call RECORD_PASS,idf6_1,esp32_idf_V7_1_2)
	touch .test_idf6_1

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

# native ESP-IDF 6.1.0 (RMT + I2S enabled, MCPWM/PCNT off for now)
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
	@echo "make test_idf5_3               IDF 5.3 M1..M3 with STEPPER_CONFIG=1 (RMT/I2S/MCPWM)"
	@echo "make test_idf6_1               IDF 6.1 M1..M2 with STEPPER_CONFIG=1 (RMT/I2S, MCPWM off)"
	@echo "make test_all                  compile+test: arduino_espidf4, arduino_3, idf5, idf6_1, arduino_latest"
	@echo "Passed configs: ls .test_*   and  cat test_all.log"

clean:
	rm -f seq*.log test_all.log .test_*
