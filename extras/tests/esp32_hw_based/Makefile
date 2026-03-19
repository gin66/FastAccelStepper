.PHONY: all test test_all test_idf5_3 clean compile \
	compile_arduino_espidf4 compile_arduino_espidf5 \
	compile_arduino_espidf_V6_13_0 compile_idf4 compile_idf5 compile_idf5_V6_13_0 \
	.test_espidf4 .test_espidf5 .test_arduino_espidf_V6_13_0 .test_idf5 .test_idf5_V6_13_0

all: compile test

# Test scripts use uv shebang with pyserial (no grabserial needed).

# macos:
DEV=cu.usbserial-0001

# for espidf5 ESP32 classic has 6*mcpwm+8*rmt, S2/S3/C3/C6 have only RMT+I2S
# for espidf4 there is 6*mcpwm+8*rmt (ESP32) or MCPWM/PCNT+RMT (S3)
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

#test_all: .test_espidf4 .test_espidf5 .test_idf5 .test_idf5_V6_13_0 .test_arduino_espidf_V6_13_0
test_all: .test_espidf4 .test_espidf5 .test_idf5 .test_arduino_espidf_V6_13_0

.test_espidf4:
	make compile_arduino_espidf4 && make test && touch .test_espidf4

.test_espidf5:
	make compile_arduino_espidf5 && make test && touch .test_espidf5

# Cannot install/run
#.test_idf4:
#	make compile_idf4 && make test && touch .test_idf4

.test_arduino_espidf_V6_13_0:
	make compile_arduino_espidf_V6_13_0 && make test && touch .test_arduino_espidf_V6_13_0

.test_idf5:
	make compile_idf5 && make test && touch .test_idf5

.test_idf5_V6_13_0:
	make compile_idf5_V6_13_0 && make test && touch .test_idf5_V6_13_0

compile: compile_arduino_espidf4

compile_arduino_espidf4:
	(cd ../../../pio_dirs/StepperDemo;rm -fR .pio; pio run -e esp32_V6_8_1 -t upload --upload-port /dev/$(DEV))

compile_arduino_espidf5:
	(cd ../../../pio_dirs/StepperDemo;rm -fR .pio; pio run -e esp32_pioarduino_V53_03_11 -t upload --upload-port /dev/$(DEV))

compile_arduino_espidf_V6_13_0:
	(cd ../../../pio_dirs/StepperDemo;rm -fR .pio; pio run -e esp32_V6_13_0 -t upload --upload-port /dev/$(DEV))

compile_idf4:
	(cd ../../scripts;./build-pio-dirs.sh)
	(cd ../../../pio_espidf/StepperDemo;rm -fR .pio; pio run -e esp32_idf_V5_3_0 -t upload --upload-port /dev/$(DEV))

compile_idf5:
	(cd ../../scripts;./build-pio-dirs.sh)
	(cd ../../../pio_espidf/StepperDemo;rm -fR .pio; pio run -e esp32_idf_V6_8_1 -t upload --upload-port /dev/$(DEV))

compile_idf5_V6_13_0:
	(cd ../../scripts;./build-pio-dirs.sh)
	(cd ../../../pio_espidf/StepperDemo;rm -fR .pio; pio run -e esp32_idf_V6_13_0 -t upload --upload-port /dev/$(DEV))

clean:
	rm -f seq*.log .test_*
