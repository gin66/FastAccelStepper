.PHONY: all test test_all clean compile compile_arduino_espidf4 compile_arduino_espidf5 \
	compile_arduino_espidf_V6_13_0 compile_idf4 compile_idf5 compile_idf5_V6_13_0 \
	.test_espidf4 .test_espidf5 .test_arduino_espidf_V6_13_0 .test_idf5 .test_idf5_V6_13_0

all: compile test

# grabserial is a python tool.

# macos:
DEV=cu.usbserial-0001

# for espidf4 there is 6*mcpwm+8*rmt => M1 is mcpwn and M9 is rmt
# for espidf5 there is 8*rmt+2*i2s => M1 is rmt and M9 is i2s
test: M1.test M9.test

%.test:
	MOTOR=$* ./seq_04.sh $(DEV)
	MOTOR=$* ./seq_05.sh $(DEV)
	MOTOR=$* ./seq_06.sh $(DEV)
	MOTOR=$* ./seq_01a.sh $(DEV)
	MOTOR=$* ./seq_01b.sh $(DEV)
	MOTOR=$* ./seq_02.sh $(DEV)
	MOTOR=$* ./seq_07a.sh $(DEV)
	MOTOR=$* ./seq_07b.sh $(DEV)
	MOTOR=$* ./seq_07c.sh $(DEV)
	MOTOR=$* ./seq_03.sh $(DEV)

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
