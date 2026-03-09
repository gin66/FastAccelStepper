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

compile: compile_arduino_espidf4

compile_arduino_espidf4:
	(cd ../../../pio_dirs/StepperDemo;rm -fR .pio; pio run -e esp32_V6_8_1 -t upload --upload-port /dev/$(DEV))

compile_arduino_espidf5:
	(cd ../../../pio_dirs/StepperDemo;rm -fR .pio; pio run -e esp32_pioarduino_V53_03_11 -t upload --upload-port /dev/$(DEV))

compile_idf4:
	(cd ../../scripts;./build-pio-dirs.sh)
	(cd ../../../pio_espidf/StepperDemo;rm -fR .pio; pio run -e esp32_idf_V5_3_0 -t upload --upload-port /dev/$(DEV))

compile_idf5:
	(cd ../../scripts;./build-pio-dirs.sh)
	(cd ../../../pio_espidf/StepperDemo;rm -fR .pio; pio run -e esp32_idf_V6_8_1 -t upload --upload-port /dev/$(DEV))

clean:
	rm -f seq*.log
