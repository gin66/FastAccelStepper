all: compile test

DEV=ttyUSB0

test: M1.test M7.test

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

compile:
	(cd ../../../pio_dirs/StepperDemo;rm -fR .pio; pio run -e esp32_V6_8_1 -t upload --upload-port /dev/$(DEV))

compile_idf4:
	(cd ../../../pio_espidf/StepperDemo;rm -fR .pio; pio run -e esp32_idf_V5_3_0 -t upload --upload-port /dev/$(DEV))

compile_idf5:
	(cd ../../../pio_espidf/StepperDemo;rm -fR .pio; pio run -e esp32_idf_V6_8_1 -t upload --upload-port /dev/$(DEV))

clean:
	rm -f seq*.log
