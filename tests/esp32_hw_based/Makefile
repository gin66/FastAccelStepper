all: compile test

DEV=ttyUSB0

test: test_M1 test_M7

test_M1:
	MOTOR=M1 ./seq_01.sh $(DEV)
	MOTOR=M1 ./seq_02.sh $(DEV)
	MOTOR=M1 ./seq_03.sh $(DEV)

test_M7:
	MOTOR=M7 ./seq_01.sh $(DEV)
	MOTOR=M7 ./seq_02.sh $(DEV)
	MOTOR=M7 ./seq_03.sh $(DEV)

compile:
	(cd ../../pio_dirs/StepperDemo;rm -fR .pio; pio run -e esp32 -t upload --upload-port /dev/$(DEV))

clean:
	rm seq*.log
