all: compile test

test:
	./seq_01.sh
	./seq_02.sh
	./seq_03.sh

compile:
	(cd ../../pio_dirs/StepperDemo; pio run -e esp32 -t upload)

clean:
	rm seq*.log
