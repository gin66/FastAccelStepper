
TESTS=$(wildcard test_*)
TESTS_SD=$(wildcard test_*sd_*)

TEST_FILES=$(addsuffix /.tested,$(TESTS))
SD_SRC_DIRS=$(addsuffix /src/.dir,$(TESTS_SD))

SRC=$(wildcard ../src/*)

test: $(TEST_FILES)

%/.tested:	$(SRC) run_avr %/expect.txt %/platformio.ini
	make -C $(dir $@) -f ../Makefile.test

links: $(SD_SRC_DIRS)

%/src/.dir:
	mkdir -p $(dir $@)
	cd $(dir $@); ln -sf ../../../examples/StepperDemo/* .

run_avr: simavr/simavr/run_avr
	ln -s simavr/simavr/run_avr .

simavr/simavr/run_avr:
	git clone https://github.com/gin66/simavr.git
	make -C simavr build-simavr

clean:
	rm -fR */.pio */.tested */x.vcd */result.txt
	find . -type l -delete
	find . -type d -empty -delete
