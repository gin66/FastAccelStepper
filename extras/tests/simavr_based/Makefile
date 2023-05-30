ifndef SILENCE
	SILENCE=0
endif

PRJ_ROOT=$(shell git rev-parse --show-toplevel)
TESTS=$(wildcard test_*)
TESTS_SD=$(wildcard test_*sd_*)

TEST_FILES=$(addsuffix /.tested,$(TESTS))
SD_SRC_DIRS=$(addsuffix /src/.dir,$(TESTS_SD))

SRC=$(wildcard ../../src/*)

test: $(TEST_FILES) pmf externalCall 

pmf:
	make -C test_pmf

externalCall:
	make -C test_externalCall

%/.tested:	$(SRC) run_avr %/expect.txt %/platformio.ini
	make SILENCE=$(SILENCE)  -C $(dir $@)

makefiles: $(addsuffix /Makefile,$(TESTS))

%/Makefile:
	cd $(dir $@); ln -s ../Makefile.test Makefile

links: $(SD_SRC_DIRS)

%/src/.dir:
	mkdir -p $(dir $@)
	cd $(dir $@); ln -sf $(PRJ_ROOT)/examples/StepperDemo/* .

run_avr: simavr/simavr/run_avr
	ln -s simavr/simavr/run_avr .

simavr/simavr/run_avr: simavr/simavr/sim/run_avr.c
	make -C simavr build-simavr

simavr/simavr/sim/run_avr.c:
	git clone https://github.com/gin66/simavr.git
	# these versions work out of the box
	#(cd simavr;git checkout	8d660ce)
	#(cd simavr;git checkout	a56b550)
	#(cd simavr;git checkout	c6d5375)
	#(cd simavr;git checkout	9ed8eb6)
	#(cd simavr;git checkout	b653a13)
	#
	# these versions work only with patch applied
	#(cd simavr;git checkout	d2aaebd)
	#(cd simavr;git checkout	ea4c450)
	#(cd simavr;git checkout	d6ba83c)
	#(cd simavr;git checkout	ce6738b)
	#(cd simavr;git checkout	a5634af)
	#(cd simavr;git checkout	13363d5)
	#(cd simavr;git checkout	1379eed)
	#
	# these versions cannot be reverse patched
	#(cd simavr;git checkout	c96ea61) # fails
	#(cd simavr;git checkout	4d1750b) # fails
	#(cd simavr;git checkout	adfce91) # fails
	#
	# test using:
	#     make proper run_avr;make -C test_sd_01b_2560t3/
	#
	# patch derived by:
	#     git diff b653a13..d2aaebd >../../simavr.patch
	#
	# select current HEAD as of now
	(cd simavr;git checkout	7003af)
	# apply patch, which is now minimal:
#	(cd simavr;patch -p1 -t <../simavr.patch)
	(cd simavr;patch -p1 -t <../simavr.patch2)

proper: clean
	rm -f run_avr
	rm -fR simavr

clean:
	rm -fR */.pio */.tested */x.vcd */result.txt
	find . -type l -delete
	find . -type d -empty -delete
	make links makefiles

