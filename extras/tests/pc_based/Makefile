
PRJ_ROOT=$(shell git rev-parse --show-toplevel)
CFLAGS=-DTEST -Werror -g -I$(PRJ_ROOT)/src
CXXFLAGS=-DTEST -Werror -g -DF_CPU=16000000 -I$(PRJ_ROOT)/src
LDLIBS=-lm -lc

TESTS=$(basename $(wildcard test_??.cpp))

test: $(TESTS) log2_test rmc_test
	./rmc_test
	./log2_test
	rm -f test.log
	$(addsuffix >>test.log &&,$(addprefix ./,$(TESTS))) echo "All tests passed" && rm test.log

png: $(patsubst %.gnuplot,%.gnuplot.png,$(wildcard test_*.gnuplot))

test_%.gnuplot.png: test_%.gnuplot
	gnuplot $<

COV_DIR=coverage_report
COV_FLAGS=-fprofile-instr-generate -fcoverage-mapping
PROFDATA=coverage.profdata
COV_IGNORE=-ignore-filename-regex='(pd_test|StepperISR_test|test_[0-9]|log2_test|rmc_test|ramp_helper|RampChecker|i2s_constants|Log2RepresentationConst|common\.h|test_pc\.h|queue\.h)'
COV_BINS=$(word 1,$(TESTS)) $(addprefix -object ,$(wordlist 2,99,$(TESTS))) -object log2_test -object rmc_test

coverage:
	$(MAKE) clean_coverage
	LLVM_PROFILE_FILE="%p-%m.profraw" $(MAKE) CFLAGS="$(CFLAGS) $(COV_FLAGS)" CXXFLAGS="$(CXXFLAGS) $(COV_FLAGS)" LDLIBS="$(LDLIBS) $(COV_FLAGS)" test
	xcrun llvm-profdata merge -sparse *.profraw -o $(PROFDATA)
	xcrun llvm-cov show $(COV_BINS) \
	     -instr-profile=$(PROFDATA) \
	     -format=html -output-dir=$(COV_DIR) \
	     -show-branches=count -show-branch-summary $(COV_IGNORE)
	xcrun llvm-cov report $(COV_BINS) \
	     -instr-profile=$(PROFDATA) \
	     -show-branch-summary $(COV_IGNORE)
	@echo ""
	@echo "Branch coverage report: $(COV_DIR)/index.html"

clean_coverage:
	rm -f *.profraw *.profdata
	rm -rf $(COV_DIR)

ramp_helper: ramp_helper.o $(LIB_O)
	gcc -o $@ $< $(LIB_O) $(LDLIBS)

ramp_helper.o: ramp_helper.cpp $(SRC_LIB_H)
	g++ -c $(CXXFLAGS) -o $@ $<

LIB_H=FastAccelStepper.h FastAccelStepperEngine.h log2/Log2Representation.h log2/Log2RepresentationConst.h \
	  fas_queue/stepper_queue.h fas_queue/queue.h fas_queue/base.h \
	  fas_ramp/RampGenerator.h fas_ramp/RampControl.h fas_ramp/RampCalculator.h \
	  fas_arch/common.h fas_member/move_timed.h fas_member/add_queue_entry.h pd_test/test_queue.h pd_test/dir_change_pause.h
LIB_O=FastAccelStepper.o FastAccelStepperEngine.o Log2Representation.o StepperISR_test.o \
	  RampGenerator.o RampControl.o RampCalculator.o queue_add_entry.o queue_get_position.o queue_init.o queue_utils.o

SRC_LIB_H=$(addprefix $(PRJ_ROOT)/src/,$(LIB_H))

test_21: test_21.o
	g++ -o $@ $< $(LDLIBS)

test_21.o: test_21.cpp $(PRJ_ROOT)/src/pd_esp32/i2s_fill.cpp $(PRJ_ROOT)/src/pd_esp32/i2s_fill.h $(PRJ_ROOT)/src/pd_esp32/i2s_constants.h
	g++ -c $(CXXFLAGS) -o $@ $<

test_22: test_22.o
	g++ -o $@ $< $(LDLIBS)

test_22.o: test_22.cpp i2s_test_common.h $(PRJ_ROOT)/src/pd_esp32/i2s_fill.cpp $(PRJ_ROOT)/src/pd_esp32/i2s_fill.h $(PRJ_ROOT)/src/pd_esp32/i2s_constants.h
	g++ -c $(CXXFLAGS) -o $@ $<

# test_24 unit-tests the inline FastAccelStepper::moveTimed() and
# addQueueEntry() bodies (see src/fas_member/move_timed.h and
# src/fas_member/add_queue_entry.h) against the test's own StepperQueue. It
# is self-contained: it defines the queue/stepper members used by the inline
# bodies, so it must not be linked against FastAccelStepper.o / queue_add_entry.o.
test_24: test_24.o
	g++ -o $@ $< $(LDLIBS)

# test_25 unit-tests the moveTimed() pause reporting (see
# src/fas_member/move_timed.h): per-call actual_duration semantics, the
# injected pause ticks reported on DirChangePauseInjected / DirPin2msPauseAdded,
# steps=0 inheriting last direction, and prepare_revert XOR.
# It is self-contained like test_24 and must not link LIB_O.
test_25: test_25.o
	g++ -o $@ $< $(LDLIBS)

test_%: test_%.o $(LIB_O)
	gcc -o $@ $< $(LIB_O) $(LDLIBS)

test_%.o: test_%.cpp $(SRC_LIB_H) RampChecker.h
	g++ -c $(CXXFLAGS) -o $@ $<

# FasNAxis headers: test_26 includes them, so a header-only change must rebuild.
test_26.o: naxis_plot.h naxis_sim_port.h naxis_ref.h naxis_html_dump.h \
	$(PRJ_ROOT)/src/FasNAxis.h \
	$(PRJ_ROOT)/src/fas_naxis/remaining.h \
	$(PRJ_ROOT)/src/fas_naxis/ramp_law.h \
	$(PRJ_ROOT)/src/fas_naxis/ramp_map.h \
	$(PRJ_ROOT)/src/fas_naxis/dda.h \
	$(PRJ_ROOT)/src/fas_naxis/linear.h \

# Step 10: the HTML viewer is behind -DFAS_NAXIS_TRACE. This target rebuilds
# test_26 with the macro so the F5 run also writes extras/n_axes/tests/out/F5.html.
# Not part of `make test`; the gnuplot file is the plot, the HTML is extra.
.PHONY: test_26_trace
test_26_trace: $(LIB_O)
	g++ -c $(CXXFLAGS) -DFAS_NAXIS_TRACE -DNAXIS_HTML_ROOT=\"$(PRJ_ROOT)/extras/n_axes\" -o test_26.o test_26.cpp
	g++ -o test_26 test_26.o $(LIB_O) $(LDLIBS)
	./test_26

# Step 2b theory-probe mutations. This is a test (rebuilds test_26 with a
# wrong model and requires FAIL), not a library script. Not part of `make test`.
.PHONY: mutations
mutations:
	sh ./prove_mutations.sh

log2_test: log2_test.o Log2Representation.o
log2_test.o: log2_test.cpp $(PRJ_ROOT)/src/log2/Log2Representation.h test_03.h

rmc_test: rmc_test.o Log2Representation.o RampCalculator.o
rmc_test.o: rmc_test.cpp $(PRJ_ROOT)/src/log2/Log2Representation.h $(PRJ_ROOT)/src/fas_ramp/RampCalculator.h test_03.h

FastAccelStepper.o: $(PRJ_ROOT)/src/FastAccelStepper.cpp $(SRC_LIB_H)
	$(COMPILE.cpp) $< -o $@

FastAccelStepperEngine.o: $(PRJ_ROOT)/src/FastAccelStepperEngine.cpp $(SRC_LIB_H)
	$(COMPILE.cpp) $< -o $@

Log2Representation.o: $(PRJ_ROOT)/src/log2/Log2Representation.cpp $(PRJ_ROOT)/src/log2/Log2Representation.h
	$(COMPILE.cpp) $< -o $@

RampGenerator.o: $(PRJ_ROOT)/src/fas_ramp/RampGenerator.cpp $(SRC_LIB_H)
	$(COMPILE.cpp) $< -o $@

RampControl.o: $(PRJ_ROOT)/src/fas_ramp/RampControl.cpp $(SRC_LIB_H)
	$(COMPILE.cpp) $< -o $@

RampCalculator.o: $(PRJ_ROOT)/src/fas_ramp/RampCalculator.cpp $(SRC_LIB_H)
	$(COMPILE.cpp) $< -o $@

queue_add_entry.o: $(PRJ_ROOT)/src/fas_queue/queue_add_entry.cpp $(SRC_LIB_H)
	$(COMPILE.cpp) $< -o $@

queue_get_position.o: $(PRJ_ROOT)/src/fas_queue/queue_get_position.cpp $(SRC_LIB_H)
	$(COMPILE.cpp) $< -o $@

queue_init.o: $(PRJ_ROOT)/src/fas_queue/queue_init.cpp $(SRC_LIB_H)
	$(COMPILE.cpp) $< -o $@

queue_utils.o: $(PRJ_ROOT)/src/fas_queue/queue_utils.cpp $(SRC_LIB_H)
	$(COMPILE.cpp) $< -o $@

StepperISR_test.o: StepperISR_test.cpp $(SRC_LIB_H)

VERSION=$(shell git rev-parse --short HEAD)

clean: clean_coverage
	rm -f *.o test_[0-9][0-9] test_26_FAS_NAXIS_* *.gnuplot *.png log2_test rmc_test test.log
