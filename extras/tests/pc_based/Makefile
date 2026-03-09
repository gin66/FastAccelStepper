
PRJ_ROOT=$(shell git rev-parse --show-toplevel)
CFLAGS=-DTEST -Werror -g -I$(PRJ_ROOT)/src
CXXFLAGS=-DTEST -Werror -g -DF_CPU=16000000 -I$(PRJ_ROOT)/src
LDLIBS=-lm -lc

TESTS=$(basename $(wildcard test_??.cpp))

test: $(TESTS) log2_test rmc_test
	./rmc_test
	./log2_test
	rm -f test.log
	$(addsuffix >>test.log &&,$(addprefix ./,$(TESTS))) echo "All tests passed"

ramp_helper: ramp_helper.o $(LIB_O)
	gcc -o $@ $< $(LIB_O) $(LDLIBS)

ramp_helper.o: ramp_helper.cpp $(SRC_LIB_H) stubs.h
	g++ -c $(CXXFLAGS) -o $@ $<

LIB_H=FastAccelStepper.h FastAccelStepperEngine.h log2/Log2Representation.h log2/Log2RepresentationConst.h \
	  fas_queue/stepper_queue.h fas_queue/queue.h fas_queue/base.h \
	  fas_ramp/RampGenerator.h fas_ramp/RampControl.h fas_ramp/RampCalculator.h \
	  fas_arch/common.h pd_test/test_queue.h
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

test_%: test_%.o $(LIB_O)
	gcc -o $@ $< $(LIB_O) $(LDLIBS)

test_%.o: test_%.cpp $(SRC_LIB_H) RampChecker.h stubs.h
	g++ -c $(CXXFLAGS) -o $@ $<

log2_test: log2_test.o Log2Representation.o
log2_test.o: log2_test.cpp $(PRJ_ROOT)/src/log2/Log2Representation.h stubs.h test_03.h

rmc_test: rmc_test.o Log2Representation.o RampCalculator.o
rmc_test.o: rmc_test.cpp $(PRJ_ROOT)/src/log2/Log2Representation.h $(PRJ_ROOT)/src/fas_ramp/RampCalculator.h stubs.h test_03.h

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

clean:
	rm -f *.o test_[0-9][0-9] *.gnuplot log2_test rmc_test test.log
