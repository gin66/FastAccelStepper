
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

LIB_H=FastAccelStepper.h Log2Representation.h StepperISR.h \
	  RampGenerator.h RampControl.h RampCalculator.h \
	  fas_arch/common.h
LIB_O=FastAccelStepper.o Log2Representation.o StepperISR_test.o \
	  RampGenerator.o RampControl.o RampCalculator.o StepperISR.o

SRC_LIB_H=$(addprefix $(PRJ_ROOT)/src/,$(LIB_H))

test_%: test_%.o $(LIB_O)
	gcc -o $@ $< $(LIB_O) $(LDLIBS)

test_%.o: test_%.cpp $(SRC_LIB_H) RampChecker.h stubs.h
	g++ -c $(CXXFLAGS) -o $@ $<

log2_test: log2_test.o Log2Representation.o
log2_test.o: log2_test.cpp $(PRJ_ROOT)/src/Log2Representation.h stubs.h test_03.h

rmc_test: rmc_test.o Log2Representation.o RampCalculator.o
rmc_test.o: rmc_test.cpp $(PRJ_ROOT)/src/Log2Representation.h $(PRJ_ROOT)/src/RampCalculator.h stubs.h test_03.h

FastAccelStepper.o: $(PRJ_ROOT)/src/FastAccelStepper.cpp $(SRC_LIB_H)
	$(COMPILE.cpp) $< -o $@

Log2Representation.o: $(PRJ_ROOT)/src/Log2Representation.cpp $(PRJ_ROOT)/src/Log2Representation.h
	$(COMPILE.cpp) $< -o $@

RampGenerator.o: $(PRJ_ROOT)/src/RampGenerator.cpp $(SRC_LIB_H)
	$(COMPILE.cpp) $< -o $@

RampControl.o: $(PRJ_ROOT)/src/RampControl.cpp $(SRC_LIB_H)
	$(COMPILE.cpp) $< -o $@

RampCalculator.o: $(PRJ_ROOT)/src/RampCalculator.cpp $(SRC_LIB_H)
	$(COMPILE.cpp) $< -o $@

StepperISR.o: $(PRJ_ROOT)/src/StepperISR.cpp $(SRC_LIB_H)
	$(COMPILE.cpp) $< -o $@

StepperISR_test.o: StepperISR_test.cpp $(SRC_LIB_H)

VERSION=$(shell git rev-parse --short HEAD)

clean:
	rm -f *.o test_[0-9][0-9] *.gnuplot log2_test rmc_test test.log
