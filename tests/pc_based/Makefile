
CFLAGS=-DTEST -Werror -g -I../../src
CXXFLAGS=-DTEST -Werror -g -DF_CPU=16000000 -I../../src
LDLIBS=-lm -lc

TESTS=$(basename $(wildcard test_??.cpp))

test: $(TESTS) pmf_test rmc_test
	./rmc_test
	./pmf_test
	$(addsuffix &&,$(addprefix ./,$(TESTS))) echo "All tests passed"

LIB_H=FastAccelStepper.h PoorManFloat.h StepperISR.h RampGenerator.h RampCalculator.h common.h
LIB_O=FastAccelStepper.o PoorManFloat.o StepperISR_test.o RampGenerator.o  RampCalculator.o

SRC_LIB_H=$(addprefix ../../src/,$(LIB_H))

test_%: test_%.o $(LIB_O)
	gcc -o $@ $< $(LIB_O) $(LDLIBS)

test_%.o: test_%.cpp $(SRC_LIB_H) RampChecker.h stubs.h
	g++ -c $(CXXFLAGS) -o $@ $<

pmf_test: pmf_test.o PoorManFloat.o
pmf_test.o: pmf_test.cpp ../../src/PoorManFloat.h stubs.h test_03.h

rmc_test: rmc_test.o PoorManFloat.o RampCalculator.o
rmc_test.o: rmc_test.cpp ../../src/PoorManFloat.h ../../src/RampCalculator.h stubs.h test_03.h

FastAccelStepper.o: ../../src/FastAccelStepper.cpp $(SRC_LIB_H)
	$(COMPILE.cpp) $< -o $@

PoorManFloat.o: ../../src/PoorManFloat.cpp ../../src/PoorManFloat.h
	$(COMPILE.cpp) $< -o $@

RampGenerator.o: ../../src/RampGenerator.cpp $(SRC_LIB_H)
	$(COMPILE.cpp) $< -o $@

RampCalculator.o: ../../src/RampCalculator.cpp $(SRC_LIB_H)
	$(COMPILE.cpp) $< -o $@

StepperISR_test.o: StepperISR_test.cpp $(SRC_LIB_H)

VERSION=$(shell git rev-parse --short HEAD)

fmt:
	clang-format --style=Google -i ../../src/* *.ino *.cpp *.h ../../examples/*/*.ino ../../examples/*/test_seq*
	echo $(VERSION)
	sed -i -e 's/#define VERSION.*$$/#define VERSION "post-$(VERSION)"/' ../../examples/StepperDemo/StepperDemo.ino

clean:
	rm *.o test_[0-9][0-9] *.gnuplot pmf_test rmc_test
