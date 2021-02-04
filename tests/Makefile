
CFLAGS=-DTEST -Werror -g -I../src
CXXFLAGS=-DTEST -Werror -g -DF_CPU=16000000 -I../src
LDLIBS=-lm -lc

TESTS=$(basename $(wildcard test_??.cpp))

test: $(TESTS) pmf_test
	./pmf_test
	$(addsuffix &&,$(addprefix ./,$(TESTS))) echo "All tests passed"

LIB_H=../src/FastAccelStepper.h ../src/PoorManFloat.h ../src/StepperISR.h ../src/RampGenerator.h 

test_%: test_%.o FastAccelStepper.o PoorManFloat.o StepperISR_test.o RampGenerator.o
	gcc -o $@ $? $(LDLIBS)

test_%.o: test_%.cpp $(LIB_H) RampChecker.h stubs.h

pmf_test: pmf_test.o PoorManFloat.o
pmf_test.o: pmf_test.cpp ../src/PoorManFloat.h stubs.h test_03.h

FastAccelStepper.o: ../src/FastAccelStepper.cpp ../src/FastAccelStepper.h ../src/PoorManFloat.h ../src/StepperISR.h stubs.h ../src/RampGenerator.h ../src/common.h
	$(COMPILE.cpp) $< -o $@

PoorManFloat.o: ../src/PoorManFloat.cpp ../src/PoorManFloat.h
	$(COMPILE.cpp) $< -o $@

RampGenerator.o: ../src/RampGenerator.cpp ../src/RampGenerator.h ../src/common.h
	$(COMPILE.cpp) $< -o $@

RampCalculator.o: ../src/RampCalculator.cpp ../src/RampCalculator.h ../src/common.h ../src/FastAccelStepper.h ../src/StepperISR.h
	$(COMPILE.cpp) $< -o $@

StepperISR_test.o: StepperISR_test.cpp ../src/StepperISR.h ../src/common.h ../src/FastAccelStepper.h ../src/StepperISR.h

VERSION=$(shell git rev-parse --short HEAD)

fmt:
	clang-format --style=Google -i ../src/* *.ino *.cpp *.h ../examples/*/*.ino ../examples/*/test_seq*
	echo $(VERSION)
	sed -i -e 's/#define VERSION.*$$/#define VERSION "post-$(VERSION)"/' ../examples/StepperDemo/StepperDemo.ino

clean:
	rm *.o test_[0-9][0-9] *.gnuplot
