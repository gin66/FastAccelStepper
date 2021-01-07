
CFLAGS=-DTEST -Werror -g -I../src
CXXFLAGS=-DTEST -Werror -g -DF_CPU=16000000 -I../src
LDLIBS=-lm

test: test_01 test_02 test_03 test_04 test_05 test_06 test_07
	./test_01
	./test_02
	./test_03
	./test_04
	./test_05
	./test_06
	./test_07

LIB_H=../src/FastAccelStepper.h ../src/PoorManFloat.h ../src/StepperISR.h ../src/RampGenerator.h

test_01: test_01.o FastAccelStepper.o PoorManFloat.o StepperISR_test.o RampGenerator.o
test_02: test_02.o FastAccelStepper.o PoorManFloat.o StepperISR_test.o RampGenerator.o
test_03: test_03.o PoorManFloat.o
test_04: test_04.o FastAccelStepper.o PoorManFloat.o StepperISR_test.o RampGenerator.o
test_05: test_05.o FastAccelStepper.o PoorManFloat.o StepperISR_test.o RampGenerator.o
test_06: test_06.o FastAccelStepper.o PoorManFloat.o StepperISR_test.o RampGenerator.o
test_07: test_07.o FastAccelStepper.o PoorManFloat.o StepperISR_test.o RampGenerator.o

test_01.o: test_01.cpp $(LIB_H) RampChecker.h stubs.h
test_02.o: test_02.cpp $(LIB_H) RampChecker.h stubs.h
test_03.o: test_03.cpp ../src/PoorManFloat.h stubs.h test_03.h
test_04.o: test_04.cpp $(LIB_H) RampChecker.h stubs.h
test_05.o: test_05.cpp $(LIB_H) RampChecker.h stubs.h
test_06.o: test_06.cpp $(LIB_H) RampChecker.h stubs.h
test_07.o: test_07.cpp $(LIB_H) RampChecker.h stubs.h

FastAccelStepper.o: ../src/FastAccelStepper.cpp ../src/FastAccelStepper.h ../src/PoorManFloat.h ../src/StepperISR.h stubs.h ../src/RampGenerator.h ../src/common.h
	$(COMPILE.cpp) $< -o $@

PoorManFloat.o: ../src/PoorManFloat.cpp ../src/PoorManFloat.h
	$(COMPILE.cpp) $< -o $@

RampGenerator.o: ../src/RampGenerator.cpp ../src/RampGenerator.h ../src/common.h
	$(COMPILE.cpp) $< -o $@

StepperISR_test.o: StepperISR_test.cpp ../src/StepperISR.h ../src/common.h

test_%.o: test_%.cpp stubs.h

VERSION=$(shell git rev-parse --short HEAD)

fmt:
	clang-format --style=Google -i ../src/* test_*.cpp stubs.h ../examples/*/*.ino ../examples/*/test_seq*
	echo $(VERSION)
	sed -i -e 's/#define VERSION.*$$/#define VERSION "post-$(VERSION)"/' ../examples/StepperDemo/StepperDemo.ino

clean:
	rm *.o test_[0-9][0-9] *.gnuplot
