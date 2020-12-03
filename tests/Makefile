
CFLAGS=-DTEST -Werror -g
CXXFLAGS=-DTEST -Werror -g -DF_CPU=16000000
LDLIBS=-lm

test: test_01 test_02 test_03 test_04 test_05
	./test_01
	./test_02
	./test_03
	./test_04
	./test_05

LIB_H=FastAccelStepper.h PoorManFloat.h StepperISR.h RampGenerator.h

test_01: test_01.o FastAccelStepper.o PoorManFloat.o StepperISR_test.o RampGenerator.o
test_02: test_02.o FastAccelStepper.o PoorManFloat.o StepperISR_test.o RampGenerator.o
test_03: test_03.o PoorManFloat.o
test_04: test_04.o FastAccelStepper.o PoorManFloat.o StepperISR_test.o RampGenerator.o
test_05: test_05.o FastAccelStepper.o PoorManFloat.o StepperISR_test.o RampGenerator.o

test_01.o: test_01.cpp $(LIB_H) RampChecker.h stubs.h
test_02.o: test_02.cpp $(LIB_H) RampChecker.h stubs.h
test_03.o: test_03.cpp PoorManFloat.h stubs.h test_03.h
test_04.o: test_04.cpp $(LIB_H) RampChecker.h stubs.h
test_05.o: test_05.cpp $(LIB_H) RampChecker.h stubs.h

FastAccelStepper.o: FastAccelStepper.cpp FastAccelStepper.h PoorManFloat.h StepperISR.h stubs.h RampGenerator.h common.h

PoorManFloat.o: PoorManFloat.cpp PoorManFloat.h

StepperISR_test.o: StepperISR_test.cpp StepperISR.h common.h

RampGenerator.o: RampGenerator.cpp RampGenerator.h common.h

test_%.o: test_%.cpp stubs.h

FastAccelStepper.cpp: symlinks
FastAccelStepper.h: symlinks
PoorManFloat.cpp: symlinks
PoorManFloat.h: symlinks
StepperISR.h: symlinks
RampGenerator.h: symlinks
RampGenerator.cpp: symlinks

symlinks:
	ln -sf ../src/* .

fmt:
	clang-format --style=Google -i ../src/* test_*.cpp stubs.h ../examples/*/*.ino

clean:
	rm *.o test_[0-9][0-9] *.gnuplot
