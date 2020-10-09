
CFLAGS=-DTEST -Werror -g
CXXFLAGS=-DTEST -Werror -g -DF_CPU=16000000
LDLIBS=-lm

test: test_01 test_02 test_03 test_04 test_05
	./test_01
	./test_02
	./test_03
	./test_04
	./test_05

test_01: test_01.cpp stubs.h FastAccelStepper.o PoorManFloat.o StepperISR_test.o RampGenerator.o
test_02: test_02.cpp stubs.h FastAccelStepper.o PoorManFloat.o StepperISR_test.o RampGenerator.o
test_03: test_03.cpp stubs.h PoorManFloat.o
test_04: test_04.cpp stubs.h FastAccelStepper.o PoorManFloat.o StepperISR_test.o RampGenerator.o
test_05: test_05.cpp stubs.h FastAccelStepper.o PoorManFloat.o StepperISR_test.o RampGenerator.o

FastAccelStepper.o: FastAccelStepper.cpp FastAccelStepper.h PoorManFloat.h StepperISR.h stubs.h RampGenerator.h

PoorManFloat.o: PoorManFloat.cpp PoorManFloat.h

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
