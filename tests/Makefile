
CFLAGS=-DTEST -Werror
CXXFLAGS=-DTEST -Werror
LDLIBS=-lm

test: test_01 test_02 test_03
	./test_03
	./test_01
	./test_02

test_01: test_01.cpp stubs.h FastAccelStepper.o PoorManFloat.o
test_02: test_02.cpp stubs.h FastAccelStepper.o PoorManFloat.o
test_03: test_03.cpp stubs.h FastAccelStepper.o PoorManFloat.o

FastAccelStepper.o: FastAccelStepper.cpp FastAccelStepper.h PoorManFloat.h stubs.h

PoorManFloat.o: PoorManFloat.cpp PoorManFloat.h

test_%.o: test_%.cpp stubs.h


fmt:
	clang-format --style=Google -i ../src/* test_*.cpp stubs.h ../examples/DualStepper/DualStepper.ino

clean:
	rm *.o test_[0-9][0-9]
