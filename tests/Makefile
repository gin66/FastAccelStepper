
CFLAGS=-DTEST -Werror
CXXFLAGS=-DTEST -Werror
LDLIBS=-lm

test: test_01 test_02
	./test_01
	./test_02

test_01: test_01.cpp stubs.h FastAccelStepper.o
test_02: test_02.cpp stubs.h FastAccelStepper.o

FastAccelStepper.o: FastAccelStepper.cpp FastAccelStepper.h stubs.h

test_%.o: test_%.cpp stubs.h


fmt:
	clang-format --style=Google -i ../src/* *.cpp *.h

clean:
	rm *.o test_[0-9][0-9]
