
CFLAGS=-DTEST -Werror
CXXFLAGS=-DTEST -Werror
LDLIBS=-lm

test: tests/test_01 tests/test_02
	tests/test_01
	tests/test_02

tests/test_01: tests/test_01.cpp tests/stubs.h tests/FastAccelStepper.o
tests/test_02: tests/test_02.cpp tests/stubs.h tests/FastAccelStepper.o

tests/FastAccelStepper.o: tests/FastAccelStepper.cpp tests/FastAccelStepper.h tests/stubs.h

tests/test_%.o: tests/test_%.cpp tests/stubs.h


fmt:
	clang-format --style=Google -i src/* tests/*.cpp tests/*.h

clean:
	rm tests/*.o tests/test_[0-9][0-9]
