
CFLAGS=-DTEST
CXXFLAGS=-DTEST
LDLIBS=-lm

test: tests/test_01
	tests/test_01

tests/test_01: tests/test_01.o tests/FastAccelStepper.o

tests/FastAccelStepper.o: tests/FastAccelStepper.cpp tests/FastAccelStepper.h tests/stubs.h

tests/test_01.o: tests/test_01.cpp tests/stubs.h

clean:
	rm tests/*.o tests/test_[0-9][0-9]
