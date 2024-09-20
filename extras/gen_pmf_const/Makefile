CPPFLAGS=-I../../src

../../src/PoorManFloatConst.h: main
	./main >../../src/PoorManFloatConst.h

main: main.o PoorManFloat.o

main.o:	main.cpp

PoorManFloat.o: PoorManFloat.cpp

PoorManFloat.cpp: ../../src/PoorManFloat.cpp
	cp ../../src/PoorManFloat.cpp PoorManFloat.cpp

clean:
	rm -f PoorManFloat.cpp
	rm -f main main.o PoorManFloat.o
