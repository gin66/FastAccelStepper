CPPFLAGS=-I../../src

../../src/Log2RepresentationConst.h: main
	./main >../../src/Log2RepresentationConst.h

main: main.o Log2Representation.o

main.o:	main.cpp

Log2Representation.o: Log2Representation.cpp

Log2Representation.cpp: ../../src/Log2Representation.cpp
	cp ../../src/Log2Representation.cpp Log2Representation.cpp

clean:
	rm -f Log2Representation.cpp
	rm -f main main.o Log2Representation.o
