#!/bin/sh

if [ "$GITHUB_WORKSPACE" != "" ]
then
	# Make sure we are inside the github workspace
	cd $GITHUB_WORKSPACE
fi

# install platformio, if needed
which pio
if [ $? -ne 0 ]
then
	# Install PlatformIO CLI
	export PATH=$PATH:~/.platformio/penv/bin
	curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py -o get-platformio.py
	python3 get-platformio.py

	pio platform install "atmelavr"
	pio platform install "espressif32"
fi

rm -fR pio_dirs

mkdir pio_dirs
for i in `ls examples`
do
	mkdir -p pio_dirs/$i/src
	cd pio_dirs/$i
	ln -s ../../ci/platformio.ini .
	cd src
	ln -s ../../../src/*.cpp .
	ln -s ../../../src/*.h .
	ln -s ../../../examples/$i/*.ino .
	cd ../../..
done

set -e
for i in pio_dirs/*
do
	for p in avr esp32 esp32_debug
	do
		echo $p: $i
		(cd $i;pio run -s -e $p)
	done
done
