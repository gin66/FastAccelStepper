#!/bin/sh

TARGETS=${1:-avr atmega2560 esp32 atmelsam}
echo "execute for ${TARGETS}"

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

	# Use automated install from pio run
	# pio platform install "atmelavr"
	# pio platform install "atmelsam"
	# pio platform install "espressif32"
fi

# arduino libraries are not allowed to contain symbolic links in the repository.
# So create the pio_dirs-directory during the github action 
rm -fR pio_dirs
mkdir pio_dirs
for i in `ls examples`
do
	mkdir -p pio_dirs/$i/src
	cd pio_dirs/$i
	ln -s ../../ci/platformio.ini .
	cd src
	FILES=`cd ../../../examples/$i;find . -type f`
	for f in $FILES;do ln -s ../../../examples/$i/$f .;done
	cd ../../..
done

# Make one directory to test PoorManFloat no device
mkdir pio_dirs/PMF_test
mkdir pio_dirs/PMF_test/src
cd pio_dirs/PMF_test
ln -s ../../ci/platformio.ini .
cd src
#sed  -e 's/%d/%ld/g' <../../../tests/test_03.h >test_03.h
ln -s ../../../tests/pc_based/test_03.h .
ln -s ../../../tests/pc_based/PMF_test.ino PMF_test.ino
cd ../../..

set -e
for i in pio_dirs/*
do
	for p in ${TARGETS}
	do
		echo $p: $i
		(cd $i;pio run -s -e $p)
	done
done
