#!/bin/sh

echo "build directories"

if [ "$GITHUB_WORKSPACE" != "" ]
then
	# Make sure we are inside the github workspace
	cd $GITHUB_WORKSPACE
fi

# Whatever this script is started from, cd to the top level
ROOT=`git rev-parse --show-toplevel`
cd $ROOT

pwd

# So create the pio_dirs-directory during the github action 
rm -fR pio_dirs
mkdir pio_dirs
for SRC in examples extras/issues
do
	for i in `ls $SRC`
	do
		(mkdir -p pio_dirs/$i/src
		cd pio_dirs/$i
	        mkdir FastAccelStepper
	        ln -s $ROOT/src FastAccelStepper
		ln -s ../../extras/ci/platformio.ini .
		cd src
		FILES=`cd ../../../$SRC/$i;find . -type f`
		for f in $FILES;do ln -s ../../../$SRC/$i/$f .;done
		)
	done
done

# ESP-IDF examples: link sources into the PlatformIO build directory.
# StepperDemo.ino cannot be symlinked as StepperDemo.cpp: PlatformIO's
# espidf.py compiles os.path.realpath(src), so the suffix becomes .ino
# and SCons rejects it. A one-line .cpp include keeps the suffix and
# still tracks edits to the sketch. Other files are plain symlinks.
rm -fR pio_espidf
mkdir pio_espidf
for i in `cd extras;ls idf_examples`
do
	(mkdir -p pio_espidf/$i/src
	cd pio_espidf/$i
	ln -s ../../extras/ci/platformio.ini .
	cd src
	FILES=`cd ../../../extras/idf_examples/$i;find . -type f`
	for f in $FILES;do ln -s ../../../extras/idf_examples/$i/$f .;done
	)
done
mkdir -p pio_espidf/StepperDemo/src
(
	cd pio_espidf/StepperDemo
	ln -s ../../extras/ci/platformio.ini .
	cd src
	FILES=`cd ../../../examples/StepperDemo;find . -type f`
	for f in $FILES
	do
		base=`basename "$f"`
		if [ "$base" = "StepperDemo.ino" ]
		then
			printf '%s\n' '#include "../../../examples/StepperDemo/StepperDemo.ino"' >StepperDemo.cpp
		else
			ln -s ../../../examples/StepperDemo/$f .
		fi
	done
)

for i in `cd pio_espidf;ls`
do
	(cd pio_espidf/$i
    mkdir FastAccelStepper
    ln -s $ROOT/src FastAccelStepper
    ln -s $ROOT/CMakeLists.txt FastAccelStepper
	)
done

# Make one directory to test Log2Representation on simulator
mkdir pio_dirs/LOG2_test
mkdir pio_dirs/LOG2_test/src
cd pio_dirs/LOG2_test
mkdir FastAccelStepper
ln -s $ROOT/src FastAccelStepper
ln -s ../../extras/ci/platformio.ini .
cd src
#sed  -e 's/%d/%ld/g' <../../../tests/test_03.h >test_03.h
ln -s ../../../extras/tests/pc_based/test_03.h .
ln -s ../../../extras/tests/pc_based/LOG2_test.ino LOG2_test.ino
cd ../../..

ls -al pio_*
find pio_*