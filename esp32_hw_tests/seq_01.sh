#!/bin/sh

DEV="-d /dev/ttyUSB0 -b 115200"

CMD="M1 H25000 A10000 f w1000 X R1000 "
PASS=">> M1: @1000"

grabserial $DEV -c 'r ' -e 1

grabserial $DEV -c "$CMD" -q "$PASS" -e 3 -o seq_01.log
echo

if [ `grep -c "$PASS" seq_01.log` -eq 1 ]
then
	echo PASS
else
	grabserial $DEV -c 'r ' -q StepperDemo -e 1
	echo
	echo FAIL
	exit 1
fi

