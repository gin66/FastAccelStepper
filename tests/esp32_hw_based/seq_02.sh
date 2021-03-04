#!/bin/sh
DEV="-d /dev/ttyUSB0 -b 115200"

CMD="M1 p7,0,0 H30000 A100000 R30000 "
PASS=">> M1: @30000 \\[30000\\]"

LOG="$0.log"

grabserial $DEV -c 'r ' -e 1

grabserial $DEV -c "$CMD" -q "$PASS" -e 3 -o $LOG
echo

if [ `gawk -f seq_02.awk $LOG | grep -c PASS` -eq 1 ]
then
	echo PASS
else
	grabserial $DEV -c 'r ' -q StepperDemo -e 1
	echo
	echo FAIL
	exit 1
fi

