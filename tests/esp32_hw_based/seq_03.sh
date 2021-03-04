#!/bin/sh
DEV="-d /dev/ttyUSB0 -b 115200"

CMD="t M1 07 R "
PASS="Test passed"
MAX_RUN_S=10

LOG="$0.log"

grabserial $DEV -c 'r ' -e 1

grabserial $DEV -c "$CMD" -q "$PASS" -e $MAX_RUN_S -o $LOG
echo

#if [ `gawk -f seq_02.awk $LOG | grep -c PASS` -eq 1 ]
if [ `grep -c "$PASS" $LOG` -eq 1 ]
then
	echo PASS
else
	grabserial $DEV -c 'r ' -q StepperDemo -e 1
	echo
	echo FAIL
	exit 1
fi

