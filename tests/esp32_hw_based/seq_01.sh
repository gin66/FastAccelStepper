#!/bin/sh

DEV="-d /dev/ttyUSB0 -b 115200"

CMD="M1 p7,0,0 H25000 A10000 f w1000 X pc R1000 "
PASS=">> M1: @1000 \\[1000\\]"

LOG="$0.log"

grabserial $DEV -c 'reset ' -q "M1:" -e 10
sleep 2

grabserial $DEV -c "$CMD" -q "$PASS" -e 3 -o $LOG
echo

#if [ `gawk -f judge_pcnt_sync.awk $LOG | grep -c PASS` -ne 1 ]
#then
#	grabserial $DEV -c 'r ' -q StepperDemo -e 1
#	echo
#	echo FAIL $0 pulse counter mismatch
#	exit 1
#fi
if [ `grep -c "$PASS" $LOG` -eq 1 ]
then
	echo PASS
else
	grabserial $DEV -c 'r ' -q StepperDemo -e 1
	echo
	echo "FAIL $0 result pattern: $PASS"
	exit 1
fi

