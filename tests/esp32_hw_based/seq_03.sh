#!/bin/sh
DEV="-d /dev/ttyUSB0 -b 115200"

COMPLETE="Test completed"
PASS="Test passed"
MAX_RUN_S=300


for SEQ in 01 02 03 04 06 07 10 11
do
	LOG="$0_$SEQ.log"
	CMD="M1 p7,0,0 t M1 $SEQ R "

	grabserial $DEV -c 'reset ' -q "M1:" -e 10
	sleep 2

	grabserial $DEV -c "$CMD" -q "$COMPLETE" -e $MAX_RUN_S -o $LOG
	echo

	if [ `gawk -f judge_pcnt_sync.awk $LOG | grep -c PASS` -ne 1 ]
	then
		grabserial $DEV -c 'r ' -q StepperDemo -e 1
		echo
		echo FAIL $0 pulse counter mismatch
		exit 1
	fi

	if [ `grep -c "$PASS" $LOG` -eq 1 ]
	then
		echo PASS
	else
		grabserial $DEV -c 'r ' -q StepperDemo -e 1
		echo
		echo "FAIL $0 result pattern: $PASS"
		exit 1
	fi
done
