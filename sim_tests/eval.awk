BEGIN {
	low = 0
	high = 0
}

/^0\$$/ { low++ }
/^1\$$/ { high++ }

END {
	print "L->H=",high,"H->L=",low >"result.txt"
}t
