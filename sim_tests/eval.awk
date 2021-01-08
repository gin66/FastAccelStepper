BEGIN {
	low = 0
	high = 0
	last_transition_l_h = 0
	transition_h_l = 0
	transition_l_h = 0
	ref = 16*100
}

/^#/ { time = substr($1,2) + 0 }
/^1\$$/ { high++; transition_l_h = time; transition_h_l = 0 }
/^0\$$/ { low++; transition_h_l = time }

(transition_h_l != 0) && (transition_l_h != 0) {
	x = ""
	if (last_transition_l_h != 0) {
		x = sprintf("period = %d us, ", (transition_l_h-last_transition_l_h)/ref)
	}
	x = sprintf("%shigh_time = %d us", x, (transition_h_l-transition_l_h)/ref)
	print(high,x)
	last_transition_l_h = transition_l_h
	transition_h_l = 0
	transition_l_h = 0
}	

END {
	print "L->H=",high,"H->L=",low >"result.txt"
}t
