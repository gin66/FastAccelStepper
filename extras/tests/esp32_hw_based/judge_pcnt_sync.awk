BEGIN {
	pass = 1
}

# This is for running motor
/^M[17]:/ {
	api = substr($2,2)
	pcnt = substr($3,2,length($3)-2)
	if (pcnt < 0) {
		if (api > 0) {
			while (pcnt < 0) {
				pcnt += 32767
			}
		}
	}
	api = api % 32767
	delta = pcnt - api
	if (api > pcnt) {
		delta = api - pcnt
	}
	if ((delta > 66) && (delta < 32767-66)) {
	    print
		print api, pcnt
		pass = 0
		print "FAIL HERE ^^^"
	}
}
# This is for selected motor
/^>> M[17]:/ {
	print
	api = substr($3,2)
	api = api % 32767
	pcnt = substr($4,2,length($4)-2)
	if (pcnt < 0) {
	    pcnt += 32767
	}
	if (api != pcnt) {
		print api, pcnt
		pass = 0
		print "FAIL HERE ^^^"
	}
}

END {
	if (pass) {
		print "PASS"
	}
	else {
		print "FAIL"
	}
}
