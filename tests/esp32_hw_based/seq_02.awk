BEGIN {
	pass = 1
}

/^M1:/ {
	print
	api = substr($2,2)
	pcnt = substr($3,2,length($3)-2)
	if ((api-pcnt > 1) || (pcnt-api > 1)) {
		pass = 0
		print "FAIL HERE ^^^"
	}
}
/^>> M1:/ {
	print
	api = substr($3,2)
	pcnt = substr($4,2,length($4)-2)
	if (api != pcnt) {
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
