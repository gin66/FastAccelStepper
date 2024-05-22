BEGIN {
	ok = 0
}

FNR == NR {
	# result.txt
	lines[FNR] = $0
}

/^Position\[A\]/ {
	print
}
$0 == "Position[A]=0" {
	ok = 1
}

END {
	if (ok == 1) {
		print("PASS")
		print("PASS") > ".tested"
	}
}
