BEGIN {
	ok = 1
	timing = 0
	if (DIR ~ /timing/) {
		timing = 1
	}
}

# Compare one line using range syntax: ~N at end of expect line means
# the result value must be within ±N of the expected value.
#
# Example: "Position[A]=50536~300" accepts any value in [50236, 50836].
#
# Returns 1 if the line matches within the specified range, 0 otherwise.
function compare_range(r, e,
		tpos, tol, e_base, e_val, e_prefix, r_sub, r_val, diff) {
	tpos = index(e, "~")
	tol = substr(e, tpos + 1) + 0
	e_base = substr(e, 1, tpos - 1)

	# Find the last (possibly negative) number in e_base
	match(e_base, /[-]?[0-9]+$/)
	if (RSTART == 0) return 0
	e_val = substr(e_base, RSTART, RLENGTH) + 0
	e_prefix = substr(e_base, 1, RSTART - 1)

	# Prefix must match exactly
	if (substr(r, 1, length(e_prefix)) != e_prefix) return 0

	# Extract the corresponding number from result
	r_sub = substr(r, length(e_prefix) + 1)
	match(r_sub, /^[-]?[0-9]+/)
	if (RSTART == 0) return 0
	r_val = substr(r_sub, 1, RLENGTH) + 0

	diff = r_val - e_val
	if (diff < 0) diff = -diff
	return diff <= tol
}

# Compare one line from result.txt (r) against one line from expect.txt (e),
# supporting the "<=" upper-bound operator in timing values.
#
# In expect.txt a value can be written as:
#   =Xus   or  = X us   -- exact match (default)
#   <=Xus  or  <=X us   -- result must not exceed X
#
# The structural parts of the line (everything outside the us values) must
# match exactly between result and expect.
function compare_us_line(r, e,
		r_text, e_text, r_copy, e_copy,
		e_part, r_part, e_op, e_val, r_val, ok_line) {
	ok_line = 1

	# Verify structural equality (non-value parts)
	r_text = r; gsub(/=[0-9]+ *us/, "US", r_text)
	e_text = e; gsub(/[<>]*=[0-9]+ *us/, "US", e_text)
	if (r_text != e_text) return 0

	# Compare us values pairwise, applying the operator from expect
	r_copy = r
	e_copy = e
	while (match(e_copy, /[<>]*=[0-9]+ *us/)) {
		e_part = substr(e_copy, RSTART, RLENGTH)
		e_copy = substr(e_copy, RSTART + RLENGTH)

		if (e_part ~ /^<=/) {
			e_op = "<="
		} else {
			e_op = "="
		}
		match(e_part, /[0-9]+/)
		e_val = substr(e_part, RSTART, RLENGTH) + 0

		if (!match(r_copy, /=[0-9]+ *us/)) return 0
		r_part = substr(r_copy, RSTART, RLENGTH)
		r_copy = substr(r_copy, RSTART + RLENGTH)
		match(r_part, /[0-9]+/)
		r_val = substr(r_part, RSTART, RLENGTH) + 0

		if (e_op == "<=") {
			if (r_val > e_val) ok_line = 0
		} else {
			if (r_val != e_val) ok_line = 0
		}
	}
	return ok_line
}

FNR == NR {
	# result.txt
	lines[FNR] = $0
}

FNR != NR {
	# expect.txt
	r = lines[FNR]
	e = $0

	r_test = r
	e_test = e

	if (e ~ /~[0-9]+$/) {
		# Range comparison: ~N at end of expect line
		if (!compare_range(r, e)) {
			print("result:",r)
			print("expect:",e)
			ok = 0
		}
	} else if (timing == 0) {
		gsub(/[0-9 ]*us/,"", r_test)
		gsub(/[0-9 ]*us/,"", e_test)
		if (e_test != r_test) {
			print("result:",r)
			print("expect:",e)
			ok = 0
		}
	} else {
		# timing test: lines with us values support the <= operator
		if (e ~ /us/) {
			if (!compare_us_line(r, e)) {
				print("result:",r)
				print("expect:",e)
				ok = 0
			}
		} else {
			if (r != e) {
				print("result:",r)
				print("expect:",e)
				ok = 0
			}
		}
	}
}

END {
	if (ok == 1) {
		print("PASS")
		print("PASS") > ".tested"
	}
}
