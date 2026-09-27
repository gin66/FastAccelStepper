# judge_sync.awk — Verify synchronous start of stepper channels.
#
# Parses x.vcd (raw VCD from simavr) to find the timestamp of the first
# L→H transition for each step channel, then checks they started within
# max_delta µs of each other.
#
# Reads two files:
#   1) x.vcd — VCD output from simavr
#   2) expect.txt — single line: max_delta_us=<N> (or just <N>)
#
# VCD format (simplified):
#   $var wire 1 <id> <name> $end   — maps variable name to single-char id
#   #<time>                        — time marker (in timescale units)
#   <value><id>                    — data: value (0 or 1) followed by id
#
# Example VCD for test_sd_04_timing_328p:
#   $var wire 1 # StepA $end       → StepA id = '#'
#   $var wire 1 $ StepB $end       → StepB id = '$'
#   $timescale 10ns                → time unit = 10 nanoseconds
#   #7401400  → time = 7401400 * 10ns = 74.014 µs
#   1#        → StepA went high at that time

BEGIN {
	ok = 1
	step_id["StepA"] = 0
	step_id["StepB"] = 0
	step_id["StepC"] = 0
	current_time = 0
	time_scale = 0
	first_h_A = -1
	first_h_B = -1
	first_h_C = -1
	step_A_set = 0
	step_B_set = 0
	step_C_set = 0
	max_delta = 0
	has_delta = 0
	in_header = 0
	reading_dumpvars = 0
}

# Parse VCD header for variable names
/^\$var wire/ {
	# $var wire <size> <id_char> <variable_name> $end
	var_name = $5
	var_id = $4
	if (var_name == "StepA") step_id["StepA"] = var_id
	if (var_name == "StepB") step_id["StepB"] = var_id
	if (var_name == "StepC") step_id["StepC"] = var_id
}

# Get time resolution
/^\$timeScale/ {
	# $timescale 10ns or 1us etc.
	timescale_str = $2
	if (timescale_str ~ /ns/) {
		sub(/ns$/, "", timescale_str)
		time_scale = timescale_str + 0
	} else if (timescale_str ~ /us/) {
		sub(/us$/, "", timescale_str)
		time_scale = (timescale_str + 0) * 1000
	}
	# time_scale is now the VCD time-unit in nanoseconds
}

# Time marker
/^#/ && !in_header {
	# VCD time: #<number> — convert to µs
	vcd_time = substr($0, 2) + 0
	current_time = vcd_time / 1000.0  # ns → µs
	next
}

# Data lines: <value><id>
# Match lines like "1#" or "0$" (value + variable id character)
{
	# Get the value (first char) and id (second char)
	if (length($0) >= 2) {
		val = substr($0, 1, 1)
		id_char = substr($0, 2, 1)
		
		if (val == "1") {
			# High transition (L→H) — this is the start of a pulse
			if (id_char == step_id["StepA"] && !step_A_set) {
				first_h_A = current_time
				step_A_set = 1
			}
			if (id_char == step_id["StepB"] && !step_B_set) {
				first_h_B = current_time
				step_B_set = 1
			}
			if (id_char == step_id["StepC"] && !step_C_set) {
				first_h_C = current_time
				step_C_set = 1
			}
		}
	}
}

# Read expect.txt (second file argument)
FNR != NR {
	if (match($0, /max_delta_us=([0-9]+)/, arr)) {
		max_delta = arr[1] + 0
		has_delta = 1
	} else if (match($0, /^[0-9]+$/)) {
		max_delta = $0 + 0
		has_delta = 1
	}
}

END {
	if (!has_delta) max_delta = 10  # default 10 µs tolerance

	print "=== Synchronous Start Judgment ==="
	
	if (first_h_A < 0 || first_h_B < 0) {
		print "FAIL: Could not find first pulse for both StepA and StepB"
		print "  StepA found: " (step_A_set ? "yes" : "no")
		print "  StepB found: " (step_B_set ? "yes" : "no")
		if (step_A_set) print "  StepA first pulse: " sprintf("%.3f", first_h_A) " µs"
		if (step_B_set) print "  StepB first pulse: " sprintf("%.3f", first_h_B) " µs"
		ok = 0
	} else {
		delta = first_h_B - first_h_A
		if (delta < 0) delta = -delta
		print "StepA first pulse: " sprintf("%.3f", first_h_A) " µs"
		print "StepB first pulse: " sprintf("%.3f", first_h_B) " µs"
		print "Delta (|tB - tA|): " sprintf("%.3f", delta) " µs"
		print "Max allowed delta: " max_delta " µs"
		
		if (delta > max_delta) {
			print "FAIL: Delta " sprintf("%.3f", delta) " µs exceeds " max_delta " µs"
			ok = 0
		}
	}

	# If StepC was configured (2560 3-stepper), also verify
	if (step_id["StepC"] != 0) {
		if (first_h_C < 0) {
			print "FAIL: StepC did not pulse"
			ok = 0
		} else {
			print "StepC first pulse: " sprintf("%.3f", first_h_C) " µs"
			delta_AC = first_h_C - first_h_A
			if (delta_AC < 0) delta_AC = -delta_AC
			if (delta_AC > max_delta) {
				print "FAIL: StepC-StepA delta " sprintf("%.3f", delta_AC) " µs exceeds " max_delta " µs"
				ok = 0
			}
		}
	}

	if (ok == 1) {
		print "PASS: Steps started within " max_delta " µs of each other"
		print "PASS" > ".tested"
	} else {
		print "FAIL"
	}
}