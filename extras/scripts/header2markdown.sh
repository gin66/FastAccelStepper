#!/bin/sh

# This extract all lines with C-comments // as plain text
# Places all C-lines in quotes

gawk '
/\/\// && (in_code == 1) { print }

/\/\// {
	gsub("[ \t]*// *","")
	print
	next
}

in_code == 0 {
	in_code = 1
	print("```cpp")
}

(in_code == 1) && /^$/ { in_code = 0 }

in_code == 1 { print }
' ../../src/FastAccelStepper.h >../doc/FastAccelStepper_API.md
