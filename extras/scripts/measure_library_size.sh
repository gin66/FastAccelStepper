#!/bin/bash
#
# Measure FastAccelStepper library RAM/ROM footprint across architectures
# Uses the difference method: (UsageExample - Baseline) = library size
#

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel)"
OUTPUT_FILE="$ROOT_DIR/extras/doc/memory_report.md"

# Architectures to test (env names from extras/ci/platformio.ini)
# Format: "env_name:Board_Name:Framework"
ARCHITECTURES=(
	"nanoatmega328:ATmega328P:Arduino"
	"atmega2560:ATmega2560:Arduino"
	"atmega32u4:ATmega32U4:Arduino"
	"esp32_V6_13_0:ESP32:Arduino"
	"esp32s2_V6_13_0:ESP32-S2:Arduino"
	"esp32s3_V6_13_0:ESP32-S3:Arduino"
	"esp32c3_V6_13_0:ESP32-C3:Arduino"
	"rpipico:RP2040:Arduino"
	"rpipico2:RP2350:Arduino"
	"atmelsam:SAM3X8E:Arduino"
)

cd "$ROOT_DIR"

setup_baseline() {
	if [ ! -d "pio_dirs/Baseline" ]; then
		echo "Creating Baseline project..."
		mkdir -p pio_dirs/Baseline/src
		cd pio_dirs/Baseline
		mkdir -p FastAccelStepper
		ln -sf "$ROOT_DIR/src" FastAccelStepper
		ln -sf ../../extras/ci/platformio.ini .
		cat >src/Baseline.ino <<'EOF'
// Empty baseline sketch for memory measurement
void setup() {}
void loop() {}
EOF
		cd "$ROOT_DIR"
	fi
}

parse_memory() {
	local output="$1"
	local ram_line flash_line ram flash

	ram_line=$(echo "$output" | grep -E "^\s*(RAM|Data):" | head -1)
	flash_line=$(echo "$output" | grep -E "^\s*(Flash|Program):" | head -1)

	if [ -n "$ram_line" ]; then
		ram=$(echo "$ram_line" | grep -oE '(used )?[0-9]+ bytes' | grep -oE '[0-9]+' | head -1)
	else
		ram=0
	fi

	if [ -n "$flash_line" ]; then
		flash=$(echo "$flash_line" | grep -oE '(used )?[0-9]+ bytes' | grep -oE '[0-9]+' | head -1)
	else
		flash=0
	fi

	echo "$ram $flash"
}

measure_project() {
	local project="$1"
	local env="$2"
	local output

	cd "$ROOT_DIR/pio_dirs/$project"

	output=$(pio run -e "$env" 2>&1) || {
		echo "FAILED to build $project for $env" >&2
		echo "$output" | tail -20 >&2
		cd "$ROOT_DIR"
		return 1
	}

	cd "$ROOT_DIR"
	parse_memory "$output"
}

measure_architecture() {
	local entry="$1"
	local env board
	local baseline_result test_result
	local baseline_ram baseline_flash test_ram test_flash lib_ram lib_flash

	env=$(echo "$entry" | cut -d: -f1)
	board=$(echo "$entry" | cut -d: -f2)

	echo -n "  Measuring $env ($board)..."

	baseline_result=$(measure_project "Baseline" "$env") || return 1
	baseline_ram=$(echo "$baseline_result" | cut -d' ' -f1)
	baseline_flash=$(echo "$baseline_result" | cut -d' ' -f2)

	test_result=$(measure_project "UsageExample" "$env") || return 1
	test_ram=$(echo "$test_result" | cut -d' ' -f1)
	test_flash=$(echo "$test_result" | cut -d' ' -f2)

	lib_ram=$((test_ram - baseline_ram))
	lib_flash=$((test_flash - baseline_flash))

	[ "$lib_ram" -lt 0 ] && lib_ram=0
	[ "$lib_flash" -lt 0 ] && lib_flash=0

	echo " RAM: ${lib_ram}B, Flash: ${lib_flash}B"

	echo "| $board | $lib_ram | $lib_flash |" >>"$TMP_RESULTS"
}

generate_report() {
	cat >"$OUTPUT_FILE" <<'HEADER'
# FastAccelStepper Memory Footprint

HEADER
	echo "Generated: $(date '+%Y-%m-%d %H:%M:%S')" >>"$OUTPUT_FILE"
	echo "" >>"$OUTPUT_FILE"
	cat >>"$OUTPUT_FILE" <<'HEADER2'
Library size measured using the difference method:
- Baseline: Empty Arduino sketch (setup/loop only)
- Test: UsageExample with basic stepper initialization

| Board | RAM (bytes) | Flash (bytes) |
|-------|-------------|---------------|
HEADER2

	sort "$TMP_RESULTS" >>"$OUTPUT_FILE"

	cat >>"$OUTPUT_FILE" <<'FOOTER'

**Notes:**
- RAM = .data + .bss (static allocation)
- Flash = .text + .data (code + initialized data)
- Actual usage may vary based on enabled features
- All builds use release optimization (-Os)
- Values include peripheral drivers pulled in by the library:
  - ESP32: RMT, MCPWM, PCNT drivers (~15-20KB of the flash)
  - SAM: Timer/Counter drivers
- For detailed per-symbol analysis, use PlatformIO's "Project Inspect" feature
FOOTER

	echo ""
	echo "Report saved to: $OUTPUT_FILE"
}

main() {
	echo "FastAccelStepper Memory Footprint Measurement"
	echo "============================================="
	echo ""

	setup_baseline

	TMP_RESULTS=$(mktemp)
	trap "rm -f $TMP_RESULTS" EXIT

	local failed=0
	local succeeded=0

	echo "Measuring architectures..."

	for entry in "${ARCHITECTURES[@]}"; do
		if measure_architecture "$entry"; then
			succeeded=$((succeeded + 1))
		else
			failed=$((failed + 1))
		fi
	done

	echo ""
	echo "Build summary: $succeeded succeeded, $failed failed"

	if [ "$succeeded" -gt 0 ]; then
		generate_report
	else
		echo "No successful builds, skipping report generation"
		exit 1
	fi
}

main "$@"
