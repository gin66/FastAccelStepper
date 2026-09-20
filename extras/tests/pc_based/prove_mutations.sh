#!/bin/sh
# prove_mutations.sh — Step 2b mutation proof (todo.md "Done when").
#
# This is a *test*, next to test_26, not a library build script. It is not
# part of `make test` (two extra rebuilds of test_26). Run:
#
#   make -C extras/tests/pc_based mutations
#
# The theory probes in test_26's f2b_oracle() are only meaningful if a *wrong*
# model fails. Two compile-time hooks:
#
#   FAS_NAXIS_NO_REBIND    (remaining.h) -> binder_axis ignores ticks
#   FAS_NAXIS_NO_REST_CAP  (ramp_law.h)  -> ignore remaining-to-stop
#
# A correct model makes the counterexamples in f2b_oracle() hold, so the
# normal build passes. This script rebuilds test_26 with each hook and
# proves the corresponding probe now FAILS:
#
#    -D FAS_NAXIS_NO_REBIND    -> item 3 rebind neighbourhood fails
#    -D FAS_NAXIS_NO_REST_CAP  -> item 5 remaining-to-stop brake fails
#
# Each build links the already-compiled LIB_O objects (the same set as the
# normal test_26 rule), so only test_26 is recompiled.

set -u
cd "$(dirname "$0")"
DIR=.
PRJ=$(git rev-parse --show-toplevel)

# Ensure the LIB_O objects exist. The normal test_26 rule builds them and leaves
# them, removing only test_26.o; build them via that rule if any are missing.
if [ ! -f "$DIR/FastAccelStepper.o" ]; then
  make -C "$DIR" test_26 >/dev/null 2>&1
fi

# expect_fail: build with a hook, run, and require the program to fail (a
# test() assertion fires). Prints the tail of the run and a verdict. Returns 0
# when the mutation is proven, 1 when the wrong model somehow still passed, 2 on
# a build/link error.
expect_fail() {
  define="$1"
  label="$2"
  obj="$DIR/test_26_${define}.o"
  bin="$DIR/test_26_${define}"
  g++ -DTEST -Werror -g -DF_CPU=16000000 "-D$define" \
          -I"$PRJ/src" -c "$DIR/test_26.cpp" -o "$obj" 2>"$obj.log"
  if [ ! -f "$obj" ]; then
    echo "  ERROR: $bin failed to compile:"
    cat "$obj.log"
    rm -f "$obj" "$obj.log"
    return 2
  fi
  g++ -o "$bin" "$obj" \
          "$DIR"/FastAccelStepper.o "$DIR"/FastAccelStepperEngine.o \
          "$DIR"/Log2Representation.o "$DIR"/StepperISR_test.o \
          "$DIR"/RampGenerator.o "$DIR"/RampControl.o \
          "$DIR"/RampCalculator.o "$DIR"/queue_add_entry.o \
          "$DIR"/queue_get_position.o "$DIR"/queue_init.o \
          "$DIR"/queue_utils.o -lm -lc 2>"$bin.log"
  rm -f "$obj" "$obj.log"
  if [ ! -x "$bin" ]; then
    echo "  ERROR: $bin failed to link:"
    cat "$bin.log"
    rm -f "$bin" "$bin.log"
    return 2
  fi
  out="$("$bin" 2>/dev/null)"
  code=$?
  rm -f "$bin" "$bin.log"
  printf '%s\n' "$out" | tail -3
  if [ "$code" -eq 0 ]; then
    echo "  UNEXPECTED: test_26 still PASSES with $label disabled (probe not load-bearing)"
    return 1
  fi
  echo "  OK: test_26 FAILS with $label disabled ($label detected the wrong model)"
  return 0
}

rc=0
echo "== FAS_NAXIS_NO_REBIND: item 3 rebind neighbourhood must FAIL =="
expect_fail FAS_NAXIS_NO_REBIND "rebind"; r=$?; [ "$r" -eq 0 ] || rc=1
echo
echo "== FAS_NAXIS_NO_REST_CAP: item 5 remaining-to-stop brake must FAIL =="
expect_fail FAS_NAXIS_NO_REST_CAP "rest cap"; r=$?; [ "$r" -eq 0 ] || rc=1
echo

[ "$rc" -eq 0 ] && echo "ALL MUTATIONS PROVEN" || echo "MUTATION PROOF FAILED"
exit $rc
