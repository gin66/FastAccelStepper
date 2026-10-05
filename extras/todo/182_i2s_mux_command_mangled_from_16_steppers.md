# 182 `i2s_mux` mangles any command from n >= 16

## Priority

**HIGH** — this is the reason the mux's headline capability cannot be measured
at all. `nodir` is documented to reach 32 multiplexed steppers and `dir` 16,
because a stepper costs a bit of a 32-bit word and no analyzer channel. Every
one of those points is refused by the host's own command parser, so the number
the mux exists to demonstrate is untestable today.

## Finding

```
CONFIG 16 i2s_mux,i2s_mux      ->  ERR unknown '2s_mux,i2s_mux'
CONFIG 32 i2s_mux,...          ->  ERR unknown '2s_mdir'
```

The error is `unknown '<token>'`, so the failure is inside the CONFIG
**argument** parser, not the driver: the driver list is being read back with its
first character already gone, and the tokens are being split somewhere other
than the intended comma. The two examples are consistent with a **fixed-size
receive buffer overflowing** — `CONFIG 32 …` is long enough to lose more than
one character per token, and what survives is the tail of the line, not the
head.

Smallest failing input is n = 16, which is exactly where `nodir` is documented
to stop and `dir` to stop too, so the boundary is the line length, not the stepper
count: `CONFIG 16 …` is ~144 characters.

## What has been ruled out

- **Not `uint8_t linelen`.** The obvious suspect is a line counter wrapping at
  255, and a comment in `common/saleae_app.cpp` describes it that way. It is
  `uint16_t`, and n = 16 is only 144 characters — nowhere near a wrap. The
  documented mechanism is wrong even though the symptom is real.
- **Not pre-existing breakage in this branch's recent work.** Verified against a
  pristine checkout of the branch, which fails identically.
- **Not specific to `i2s_mux` as a driver name.** n <= 8 parses fine, and
  `CONFIG 2 i2s_mux,i2s_mux dir` answers with correct slots, so the token
  vocabulary is fine; only long lines fail.

## What to find

- **Find the actual bound, and whether it is a buffer or a `strtok` misuse.**
  The harness already has a small tokenizer (`sal_tokenize()` in
  `common/saleae_str.h`, added because the field widths are runtime values), and
  the CLAUDE-visible constraint is that nothing off AVR may put a large
  `char[]` on the stack — so whatever holds the line must be sized from
  `SALEAE_ARG2_MAX`, not from a literal. `TestStackBudget` and `TestSaleaeFmt`
  in `scripts/tests/test_saleae.py` are the places a fix should show up.
- **Confirm the 32-stepper path end to end once parsing is fixed.** Note that
  the other known mux defect — 022, `i2s_mux` in `dir` losing its second slot —
  blocks the same territory, so the two are worth fixing together and measuring
  with the same runs. 022 also records the intermittent dropped step, which is
  a different thing and stays separate.
- **Then the claim in the docs can finally be tested.** `nodir` reaching 32 and
  `dir` reaching 16 are assertions about the wire protocol (a 32-bit word, a
  stepper costing one bit and a direction costing the next). Neither has been
  measured above n = 8.

## Note

Found while closing [015/016](../doc/implemented/idf55_main_task_stack_overflow.md),
which is where it was first written down. It belongs to the mux backlog, not to
that item, and the stack overflow neither caused nor fixed it.
## 2026-10-05 — confirmed still unmeasured; the matrix cannot reach it

Worth recording explicitly, because the full six-row release matrix ran after
this item was filed and did **not** close it: the matrix sweeps the mux only to
`n = 8`. Every mux point it takes is `n = 1…8` in `nodir` plus the single
`n = 2` `dir` combination, so nothing above 8 is exercised anywhere.

That is the whole reason this item exists. The mux's design claim is that `nodir`
reaches **32** multiplexed steppers and `dir` reaches **16**, measured on an
8-channel analyzer because a multiplexed stepper costs a bit of the word and no
analyzer channel. The claim is currently supported by *zero* measurements above
n = 8, on a firmware whose host-side command parser drops the first character of
each driver token from n ≥ 16.

The cheapest thing that would confirm or kill the claim is one serial exchange,
no analyzer required:

```
QCLR
IMUX
CONFIG 16 i2s_mux x16      -> expect OK CONFIG n=16
```

The failure is already at the parser, before any hardware is armed, so it needs
no capture and no stepper — only the board. Until that one line answers `OK`,
"32 steppers on 3 wires" is a design intent and not a measurement.
