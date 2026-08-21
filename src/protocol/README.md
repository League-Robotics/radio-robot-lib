# Protocol — the v6 ASCII line-grammar handler

Two files, no dependencies beyond the C++ standard library:

    adapter.h             Adapter (the seam a robot implements) + the
                           value types it exchanges with the handler
    protocol_handler.{h,cpp}  ProtocolHandler -- the codec itself

`Protocol::ProtocolHandler` is the only thing here that ever touches a
wire byte: `feed()` reassembles arbitrary byte blocks into `\n`-terminated
lines, tokenizes each line in place on runs of `' '` (no allocation, no
`std::string`, no exceptions), dispatches to an `Adapter`, and formats
the reply -- exactly once per verb, so the adapter can neither forget a
reply nor invent a shape for one. See `docs/design/protocol.md` for the
wire format and the object model -- both live in that one document now.

**Grammar note (2026-08-20):** this package was rewritten wholesale from
an earlier colon-delimited, positional-id grammar to the space/`#id`
grammar (`line ::= sp? verb (sp field)* sp? '\n'`, commit 5a5b6da) -- a
run of spaces is one separator, a blank/all-whitespace line is ignored
silently, and the correlation id is a trailing, self-marking `#<n>`
field rather than a positional one. See `protocol_handler.h`'s own file
header for the full resolution history of what changed and why.

## Scope

In: `HELLO PING ID VER STATUS HELP GET SET TLM WHEELS STOP ESTOP`, and
their replies. No kernel, no motors, no config storage, no transport --
bytes in via `feed()`, bytes out via `Sink`. `MOVE`/`GOTO`/`SEED`/`CAL`
are deliberately out of scope; they need a planner, navigator, or
odometry this library does not own.

## Testing

`tests/protocol/` holds the package gate:

- **standalone build** (`test_protocol_package.py`) -- the package
  compiles with an include path of exactly its own directory.
- **golden vectors** (`golden_vectors.txt`, driven by
  `test_protocol_harness.py::test_golden_vectors`) -- the cross-language
  conformance fixture: literal wire examples plus every `Result`/error-code
  combination the handler can emit, asserted byte-for-byte through a mock
  adapter. Also covers the
  new grammar's own rules: space-run collapsing, the bare vs id-carrying
  `ok`/`err` reply shapes, and the malformed-line `#id` recovery rule.
- **feed()'s byte-block contract, case-as-direction, arity, blank/
  all-whitespace-line silence, the id's three wire behaviors (omitted /
  `#0` / nonzero), the malformed-line `#id` recovery rule (including
  unknown verbs), and ESTOP's no-ack-ever rule** -- individually named
  tests in `test_protocol_harness.py`; see that file's own module
  docstring.
- **chunk-split equivalence**
  (`test_protocol_harness.py::test_feed_chunk_split_equivalence_golden_vectors`)
  -- every feed()-driven golden-vector block, fed one-shot,
  byte-at-a-time, and via several fixed-seed random chunkings, must
  produce byte-identical output. This is the invariant a future
  MicroPython/JavaScript port is most likely to get wrong.
- **adversarial input + the recovery invariant**
  (`test_protocol_adversarial.py`) -- hostile bytes (embedded NUL,
  high-ASCII/UTF-8, control characters, `#`/space floods, line-length
  boundaries, unterminated fragments, non-space whitespace bytes as a
  field's leading byte) run through the REAL handler compiled with
  AddressSanitizer + UndefinedBehaviorSanitizer, each followed by a
  check that a subsequent well-formed line still dispatches correctly.
  Also holds the regression tests for the three real parser bugs the
  original (colon-era) hardening sweep found and fixed (hex-float
  values, leading-whitespace numeric fields, a NaN reaching
  `formatConfigValue()`) and a characterization test for one
  deliberately-not-fixed C-string quirk
  (`test_embedded_nul_immediately_after_verb_matches_bare_verb`) -- see
  that file's own module docstring, and `docs/design/protocol.md` §9.4,
  for the full story.

Read `protocol_handler.h`'s file header for the numbered list of wire
spec ambiguities/design calls this implementation makes (`WHEELS`'s
unenforced duration ceiling, the malformed-line `#id` recovery rule's
interaction with ESTOP's own stronger no-ack guarantee, and the id's
own stricter no-sign numeric grammar).

## Provenance

New code (2026-08-20) -- unlike `src/diffdrive/`, nothing here is
extracted from an existing implementation. `src/archive/protocol-v6/
wire_v6_verbs.h` is reference only (verb-name/arity cross-check);
nothing in this package includes or depends on it. Migrated from the
colon grammar to the space/`#id` grammar the same day, per
`docs/design/protocol.md` §9.6 (commit 5a5b6da).
