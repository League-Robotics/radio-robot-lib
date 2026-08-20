# Protocol — the v6 ASCII line-grammar handler

Two files, no dependencies beyond the C++ standard library:

    adapter.h             Adapter (the seam a robot implements) + the
                           value types it exchanges with the handler
    protocol_handler.{h,cpp}  ProtocolHandler -- the codec itself

`Protocol::ProtocolHandler` is the only thing here that ever touches a
wire byte: `feed()` reassembles arbitrary byte blocks into `\n`-terminated
lines, splits each line in place on `:` (no allocation, no `std::string`,
no exceptions), dispatches to an `Adapter`, and formats the reply --
exactly once per verb, so the adapter can neither forget a reply nor
invent a shape for one. See `docs/protocol-v6-spec.md` for the wire
format and `docs/design/protocol.md` for the object model.

## Scope (docs/plan.md Step 3)

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
  `test_protocol_harness.py::test_golden_vectors`) -- the spec S11.3
  cross-language conformance fixture: literal wire examples from the
  spec text, asserted byte-for-byte through a mock adapter.
- **feed()'s byte-block contract, case-as-direction, arity, and
  ESTOP's no-ack rule** -- individually named tests in
  `test_protocol_harness.py`; see that file's own module docstring.

Read `protocol_handler.h`'s file header for the numbered list of wire
spec ambiguities this implementation had to resolve (the optional
trailing `id` on `SET`/`WHEELS`, `GET`'s undefined unknown-name
outcome, and `WHEELS`'s unenforced duration ceiling).

## Provenance

New code, sprint per `docs/plan.md` Step 3 (2026-08-20) -- unlike
`src/diffdrive/`, nothing here is extracted from an existing
implementation. `src/archive/protocol-v6/wire_v6_verbs.h` is reference
only (verb-name/arity cross-check); nothing in this package includes
or depends on it.
