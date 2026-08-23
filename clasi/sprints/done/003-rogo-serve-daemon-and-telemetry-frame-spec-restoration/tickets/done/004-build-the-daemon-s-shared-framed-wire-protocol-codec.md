---
id: '004'
title: Build the daemon's shared framed wire-protocol codec
status: done
use-cases:
- SUC-001
- SUC-003
depends-on: []
github-issue: ''
issue: rebuild-rogo-serve-daemon-on-v6-named-sockets-pipe-mode-sim.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Build the daemon's shared framed wire-protocol codec

## Description

Foundation ticket for the daemon subsystem (sprint.md's Architecture,
Step 3 `rogo.daemon_protocol` entry). Build the framed request/reply
wire-protocol codec shared by `daemon.py` (ticket 005) and
`daemon_client.py` (ticket 008): one request line in, one JSON reply
line out, with echoed correlation ids, per the issue's own Requirement
1 ("exposes the command grammar to clients over a framed wire protocol
... with echoed correlation ids, per the elite daemon's protocol
shape"). This is a pure codec, no socket/pipe I/O of its own — mirrors
`robot_v6.codec`'s own separation from `robot_v6.transport` in this same
codebase, so the daemon's two ends can never independently drift on the
wire shape.

## Acceptance Criteria

- [x] Encoding a request produces one self-delimited line (newline- or
      length-framed — pick one, document it in the module docstring)
      carrying a correlation id.
- [x] Decoding a reply recovers the same correlation id the matching
      request carried, unambiguously pairing requests with replies.
- [x] Round-trip (encode then decode) is lossless for the request/reply
      shapes this ticket defines.
- [x] The codec has no dependency on `socket`/`subprocess`/any transport
      module — it operates on bytes/strings in and out only, so it can
      be unit-tested with no process or socket involved.

## Implementation Plan

**Approach**: Define the request/reply message shapes (request: verb +
arguments + correlation id; reply: correlation id + structured
result/error) and their line-framed JSON encoding, in a new module with
encode/decode functions on each side — no I/O.

**Files to create**:
- `src/host/rogo/daemon_protocol.py` — the codec.

**Testing plan**: New tests in `tests/host/rogo/` covering encode/decode
round-trips for at least: an ordinary request/reply pair, an
error-reply shape, and a malformed-input decode (confirms the codec
fails closed rather than raising an unhandled exception a caller can't
catch cleanly). Scoped run: `uv run python -m pytest -q
tests/host/rogo/ -k daemon_protocol`.

**Documentation updates**: Module docstring documents the wire shape
(this is this module's only "spec" — no `docs/design/` change, since
the daemon's own client protocol is host-tooling internal, not part of
the protocol-v6 wire this project's `docs/design/protocol.md` governs).
