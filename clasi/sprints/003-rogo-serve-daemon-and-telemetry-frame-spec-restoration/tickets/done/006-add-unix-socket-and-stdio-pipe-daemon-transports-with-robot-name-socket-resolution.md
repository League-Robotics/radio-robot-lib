---
id: '006'
title: Add Unix-socket and stdio-pipe daemon transports with robot-name socket resolution
status: done
use-cases:
- SUC-001
- SUC-003
depends-on:
- '005'
github-issue: ''
issue: rebuild-rogo-serve-daemon-on-v6-named-sockets-pipe-mode-sim.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add Unix-socket and stdio-pipe daemon transports with robot-name socket resolution

## Description

Attach the two listener transports the issue requires (Requirement 2)
to ticket 005's server core: a named Unix domain socket for production,
and a stdio-pipe mode (daemon forked as a child, speaking the same
framed protocol over stdin/stdout) for tests/embedding. Both transports
carry the identical wire protocol from ticket 004 — only the I/O
mechanism differs.

**Socket directory (sprint.md's Architecture Design Rationale)**: use
`$XDG_RUNTIME_DIR/rogo/` when `XDG_RUNTIME_DIR` is set, else
`~/.rogo/run/` — created with owner-only permissions (0700). Socket
filename is the resolved robot name (e.g. `tovez.sock`), so multiple
robots on one host run multiple discoverable daemons (issue
Requirement 2). Robot-name resolution: from the robot's own
hello/identify response where possible, overridable by flag; for a
`--sim` target with no flag override, fall back to a fixed default name
(e.g. `sim`) — sprint.md's Architecture Step 7 flags the exact
`--sim` default as a ticket-level decision, made here.

## Acceptance Criteria

- [x] `rogo serve` listens on a Unix socket at
      `$XDG_RUNTIME_DIR/rogo/<name>.sock` when `XDG_RUNTIME_DIR` is
      set, else `~/.rogo/run/<name>.sock`; the containing directory is
      created with 0700 permissions if it does not exist.
- [x] Two daemons started against two differently-named robots produce
      two distinct socket paths and can run concurrently without
      colliding.
- [x] `rogo serve` can instead run in stdio-pipe mode (a documented
      flag/argument), speaking the identical framed protocol over
      stdin/stdout with no socket created.
- [x] Robot-name resolution follows the hello/identify-response-first,
      flag-override-second, fixed-default-for-`--sim`-third order
      described above.
- [x] A client (this ticket's own test, or ticket 008's real client)
      can connect to either transport and complete one request/reply
      exchange.

## Implementation Plan

**Approach**: Two thin listener implementations wrapping ticket 005's
server core — one binding/accepting on a `socket.AF_UNIX` path, one
reading/writing `sys.stdin`/`sys.stdout` directly — both decoding
requests and encoding replies via `daemon_protocol` (ticket 004) and
handing decoded requests to the same server core loop.

**Files to modify**:
- `src/host/rogo/daemon.py` — add the two listener implementations and
  robot-name/socket-path resolution.

**Testing plan**: New tests in `tests/host/rogo/`: (1) socket-path
resolution unit tests covering both the `XDG_RUNTIME_DIR`-set and
-unset cases, and the two-distinct-robots case; (2) an end-to-end test
that starts a daemon in stdio-pipe mode as a subprocess, writes a
framed request to its stdin, and reads the framed reply from its
stdout (this is SUC-003's own stdio-pipe acceptance criterion, and the
mechanism ticket 007's sim-boot test and ticket 011's end-to-end test
both build on). Scoped run: `uv run python -m pytest -q
tests/host/rogo/ -k "daemon and (socket or pipe)"`.

**Documentation updates**: `rogo serve --help` text (argparse) covers
both transport modes; `src/host/rogo/README.md` gains a short section
on the daemon's two transports, mirroring the existing style of that
file's other subcommand sections.
