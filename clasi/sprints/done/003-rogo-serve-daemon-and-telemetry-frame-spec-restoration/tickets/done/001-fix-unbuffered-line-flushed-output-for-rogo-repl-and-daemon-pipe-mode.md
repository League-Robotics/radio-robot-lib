---
id: '001'
title: Fix unbuffered/line-flushed output for rogo repl and daemon pipe mode
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: rebuild-rogo-serve-daemon-on-v6-named-sockets-pipe-mode-sim.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix unbuffered/line-flushed output for rogo repl and daemon pipe mode

## Description

`rogo repl` (and, later this sprint, the daemon's own stdio pipe mode)
must always flush output line-by-line, never block-buffered when stdout
is a pipe — independent of the daemon rebuild, and small enough to land
first (issue Requirement 6: "a small standalone fix ... that could land
ahead of the daemon work"). Today, a caller piping `rogo repl`'s output
(e.g. a test harness, or an agent driving it as a subprocess) can see
output arrive in large delayed chunks instead of per-line, because
Python block-buffers stdout when it detects a non-tty. No
`PYTHONUNBUFFERED=1` workaround should be required of the caller.

## Acceptance Criteria

- [x] `rogo repl`'s output is flushed after every emitted line,
      confirmed with stdout redirected to a pipe (not a tty) and no
      `PYTHONUNBUFFERED=1` set in the test's environment.
- [x] The fix is general enough to also cover the daemon's stdio pipe
      mode once ticket 006 adds it (either a shared helper this ticket
      introduces, or a documented pattern ticket 006 follows) — this
      ticket does not implement pipe mode itself, only the flushing
      behavior `rogo repl` needs today.
- [x] No change to `rogo repl`'s existing interactive/argument-list/
      piped-stdin behavior (SUC-001's own scope: output timing only).

## Implementation Plan

**Approach**: Flush stdout after every line `repl.py`'s `print_fn`
emits (or force line buffering at process startup, e.g.
`sys.stdout.reconfigure(line_buffering=True)` — pick whichever is less
invasive once the actual `print_fn`/output call sites in `repl.py` are
in view; either satisfies the acceptance criteria).

**Files to modify**:
- `src/host/rogo/repl.py` — the line-flush fix itself.

**Testing plan**: New test in `tests/host/rogo/` that runs `repl.py`'s
command loop with stdout captured via a pipe (not `capsys`'s default
tty-like capture, which can mask this class of bug) and asserts each
line is visible before the next command is dispatched. Scoped run:
`uv run python -m pytest -q tests/host/rogo/ -k repl`.

**Documentation updates**: None required — this is a behavior fix with
no interface change; `repl.py`'s own module docstring does not currently
claim a buffering guarantee to update.
