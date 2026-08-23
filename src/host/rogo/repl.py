"""repl.py -- run rogo commands over one persistent connection, from an
argument list, piped stdin, or an interactive prompt (sprint.md's
Architecture Step 3, `rogo.repl`'s own row; ticket 006's own
Description: "Since protocol v6 is a single plain-ASCII grammar (no
COBS/CRC/protobuf translation needed, unlike elite's binary-plane
`RogoSession`/repl machinery), this module is a much smaller command
loop reusing the same `rogo.cli` per-subcommand argument parsers already
built by tickets 003/004, not a reimplementation of elite's
binary-envelope translator.")

Three ways in, one grammar -- mirrors elite's own `repl.py` module
docstring (`radio-robot-elite/src/host/robot_radio/io/repl.py`) almost
verbatim, minus the binary/COBS/telemetry-recording machinery protocol
v6 has no equivalent of:

  * argument list -- `rogo repl "drive 100 100 --ms 200" stop`
  * piped stdin   -- `cat script.rogo | rogo repl`  (one command per line)
  * interactive   -- `rogo repl`  (prompts on a tty)

No second grammar to maintain: every line, from any of the three
sources, is tokenized with `shlex.split()` and parsed by the SAME
`argparse.ArgumentParser` `rogo.cli.build_parser()` builds for direct
CLI use -- one source of truth for flags/defaults/help text, and (via
the injected `dispatch` callback) the same per-verb reporting `rogo
drive`/`rogo turn`/etc. print when run directly. This module owns only
the command LOOP (read a line, parse it, dispatch it, decide whether to
keep going): `parser` and `dispatch` are both injected by the caller
(`rogo.cli.cmd_repl()`) rather than imported directly, so this module
never imports `rogo.cli` -- importing it here would create a circular
import (`cli.cmd_repl()` needs `repl.run()`, and the per-verb dispatch
logic `dispatch` calls lives in `cli.py`, which already has direct,
non-circular access to its own private per-verb helpers). This also
keeps `run()` unit-testable against a fake parser/dispatcher, with no
real `Session` or `rogo.cli` involved at all.

`quit`/`exit` end the loop (checked before tokenizing -- neither is a
`rogo.cli` subcommand); a blank line or one starting with `#` is
ignored (comments/spacing in a piped script, matching elite's own
`dispatch()`). Any per-line parse error (an unknown verb, a bad flag,
`--help`) is caught as `SystemExit` -- both argparse's own error path
(`ArgumentParser.error()` -> `self.exit(2, ...)`) and its `--help`
action raise it -- and reported without ending the repl or crashing the
whole process; only EOF (stdin closed / Ctrl-D), Ctrl-C, or an explicit
`quit`/`exit` closes the loop (ticket 006's own AC #3). The
`input_fn`/`print_fn` injection point follows `rogo.calibrate`'s own
established pattern (its module docstring) for testable interactive
loops: a test drives this exact function with scripted input, no TTY
involved.

Argument-list mode (`commands` non-empty) never touches `input_fn`/
stdin at all: each given string is dispatched in turn, and the loop ends
after the last one regardless of any individual command's own exit
code -- mirroring elite's own `run()`, whose final `return 0` does not
depend on any dispatched command's own outcome. A repl's job is to
report each command's outcome inline (already printed by whatever
`dispatch()` call handled it) and end cleanly; it is not a test runner
that fails the whole session over one rejected command. `piped stdin`
and the `interactive prompt` share the exact same per-line code path
too (both just call `input_fn(prompt)` in a loop until `EOFError`) --
real `input()` behaves identically whether stdin is a tty or a pipe, so
there is no separate branch to maintain there either; `isatty` only
picks the cosmetic prompt string (`"rogo> "` vs `""`), never behavior.
"""

from __future__ import annotations

import argparse
import shlex
import sys
from typing import Callable, Sequence

from robot_v6.reliability import Session

_QUIT_WORDS = frozenset({"quit", "exit"})


class _Stop:
    """Sentinel `_dispatch_one_line()` returns to tell `run()`'s own
    loop to stop -- distinct from any real command's own integer exit
    code (which `run()` deliberately discards either way, per this
    module's own docstring on why a repl's process exit code does not
    track individual command outcomes)."""


_STOP = _Stop()


def _force_line_buffered_stdout() -> None:
    """Force `sys.stdout` into line-buffered mode so every line this
    process writes -- both `print_fn`'s own output below AND, more
    importantly, the per-verb `print()` calls `dispatch()` makes deep
    inside `rogo.cli` (e.g. `_run_hello()`/`_run_stop()`/etc, which never
    go through `print_fn` at all) -- reaches a piped stdout immediately
    instead of sitting in CPython's default block-buffer (which
    `sys.stdout` picks automatically whenever `isatty()` is false at
    process startup: piped, redirected-to-file, or a test harness
    reading a subprocess's stdout). Without this, a caller piping
    `rogo repl`'s output sees it arrive in large delayed chunks -- often
    only at process exit -- instead of per-line, and the only workaround
    is the caller setting `PYTHONUNBUFFERED=1`, which this module must
    not require (ticket 003-001).

    Called once per `run()` invocation rather than importing `rogo.cli`
    to patch there: `dispatch()` is injected, so this is the one place
    both call sites share (see module docstring on why `repl.py` never
    imports `cli.py`). Ticket 006's daemon stdio-pipe mode does not
    reuse this loop at all (it is a separate framed request/reply
    listener, not `run()`) -- it should call this SAME helper, or apply
    `sys.stdout.reconfigure(line_buffering=True)` itself, before writing
    its own first reply.

    `reconfigure()` implicitly flushes any already-buffered bytes before
    changing mode, so this is safe even if something was written to
    `sys.stdout` earlier in the process. Guarded because `sys.stdout` is
    not always a real `io.TextIOWrapper` -- e.g. some embeddings replace
    it with a stream that has no `reconfigure()` method -- in which case
    this is a deliberate no-op rather than a startup crash; those
    replacement streams (including pytest's own `capsys`) are almost
    always already unbuffered/write-through, so the flushing guarantee
    this exists for still holds."""
    reconfigure = getattr(sys.stdout, "reconfigure", None)
    if reconfigure is None:
        return
    try:
        reconfigure(line_buffering=True)
    except (ValueError, OSError):
        pass


def run(
    session: Session,
    commands: Sequence[str],
    parser: argparse.ArgumentParser,
    dispatch: Callable[[Session, argparse.Namespace], int],
    *,
    input_fn: Callable[[str], str] | None = None,
    print_fn: Callable[[str], None] | None = None,
    isatty: bool | None = None,
) -> int:
    """Run `commands` (argument-list mode) if given, else read lines from
    `input_fn` (defaulting to real `input()`, which reads piped stdin
    exactly as well as a real tty) until EOF, `quit`/`exit`, or
    Ctrl-C. Every line is tokenized and parsed by `parser`, then handed
    to `dispatch(session, args)`. Always returns 0 -- a repl's own exit
    code reports whether the SESSION completed cleanly, not whether
    every dispatched command individually succeeded (see module
    docstring); a genuinely dead connection surfaces as `TransportClosed`
    propagating out of `dispatch()`, which this function does NOT catch
    -- `rogo.cli.cmd_repl()`'s own `finally: conn.transport.close()`
    (the same pattern every other `cmd_*()` in that module already uses)
    is what tears the connection down, on any exit path.

    Forces `sys.stdout` into line-buffered mode on entry (see
    `_force_line_buffered_stdout()`) -- ticket 003-001's own fix, so no
    caller needs `PYTHONUNBUFFERED=1` to see `rogo repl`'s output
    arrive per-line rather than in delayed block-buffered chunks."""
    _force_line_buffered_stdout()
    _input = input_fn if input_fn is not None else input
    _print = print_fn if print_fn is not None else print
    _isatty = isatty if isatty is not None else sys.stdin.isatty()

    try:
        if commands:
            for line in commands:
                if _dispatch_one_line(session, line, parser, dispatch, _print) is _STOP:
                    break
            return 0

        prompt = "rogo> " if _isatty else ""
        while True:
            try:
                line = _input(prompt)
            except EOFError:
                break
            if _dispatch_one_line(session, line, parser, dispatch, _print) is _STOP:
                break
        return 0
    except KeyboardInterrupt:
        _print("")  # move past a bare ^C already echoed to the terminal
        return 0


def _dispatch_one_line(
    session: Session,
    line: str,
    parser: argparse.ArgumentParser,
    dispatch: Callable[[Session, argparse.Namespace], int],
    print_fn: Callable[[str], None],
) -> _Stop | None:
    """Handle exactly one line: blank/`#`-comment lines are ignored,
    `quit`/`exit` request a stop, anything else is tokenized and parsed
    by `parser` then handed to `dispatch()`. Returns `_STOP` to end the
    loop, `None` to keep going -- never a command's own exit code (see
    `run()`'s own docstring)."""
    text = line.strip()
    if not text or text.startswith("#"):
        return None
    if text in _QUIT_WORDS:
        return _STOP
    try:
        tokens = shlex.split(text)
    except ValueError as exc:
        print_fn(f"error: parse error: {exc}")
        return None
    if not tokens:
        return None
    try:
        args = parser.parse_args(tokens)
    except SystemExit:
        # argparse's own error() path (an unknown verb, a bad flag) and
        # its --help action both raise this -- either way, it already
        # printed its own usage/error/help text to stderr/stdout and
        # would otherwise kill this whole process. One bad line must
        # not end the repl (module docstring: only EOF/Ctrl-C/quit/exit
        # does).
        return None
    dispatch(session, args)
    return None
