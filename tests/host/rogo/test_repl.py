"""tests/host/rogo/test_repl.py -- ticket 006's `rogo repl`: the
generic command-loop core (`rogo.repl.run()`/`_dispatch_one_line()`)
against a fake parser/dispatcher (no real `Session` at all, proving the
module's own claimed decoupling from `rogo.cli`), plus full end-to-end
runs of all three input modes (argument list, piped stdin, interactive
prompt) against the real compiled `tools/sim` binary through
`rogo.cli.cmd_repl()`'s own wiring -- this ticket's own AC #4.
"""

from __future__ import annotations

import argparse
import io

import pytest

from rogo import cli, connection, repl


# ---------------------------------------------------------------------------
# Fakes -- a minimal one-positional-argument parser and a recording
# dispatcher, standing in for `rogo.cli.build_parser()`/
# `_dispatch_repl_line()` so `repl.run()`'s own loop logic (quit/exit,
# blank/comment lines, EOF, Ctrl-C, bad-line recovery, one-session reuse)
# can be proven with no real Session, transport, or tools/sim involved.
# ---------------------------------------------------------------------------

def _token_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="fake", exit_on_error=True)
    parser.add_argument("token")
    return parser


def _recording_dispatch(calls: list):
    def _dispatch(session, args) -> int:
        calls.append((session, args.token))
        return 0
    return _dispatch


def _scripted_input(responses: list[str]):
    """Mirrors test_calibrate.py's own `_scripted_input()` helper: a
    fake `input_fn` backed by an explicit, already-known list of
    responses, raising `EOFError` once exhausted -- exactly like real
    `input()` hitting a closed stream."""
    it = iter(responses)

    def _input(prompt: str = "") -> str:
        del prompt
        try:
            return next(it)
        except StopIteration:
            raise EOFError

    return _input


def _silent_print(_line: str) -> None:
    pass


# ---------------------------------------------------------------------------
# run() -- argument-list mode: every command dispatched, in order,
# against the SAME session object (this ticket's own "one persistent
# Session" requirement) -- no input_fn/stdin touched at all.
# ---------------------------------------------------------------------------

def test_argument_list_dispatches_every_command_against_the_same_session():
    session = object()
    calls: list = []
    exit_code = repl.run(
        session, ["a", "b", "c"], _token_parser(), _recording_dispatch(calls))

    assert exit_code == 0
    assert calls == [(session, "a"), (session, "b"), (session, "c")]


def test_argument_list_quit_stops_before_later_commands():
    session = object()
    calls: list = []
    exit_code = repl.run(
        session, ["a", "quit", "b"], _token_parser(), _recording_dispatch(calls))

    assert exit_code == 0
    assert calls == [(session, "a")]  # 'b' never dispatched


def test_argument_list_exit_word_also_stops():
    session = object()
    calls: list = []
    repl.run(session, ["a", "exit", "b"], _token_parser(), _recording_dispatch(calls))
    assert calls == [(session, "a")]


def test_argument_list_a_failing_command_does_not_stop_later_ones():
    # A repl's own exit code tracks session completion, not whether
    # every dispatched command individually succeeded (module docstring).
    session = object()
    calls: list = []

    def _failing_then_recording(session_, args) -> int:
        calls.append(args.token)
        return 1 if args.token == "bad" else 0

    exit_code = repl.run(session, ["bad", "good"], _token_parser(), _failing_then_recording)
    assert exit_code == 0
    assert calls == ["bad", "good"]


# ---------------------------------------------------------------------------
# run() -- blank lines and '#' comments are ignored, in every mode --
# same code path handles argument-list entries and input_fn-sourced
# lines alike.
# ---------------------------------------------------------------------------

def test_blank_and_comment_entries_are_ignored_in_argument_list_mode():
    session = object()
    calls: list = []
    repl.run(session, ["", "  ", "# a comment", "a"], _token_parser(), _recording_dispatch(calls))
    assert calls == [(session, "a")]


def test_blank_and_comment_lines_are_ignored_in_prompt_mode():
    session = object()
    calls: list = []
    input_fn = _scripted_input(["", "# nope", "a", ""])  # EOF after "a" + one blank
    exit_code = repl.run(
        session, [], _token_parser(), _recording_dispatch(calls),
        input_fn=input_fn, print_fn=_silent_print, isatty=True)
    assert exit_code == 0
    assert calls == [(session, "a")]


# ---------------------------------------------------------------------------
# run() -- piped stdin / interactive prompt: same per-line code path
# (this ticket's own AC #2: "no separate grammar to maintain") --
# proven here by driving the identical sequence through `input_fn` with
# `isatty` flipped both ways and observing identical dispatch results;
# `isatty` only changes the cosmetic prompt string.
# ---------------------------------------------------------------------------

def test_prompt_mode_dispatches_until_eof():
    session = object()
    calls: list = []
    input_fn = _scripted_input(["a", "b"])  # then EOFError
    exit_code = repl.run(
        session, [], _token_parser(), _recording_dispatch(calls),
        input_fn=input_fn, print_fn=_silent_print, isatty=True)
    assert exit_code == 0
    assert calls == [(session, "a"), (session, "b")]


def test_piped_and_interactive_modes_dispatch_identically(monkeypatch):
    session = object()
    lines = ["a", "b", "quit"]

    interactive_calls: list = []
    repl.run(session, [], _token_parser(), _recording_dispatch(interactive_calls),
              input_fn=_scripted_input(list(lines)), print_fn=_silent_print, isatty=True)

    piped_calls: list = []
    repl.run(session, [], _token_parser(), _recording_dispatch(piped_calls),
              input_fn=_scripted_input(list(lines)), print_fn=_silent_print, isatty=False)

    assert interactive_calls == piped_calls == [(session, "a"), (session, "b")]


def test_prompt_uses_the_tty_prompt_string_only_when_isatty(monkeypatch):
    seen_prompts: list[str] = []

    def _input(prompt: str = "") -> str:
        seen_prompts.append(prompt)
        raise EOFError

    repl.run(object(), [], _token_parser(), _recording_dispatch([]),
              input_fn=_input, print_fn=_silent_print, isatty=True)
    repl.run(object(), [], _token_parser(), _recording_dispatch([]),
              input_fn=_input, print_fn=_silent_print, isatty=False)

    assert seen_prompts == ["rogo> ", ""]


def test_prompt_mode_quit_stops_and_never_reads_input_again():
    session = object()
    calls: list = []
    read_count = {"n": 0}

    def _input(prompt: str = "") -> str:
        del prompt
        read_count["n"] += 1
        return ["a", "quit"][read_count["n"] - 1]

    exit_code = repl.run(
        session, [], _token_parser(), _recording_dispatch(calls),
        input_fn=_input, print_fn=_silent_print, isatty=True)

    assert exit_code == 0
    assert calls == [(session, "a")]
    assert read_count["n"] == 2  # never called a 3rd time after 'quit'


# ---------------------------------------------------------------------------
# run() -- clean shutdown on EOF, quit/exit, and Ctrl-C (this ticket's
# own AC #3). `run()` never raises: `cli.cmd_repl()`'s own
# `finally: conn.transport.close()` is what actually tears the
# connection down, on every one of these exit paths.
# ---------------------------------------------------------------------------

def test_prompt_mode_eof_on_first_read_ends_cleanly_with_no_dispatch():
    calls: list = []
    exit_code = repl.run(
        object(), [], _token_parser(), _recording_dispatch(calls),
        input_fn=_scripted_input([]), print_fn=_silent_print, isatty=True)
    assert exit_code == 0
    assert calls == []


def test_keyboard_interrupt_while_reading_input_ends_cleanly():
    def _raise_interrupt(prompt: str = "") -> str:
        del prompt
        raise KeyboardInterrupt

    calls: list = []
    exit_code = repl.run(
        object(), [], _token_parser(), _recording_dispatch(calls),
        input_fn=_raise_interrupt, print_fn=_silent_print, isatty=True)
    assert exit_code == 0
    assert calls == []


def test_keyboard_interrupt_during_dispatch_ends_cleanly():
    def _interrupting_dispatch(session, args):
        raise KeyboardInterrupt

    exit_code = repl.run(
        object(), [], _token_parser(), _interrupting_dispatch,
        input_fn=_scripted_input(["a"]), print_fn=_silent_print, isatty=True)
    assert exit_code == 0


# ---------------------------------------------------------------------------
# run() -- a per-line parse error (unknown flag, wrong arity, --help) is
# reported (argparse's own SystemExit) without ending the repl or
# crashing the process -- only EOF/Ctrl-C/quit/exit does (module
# docstring).
# ---------------------------------------------------------------------------

def test_malformed_line_is_reported_but_does_not_end_the_repl(capsys):
    session = object()
    calls: list = []
    # _token_parser() takes exactly one positional -- "a b" has two,
    # which argparse rejects as an unrecognized extra argument.
    input_fn = _scripted_input(["a b", "ok"])
    exit_code = repl.run(
        session, [], _token_parser(), _recording_dispatch(calls),
        input_fn=input_fn, print_fn=_silent_print, isatty=True)

    assert exit_code == 0
    assert calls == [(session, "ok")]  # the bad line was skipped, not fatal
    err = capsys.readouterr().err
    assert "unrecognized arguments" in err  # argparse's own message


def test_help_flag_mid_repl_is_swallowed_not_fatal(capsys):
    session = object()
    calls: list = []
    input_fn = _scripted_input(["--help", "ok"])
    exit_code = repl.run(
        session, [], _token_parser(), _recording_dispatch(calls),
        input_fn=input_fn, print_fn=_silent_print, isatty=True)

    assert exit_code == 0
    assert calls == [(session, "ok")]


def test_unclosed_quote_parse_error_is_reported_but_not_fatal():
    session = object()
    calls: list = []
    printed: list[str] = []
    input_fn = _scripted_input(['"unterminated', "ok"])
    exit_code = repl.run(
        session, [], _token_parser(), _recording_dispatch(calls),
        input_fn=input_fn, print_fn=printed.append, isatty=True)

    assert exit_code == 0
    assert calls == [(session, "ok")]
    assert any("parse error" in line for line in printed)


# ---------------------------------------------------------------------------
# End to end, all three input modes, against the real compiled
# `tools/sim` binary via `rogo.cli`'s own `repl` subcommand wiring --
# this ticket's own AC #1/#2/#4.
# ---------------------------------------------------------------------------

def test_repl_argument_list_runs_drive_then_stop_over_one_connection_against_sim(
    built_sim_binary, monkeypatch, capsys
):
    del built_sim_binary
    calls = {"n": 0}
    real_resolve = connection.resolve

    def _counting_resolve(args):
        calls["n"] += 1
        return real_resolve(args)

    monkeypatch.setattr(connection, "resolve", _counting_resolve)

    exit_code = cli.main(["repl", "--sim", "drive 100 100 --ms 200", "stop"])

    assert exit_code == 0
    assert calls["n"] == 1  # exactly one connection for both commands
    out = capsys.readouterr().out
    assert "WHEELS_V acked" in out
    assert "STOP acked" in out


def test_repl_piped_stdin_dispatches_through_the_same_parser_against_sim(
    built_sim_binary, monkeypatch, capsys
):
    del built_sim_binary
    monkeypatch.setattr("sys.stdin", io.StringIO("hello\nstop\n"))

    exit_code = cli.main(["repl", "--sim"])

    assert exit_code == 0
    out = capsys.readouterr().out
    assert "name=sim" in out  # hello's own device banner
    assert "STOP acked" in out


def test_repl_interactive_prompt_dispatches_through_the_same_parser_against_sim(
    built_sim_binary, monkeypatch, capsys
):
    # `input_fn` defaults to real `input()` either way (piped or tty) --
    # this test only needs `builtins.input` scripted, not a forced
    # `isatty()`, since dispatch is identical in both modes (module
    # docstring; also proven directly, with no sim, by
    # test_piped_and_interactive_modes_dispatch_identically() above).
    del built_sim_binary
    monkeypatch.setattr("builtins.input", _scripted_input(["hello", "stop", "quit"]))

    exit_code = cli.main(["repl", "--sim"])

    assert exit_code == 0
    out = capsys.readouterr().out
    assert "name=sim" in out
    assert "STOP acked" in out


def test_repl_closes_cleanly_on_explicit_quit_against_sim(built_sim_binary, capsys):
    del built_sim_binary
    exit_code = cli.main(["repl", "--sim", "hello", "quit", "stop"])

    assert exit_code == 0
    out = capsys.readouterr().out
    assert "name=sim" in out
    assert "STOP acked" not in out  # 'quit' stopped the loop before 'stop' ran


def test_repl_closes_cleanly_on_eof_against_sim(built_sim_binary, monkeypatch, capsys):
    del built_sim_binary
    monkeypatch.setattr("sys.stdin", io.StringIO(""))  # immediate EOF, no commands

    exit_code = cli.main(["repl", "--sim"])

    assert exit_code == 0
    out = capsys.readouterr().out
    assert out == ""  # nothing dispatched


def test_repl_end_to_end_help_flag_mid_session_does_not_kill_the_repl(
    built_sim_binary, capsys
):
    # A malformed/`--help` line inside an argument-list repl invocation
    # must not raise SystemExit out of cli.main() itself -- only the
    # repl's own quit/exit/EOF should end the session.
    del built_sim_binary
    exit_code = cli.main(["repl", "--sim", "drive --help", "stop"])

    assert exit_code == 0
    out = capsys.readouterr().out
    assert "STOP acked" in out
