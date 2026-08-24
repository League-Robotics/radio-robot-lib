"""tests/host/rogo/test_cli.py -- `rogo.cli`'s argparse wiring and
ticket 002's own smoke-test subcommands (`hello`/`stop`), end to end
against the real compiled `tools/sim` binary (`built_sim_binary`,
tests/host/rogo/conftest.py) -- the simplest possible proof the whole
stack (`rogo.cli` -> `rogo.connection` -> `robot_v6.motion`/
`reliability`/`transport` -> a real subprocess) works together, per
this ticket's own acceptance criteria.
"""

from __future__ import annotations

import pytest

from rogo import cli


def test_help_lists_hello_and_stop(capsys):
    with pytest.raises(SystemExit) as exc_info:
        cli.main(["--help"])
    assert exc_info.value.code == 0
    out = capsys.readouterr().out
    assert "hello" in out
    assert "stop" in out
    assert "estop" in out


def test_estop_help_distinguishes_it_from_the_sequenced_stop(capsys):
    # AC: `rogo estop --help` must make clear estop is the unsequenced
    # panic stop, distinct from the sequenced `stop`.
    with pytest.raises(SystemExit) as exc_info:
        cli.main(["estop", "--help"])
    assert exc_info.value.code == 0
    out = capsys.readouterr().out
    assert "ESTOP" in out
    assert "stop" in out.lower()


def test_no_command_is_a_usage_error():
    with pytest.raises(SystemExit) as exc_info:
        cli.main([])
    assert exc_info.value.code != 0


def test_hello_end_to_end_against_sim(built_sim_binary, capsys):
    del built_sim_binary
    exit_code = cli.main(["hello", "--sim"])
    out = capsys.readouterr().out
    assert exit_code == 0
    assert "name=sim" in out


def test_stop_end_to_end_against_sim(built_sim_binary, capsys):
    del built_sim_binary
    exit_code = cli.main(["stop", "--sim"])
    out = capsys.readouterr().out
    assert exit_code == 0
    assert "acked" in out


def test_estop_end_to_end_against_sim(built_sim_binary, capsys):
    # Ticket 004-001: `estop` sends the unsequenced ESTOP and reports
    # the robot's bare `estop` confirmation line
    # (protocol_handler.cpp's `writeLine("estop\n")`) -- exit 0 on the
    # confirmed path, same shape as hello's/stop's own smoke tests.
    del built_sim_binary
    exit_code = cli.main(["estop", "--sim"])
    out = capsys.readouterr().out
    assert exit_code == 0
    assert "ESTOP sent" in out
    assert "confirmed" in out


def test_hello_without_a_target_reports_a_clear_error(capsys):
    exit_code = cli.main(["hello"])
    err = capsys.readouterr().err
    assert exit_code == 2
    assert "no target specified" in err
