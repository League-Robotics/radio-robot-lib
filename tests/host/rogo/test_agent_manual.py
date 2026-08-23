"""tests/host/rogo/test_agent_manual.py -- pinning test for `rogo
--agent` (sprint 002 ticket 001, clasi/sprints/002-add-rogo-agent-manual/
sprint.md). Rather than a hand-maintained checklist of expected strings
(sprint.md's own Design Rationale on why a hand-written list drifts
exactly as easily as the manual it's meant to protect -- a new option
wouldn't automatically appear in a hand-written expected-list any more
than in the manual itself), this test introspects `rogo.cli.
build_parser()`'s own argparse tree -- every top-level subcommand name,
every sub-subcommand name (`config get`/`set`, `calibrate turns`/
`distance`), and every registered `option_strings` entry anywhere in the
tree -- and asserts each one appears somewhere in `agent_manual.MANUAL`.
A future subcommand/option rename or addition fails THIS test instead of
silently staling the manual.

The second test below is the plain smoke test AC #1 asks for: `rogo
--agent` alone exits 0 and prints non-empty output, resolving no target
and requiring no built `tools/sim` -- so it deliberately does not depend
on the `built_sim_binary` fixture other tests in this directory use.
"""

from __future__ import annotations

import argparse

from rogo import cli
from rogo.agent_manual import MANUAL


def _subparsers_actions(parser: argparse.ArgumentParser):
    """Yield every `_SubParsersAction` directly owned by `parser`.
    Accessing `_actions`/`_SubParsersAction` reaches into argparse's own
    implementation rather than a public API -- there is no public way to
    enumerate a parser's registered subparser choices, per this ticket's
    own Implementation Plan ("walking build_parser()'s own `_subparsers`/
    `_actions`")."""
    for action in parser._actions:  # noqa: SLF001
        if isinstance(action, argparse._SubParsersAction):  # noqa: SLF001
            yield action


def _collect_expected_strings(parser: argparse.ArgumentParser) -> set[str]:
    """Walk `parser`'s own tree -- itself, every subparser, every
    sub-subparser -- collecting every subcommand/sub-subcommand NAME
    plus every registered `option_strings` entry found anywhere. This
    function's own return value IS the pinning test's source of truth:
    whatever it finds is exactly what `MANUAL` must mention."""
    expected: set[str] = set()

    def _walk(p: argparse.ArgumentParser) -> None:
        for action in p._actions:  # noqa: SLF001
            expected.update(action.option_strings)
        for sub_action in _subparsers_actions(p):
            for name, subparser in sub_action.choices.items():
                expected.add(name)
                _walk(subparser)

    _walk(parser)
    return expected


def test_every_subcommand_and_option_appears_in_the_manual():
    parser = cli.build_parser()
    expected = _collect_expected_strings(parser)

    # Sanity check on the introspection itself -- if this ever collects
    # suspiciously few strings, the walk is broken, not the manual.
    assert len(expected) > 20

    missing = sorted(s for s in expected if s not in MANUAL)
    assert not missing, (
        f"agent_manual.MANUAL is missing coverage for: {missing!r} -- "
        "a subcommand, sub-subcommand, or option was added/renamed in "
        "rogo.cli.build_parser() without a matching MANUAL update"
    )


def test_agent_flag_prints_manual_and_exits_0_with_no_target_resolved(capsys):
    # No --sim/--connect/--port, no subcommand at all -- AC #1's own
    # wording: "alone, no other flags/subcommand". If this accidentally
    # resolved a target it would raise (no --sim built, no serial port,
    # no --connect peer) long before returning a clean 0.
    exit_code = cli.main(["--agent"])
    out = capsys.readouterr().out

    assert exit_code == 0
    assert out.strip()
    assert "rogo -- Agent Manual" in out
    assert MANUAL.strip() in out
