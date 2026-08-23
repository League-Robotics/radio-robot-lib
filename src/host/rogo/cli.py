"""cli.py -- `rogo`'s argparse entry point. Parses arguments and
dispatches to `rogo.connection`/`robot_v6.motion`; this module stays a
thin router (sprint.md's Architecture Step 3: "it routes, it does not
implement").

Ticket 002 wires exactly two subcommands -- `hello` and `stop` -- as
"the simplest possible smoke test of the whole stack end to end" (this
ticket's own description): both resolve a target via
`rogo.connection.resolve()`, exchange exactly one command/reply with it,
and print a human-readable result. Later tickets extend this module's
subcommand table (`drive`/`turn`/`goto`/`config`/`calibrate`/`repl`/
`mcp`) without changing this shape.
"""

from __future__ import annotations

import argparse
import sys
import time

from robot_v6 import motion
from robot_v6.reliability import Session
from robot_v6.transport import TransportClosed

from . import connection

_DEFAULT_TIMEOUT = 3.0  # [s] -- generous for a local subprocess/socket/serial hop


def _pump_until(session: Session, predicate, timeout: float = _DEFAULT_TIMEOUT):
    """Poll `session.pump()` until `predicate(replies_so_far)` is true
    or `timeout` seconds elapse, returning everything collected either
    way. A small local helper rather than something `reliability.py`
    itself provides: `wait_for_ack()`/`wait_for_done()` there track one
    specific numeric condition, not an arbitrary reply-verb predicate
    like "have we seen a `device` line yet" (`hello`'s own need)."""
    deadline = time.monotonic() + timeout
    collected = []
    while time.monotonic() < deadline:
        collected.extend(session.pump(0.2))
        if predicate(collected):
            break
    return collected


def cmd_hello(args: argparse.Namespace) -> int:
    """Send `HELLO`, print the resulting `device` banner. The
    simplest possible round trip: no sequencing, no motion, just proof
    the target is alive and answering (per protocol.md#8.3, `HELLO`'s
    reply is byte-identical to the unsolicited boot banner already
    emitted on connect, so this also verifies re-sending it works)."""
    conn = connection.resolve(args)
    try:
        conn.session.send_unsequenced("HELLO")
        replies = _pump_until(
            conn.session, lambda rs: any(r.verb == "device" for r in rs))
        banner = next((r for r in replies if r.verb == "device"), None)
        if banner is None:
            print("no device banner received", file=sys.stderr)
            return 1
        fields = list(banner.fields) + ["?"] * max(0, 4 - len(banner.fields))
        role, common_name, name, serial = fields[:4]
        print(f"role={role} common_name={common_name} name={name} serial={serial}")
        return 0
    except TransportClosed as exc:
        print(f"error: connection closed: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.transport.close()


def cmd_stop(args: argparse.Namespace) -> int:
    """Send the sequenced `STOP` command and report whether it was
    acked -- `rogo`'s other half of ticket 002's smoke test, exercising
    `robot_v6.motion` and the reliability layer's ack path rather than
    `hello`'s unsequenced one."""
    conn = connection.resolve(args)
    try:
        seq_id = motion.stop(conn.session)
        acked = conn.session.wait_for_ack(seq_id, timeout=_DEFAULT_TIMEOUT)
        if acked:
            print(f"STOP acked (#{seq_id})")
            return 0
        print(f"STOP sent (#{seq_id}) but not acked within "
              f"{_DEFAULT_TIMEOUT}s", file=sys.stderr)
        return 1
    except TransportClosed as exc:
        print(f"error: connection closed: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.transport.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rogo",
        description="Command-line control for a protocol-v6 robot, relay, "
                    "or tools/sim.",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_hello = sub.add_parser(
        "hello", help="Probe the target: send HELLO, print the device banner")
    connection.add_target_arguments(p_hello)
    p_hello.set_defaults(func=cmd_hello)

    p_stop = sub.add_parser(
        "stop", help="Send the sequenced STOP command")
    connection.add_target_arguments(p_stop)
    p_stop.set_defaults(func=cmd_stop)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except connection.TargetError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except connection.SimBinaryError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 3
    except OSError as exc:
        print(f"error: could not reach target: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
