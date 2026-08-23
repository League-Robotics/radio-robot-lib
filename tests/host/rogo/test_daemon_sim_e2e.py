"""tests/host/rogo/test_daemon_sim_e2e.py -- ticket 007's own SUC-003
end-to-end scenario and AC #1/#3: fork a `rogo serve --sim
--stdio-pipe`-equivalent daemon as a REAL child process
(`daemon_test_helpers.fork_stdio_daemon()`, this ticket's own reusable
harness), exchange a request/reply cycle with it over the real framed
wire, and confirm dispatch reaches the sim-backed connection.

AC #1 ("`rogo serve --sim` reaches a working daemon with no manually
started `tools/sim` process") is proven by every test below: nothing
here starts `tools/sim` itself -- the forked subprocess's own
`rogo.daemon.run_stdio_pipe_from_args()` resolves `--sim` via
`rogo.connection.resolve()`, the EXACT function every other `--sim`
end-to-end test in this directory already relies on
(`test_end_to_end_sim.py`, `test_daemon_transports.py`'s own subprocess
test) -- so a passing test here means the whole "start sim -> start
daemon against it -> talk to the daemon" flow (issue Requirement 3)
already works with no separate `tools/sim` process for a human/CI step
to start first.

AC #3 ("SUC-003's full flow ... passes as an end-to-end test") is
`test_forked_sim_daemon_hello_reaches_the_sim_backed_connection` below:
"hello" round-trips through `daemon.resolve_robot_name()` against the
REAL `tools/sim` subprocess the forked daemon itself spawned via
`--sim`, so a resolved name of `"sim"` can only mean dispatch really
reached that connection, not a stub.
"""

from __future__ import annotations

import pytest

# `forked_sim_daemon` is a fixture (daemon_test_helpers.py) -- importing
# it here, so it is bound in THIS module's own namespace, is what
# registers it with pytest for the tests below (pytest only discovers a
# fixture function that is an attribute of the requesting test module
# itself); it is never called directly, only requested by name as a
# test parameter, hence no other reference to it below.
from daemon_test_helpers import ForkedDaemonError, fork_stdio_daemon, forked_sim_daemon  # noqa: F401


def test_forked_sim_daemon_answers_ping_over_the_real_wire(built_sim_binary, forked_sim_daemon):
    del built_sim_binary
    reply = forked_sim_daemon.request("ping")
    assert reply.error is None
    assert reply.result == "pong"


def test_forked_sim_daemon_hello_reaches_the_sim_backed_connection(
    built_sim_binary, forked_sim_daemon
):
    # SUC-003's own Main Flow: fork the daemon in pipe mode, exchange a
    # request/reply cycle, confirm dispatch reaches the sim-backed
    # connection.
    del built_sim_binary
    reply = forked_sim_daemon.request("hello")
    assert reply.error is None
    assert reply.result == {"name": "sim"}


def test_forked_sim_daemon_process_exits_cleanly_on_close(built_sim_binary):
    del built_sim_binary
    forked = fork_stdio_daemon()
    try:
        reply = forked.request("ping")
        assert reply.result == "pong"
    finally:
        forked.close()
    # close() waits for the child (EOF -> run_stdio_pipe_from_args()'s
    # own finally: server.stop(); conn.transport.close(), which tears
    # down the sim subprocess it spawned too) -- a clean exit, not a
    # terminate()/kill() forced one.
    assert forked.transport.process.poll() == 0


def test_forked_sim_daemon_supports_a_custom_dispatch_table(built_sim_binary):
    # This ticket's own AC #2: usable by later tickets' own tests with
    # THEIR OWN dispatch table, not just the default ping/hello pair.
    del built_sim_binary
    forked = fork_stdio_daemon(
        dispatch_source=(
            "DISPATCH_TABLE = {'shout': lambda s, p, a: p.get('text', '').upper()}"
        )
    )
    try:
        reply = forked.request("shout", params={"text": "hi"})
        assert reply.error is None
        assert reply.result == "HI"
    finally:
        forked.close()


def test_forked_sim_daemon_reports_an_unknown_verb_as_a_failed_reply(
    built_sim_binary, forked_sim_daemon
):
    del built_sim_binary
    reply = forked_sim_daemon.request("no-such-verb")
    assert reply.error is not None
    assert reply.error.type == "UnknownVerb"


def test_forked_daemon_request_raises_if_no_reply_arrives_in_time(built_sim_binary):
    # A dispatch body that never returns (simulating a hung verb) proves
    # request()'s own timeout path -- ForkedDaemonError, not a hang.
    del built_sim_binary
    forked = fork_stdio_daemon(
        dispatch_source=(
            "import time\n"
            "DISPATCH_TABLE = {'hang': lambda s, p, a: time.sleep(0.8)}"
        )
    )
    try:
        with pytest.raises(ForkedDaemonError):
            forked.request("hang", timeout=0.2)
    finally:
        forked.close()
