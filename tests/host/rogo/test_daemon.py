"""tests/host/rogo/test_daemon.py -- `rogo.daemon.DaemonServer`: the
server core ticket 005 builds (connection ownership, dispatch
injection, estop-priority queue). No listener transport exists yet
(ticket 006) so every test here drives `DaemonServer.submit()`
directly, with a FAKE/mocked `DispatchTable` -- exactly the ticket's
own Testing plan ("an in-process fake transport ... and a fake/mocked
dispatch table").

Threading tests below are deterministic by construction, not by
guessed sleeps: a dispatch body signals `threading.Event`s at the
moments a test needs to observe (started / released / aborted), and
queue-state synchronization (waiting for a request to be enqueued
before enqueueing the next one) polls the server's own public
`pending_count` against a bounded deadline -- never a blind
`time.sleep()`-and-hope.
"""

from __future__ import annotations

import threading
import time

import pytest

from rogo import daemon
from rogo import daemon_protocol as dp
from rogo.connection import Connection
from robot_v6.reliability import Session
from robot_v6.transport import Transport


# ---------------------------------------------------------------------------
# A Connection whose Transport is never actually touched -- every dispatch
# table in this file is a fake that operates purely on `params`/`abort`, so
# this only needs to satisfy Transport's abstract interface, not behave like
# a real one.
# ---------------------------------------------------------------------------

class _FakeTransport(Transport):
    def _read_chunk(self, timeout: float | None) -> bytes:
        return b""

    def _write_bytes(self, data: bytes) -> None:
        pass

    def close(self) -> None:
        pass


def _make_connection() -> Connection:
    transport = _FakeTransport()
    return Connection(transport=transport, session=Session(transport))


def _wait_until(predicate, *, timeout: float = 2.0, interval: float = 0.005) -> bool:
    """Poll `predicate()` until it is true or `timeout` elapses --
    used ONLY to synchronize test setup on the server's own observable
    state (`pending_count`), never to assert the behavior under test
    itself (those assertions are plain equality checks on recorded
    events, below)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


# ---------------------------------------------------------------------------
# Connection ownership: one Session, reused for every request.
# ---------------------------------------------------------------------------

def test_server_reuses_the_same_session_for_every_request():
    connection = _make_connection()
    seen_sessions = []

    def recorder(session, params, abort):
        seen_sessions.append(session)
        return None

    server = daemon.DaemonServer(connection, {"ping": recorder})
    server.start()
    try:
        for i in range(5):
            reply = server.submit(dp.Request(id=i, verb="ping"))
            assert reply.error is None
    finally:
        server.stop()

    assert len(seen_sessions) == 5
    assert all(s is connection.session for s in seen_sessions)


def test_server_exposes_the_connection_and_session_it_was_constructed_with():
    connection = _make_connection()
    server = daemon.DaemonServer(connection, {})
    assert server.connection is connection
    assert server.session is connection.session


# ---------------------------------------------------------------------------
# Dispatch injection: no import of rogo.cli anywhere in daemon.py.
# ---------------------------------------------------------------------------

def test_module_has_no_dependency_on_cli():
    assert not hasattr(daemon, "cli")
    with open(daemon.__file__, encoding="utf-8") as f:
        source_text = f.read()
    for forbidden in ("import cli", "from . import cli", "from rogo import cli", "from rogo.cli"):
        assert forbidden not in source_text, f"unexpected cli dependency: {forbidden!r}"


def test_dispatch_table_is_injected_at_construction():
    calls = []

    def handler(session, params, abort):
        calls.append(params)
        return "ok"

    server = daemon.DaemonServer(_make_connection(), {"echo": handler})
    server.start()
    try:
        reply = server.submit(dp.Request(id=1, verb="echo", params={"x": 1}))
    finally:
        server.stop()

    assert calls == [{"x": 1}]
    assert reply.result == "ok"
    assert reply.error is None


# ---------------------------------------------------------------------------
# Unrecognized verbs / dispatch failures never crash the worker -- they
# become failed Replies, and the server keeps serving later requests.
# ---------------------------------------------------------------------------

def test_unknown_verb_returns_a_failed_reply():
    server = daemon.DaemonServer(_make_connection(), {})
    server.start()
    try:
        reply = server.submit(dp.Request(id=1, verb="nope"))
    finally:
        server.stop()
    assert reply.error is not None
    assert reply.error.type == "UnknownVerb"


def test_dispatch_exception_becomes_a_failed_reply_and_worker_keeps_serving():
    def boom(session, params, abort):
        raise ValueError("bad params")

    def ping(session, params, abort):
        return "pong"

    server = daemon.DaemonServer(_make_connection(), {"boom": boom, "ping": ping})
    server.start()
    try:
        failed = server.submit(dp.Request(id=1, verb="boom"))
        ok = server.submit(dp.Request(id=2, verb="ping"))
    finally:
        server.stop()

    assert failed.error is not None
    assert failed.error.type == "ValueError"
    assert failed.error.message == "bad params"
    assert ok.error is None
    assert ok.result == "pong"


# ---------------------------------------------------------------------------
# Usage errors.
# ---------------------------------------------------------------------------

def test_submit_before_start_raises():
    server = daemon.DaemonServer(_make_connection(), {})
    with pytest.raises(daemon.DaemonServerError):
        server.submit(dp.Request(id=1, verb="ping"))


def test_start_twice_raises():
    server = daemon.DaemonServer(_make_connection(), {})
    server.start()
    try:
        with pytest.raises(daemon.DaemonServerError):
            server.start()
    finally:
        server.stop()


def test_context_manager_starts_and_stops():
    calls = []

    def ping(session, params, abort):
        calls.append(1)
        return None

    with daemon.DaemonServer(_make_connection(), {"ping": ping}) as server:
        reply = server.submit(dp.Request(id=1, verb="ping"))
        assert reply.error is None

    assert calls == [1]


# ---------------------------------------------------------------------------
# FIFO ordering for ordinary (non-estop) requests -- AC: "Every other
# request type is served in arrival order (FIFO) relative to other
# non-estop requests."
# ---------------------------------------------------------------------------

def test_ordinary_requests_are_served_fifo():
    gate_started = threading.Event()
    release_gate = threading.Event()
    order = []
    order_lock = threading.Lock()

    def gate_handler(session, params, abort):
        gate_started.set()
        release_gate.wait(timeout=2.0)
        with order_lock:
            order.append("gate")
        return None

    def recorder(name):
        def _handler(session, params, abort):
            with order_lock:
                order.append(name)
            return None
        return _handler

    dispatch_table = {
        "gate": gate_handler,
        "a": recorder("a"),
        "b": recorder("b"),
        "c": recorder("c"),
    }

    server = daemon.DaemonServer(_make_connection(), dispatch_table)
    server.start()
    try:
        gate_thread = threading.Thread(target=lambda: server.submit(dp.Request(id=0, verb="gate")))
        gate_thread.start()
        assert gate_started.wait(timeout=2.0), "gate dispatch never started"

        threads = []
        for i, verb in enumerate(("a", "b", "c"), start=1):
            th = threading.Thread(target=lambda v=verb, i=i: server.submit(dp.Request(id=i, verb=v)))
            th.start()
            threads.append(th)
            # Wait until THIS request has actually landed on the queue
            # before starting the next thread, so enqueue order is
            # deterministic rather than a race between thread starts.
            assert _wait_until(lambda n=i: server.pending_count >= n), (
                f"request {verb!r} never reached the queue"
            )

        release_gate.set()
        gate_thread.join(timeout=2.0)
        for th in threads:
            th.join(timeout=2.0)
    finally:
        server.stop()

    assert order == ["gate", "a", "b", "c"]


# ---------------------------------------------------------------------------
# Estop priority: an estop submitted while other non-estop requests are
# already QUEUED (not yet started) jumps ahead of all of them.
# AC: "the queue is priority-ordered, not FIFO, for estop specifically."
# ---------------------------------------------------------------------------

def test_estop_jumps_ahead_of_already_queued_non_estop_requests():
    gate_started = threading.Event()
    release_gate = threading.Event()
    order = []
    order_lock = threading.Lock()

    def gate_handler(session, params, abort):
        gate_started.set()
        release_gate.wait(timeout=2.0)
        with order_lock:
            order.append("gate")
        return None

    def recorder(name):
        def _handler(session, params, abort):
            with order_lock:
                order.append(name)
            return None
        return _handler

    dispatch_table = {
        "gate": gate_handler,
        "turn": recorder("turn"),
        "estop": recorder("estop"),
    }

    server = daemon.DaemonServer(_make_connection(), dispatch_table)
    server.start()
    try:
        gate_thread = threading.Thread(target=lambda: server.submit(dp.Request(id=0, verb="gate")))
        gate_thread.start()
        assert gate_started.wait(timeout=2.0), "gate dispatch never started"

        turn_thread = threading.Thread(target=lambda: server.submit(dp.Request(id=1, verb="turn")))
        turn_thread.start()
        assert _wait_until(lambda: server.pending_count >= 1), "turn never reached the queue"

        estop_thread = threading.Thread(target=lambda: server.submit(dp.Request(id=2, verb="estop")))
        estop_thread.start()
        assert _wait_until(lambda: server.pending_count >= 2), "estop never reached the queue"

        release_gate.set()
        gate_thread.join(timeout=2.0)
        turn_thread.join(timeout=2.0)
        estop_thread.join(timeout=2.0)
    finally:
        server.stop()

    # "turn" was enqueued strictly before "estop" -- FIFO alone would put
    # it first. Priority ordering must still put estop ahead of it.
    assert order == ["gate", "estop", "turn"]


# ---------------------------------------------------------------------------
# The safety-critical scenario: an estop preempts a request ALREADY being
# dispatched (not merely queued), by aborting its in-progress completion
# wait, instead of waiting for it to finish naturally.
# AC: "An estop/halt request submitted while another request is
# in-progress is executed ahead of that request's completion wait."
# ---------------------------------------------------------------------------

def test_estop_preempts_an_in_progress_long_running_dispatch():
    drive_started = threading.Event()

    def fake_drive(session, params, abort):
        drive_started.set()
        # A well-behaved long-running dispatch body waits on `abort`
        # cooperatively instead of sleeping unconditionally. `Event.wait()`
        # returns True the instant `abort.set()` is called elsewhere, so
        # this resolves immediately once the estop preempts it, and only
        # hits the (generous) timeout if the abort mechanism is broken.
        aborted = abort.wait(timeout=2.0)
        return {"aborted": aborted}

    def fake_estop(session, params, abort):
        return {"stopped": True}

    server = daemon.DaemonServer(_make_connection(), {"drive": fake_drive, "estop": fake_estop})
    server.start()
    try:
        drive_reply_box = []

        def submit_drive():
            drive_reply_box.append(server.submit(dp.Request(id=1, verb="drive")))

        drive_thread = threading.Thread(target=submit_drive)
        drive_thread.start()

        assert drive_started.wait(timeout=2.0), "drive dispatch never started"

        start = time.monotonic()
        estop_reply = server.submit(dp.Request(id=2, verb="estop"))
        drive_thread.join(timeout=2.0)
        elapsed = time.monotonic() - start
    finally:
        server.stop()

    assert not drive_thread.is_alive()
    assert estop_reply.error is None
    assert estop_reply.result == {"stopped": True}

    assert len(drive_reply_box) == 1
    drive_reply = drive_reply_box[0]
    assert drive_reply.error is None
    # The drive dispatch body must have observed the abort signal, not
    # run to its own natural (2s) timeout.
    assert drive_reply.result == {"aborted": True}
    # Generous bound well under the 2s the drive body would otherwise
    # have blocked for -- proves the estop did not wait behind it.
    assert elapsed < 1.0


# ---------------------------------------------------------------------------
# Default estop verb set.
# ---------------------------------------------------------------------------

def test_default_estop_verbs_are_estop_and_halt():
    assert daemon.DEFAULT_ESTOP_VERBS == frozenset({"estop", "halt"})


def test_halt_is_treated_as_estop_priority_by_default():
    gate_started = threading.Event()
    release_gate = threading.Event()
    order = []
    order_lock = threading.Lock()

    def gate_handler(session, params, abort):
        gate_started.set()
        release_gate.wait(timeout=2.0)
        with order_lock:
            order.append("gate")
        return None

    def recorder(name):
        def _handler(session, params, abort):
            with order_lock:
                order.append(name)
            return None
        return _handler

    dispatch_table = {"gate": gate_handler, "turn": recorder("turn"), "halt": recorder("halt")}
    server = daemon.DaemonServer(_make_connection(), dispatch_table)
    server.start()
    try:
        gate_thread = threading.Thread(target=lambda: server.submit(dp.Request(id=0, verb="gate")))
        gate_thread.start()
        assert gate_started.wait(timeout=2.0)

        turn_thread = threading.Thread(target=lambda: server.submit(dp.Request(id=1, verb="turn")))
        turn_thread.start()
        assert _wait_until(lambda: server.pending_count >= 1)

        halt_thread = threading.Thread(target=lambda: server.submit(dp.Request(id=2, verb="halt")))
        halt_thread.start()
        assert _wait_until(lambda: server.pending_count >= 2)

        release_gate.set()
        gate_thread.join(timeout=2.0)
        turn_thread.join(timeout=2.0)
        halt_thread.join(timeout=2.0)
    finally:
        server.stop()

    assert order == ["gate", "halt", "turn"]


def test_custom_estop_verbs_override_the_default():
    # With a custom estop_verbs set, "panic" jumps the queue and "estop"
    # (excluded from this server's own set) is treated as an ordinary,
    # FIFO-only verb -- a purely behavioral check of the injected config,
    # no access to server internals.
    gate_started = threading.Event()
    release_gate = threading.Event()
    order = []
    order_lock = threading.Lock()

    def gate_handler(session, params, abort):
        gate_started.set()
        release_gate.wait(timeout=2.0)
        with order_lock:
            order.append("gate")
        return None

    def recorder(name):
        def _handler(session, params, abort):
            with order_lock:
                order.append(name)
            return None
        return _handler

    dispatch_table = {"gate": gate_handler, "estop": recorder("estop"), "panic": recorder("panic")}
    server = daemon.DaemonServer(
        _make_connection(), dispatch_table, estop_verbs=frozenset({"panic"}),
    )
    server.start()
    try:
        gate_thread = threading.Thread(target=lambda: server.submit(dp.Request(id=0, verb="gate")))
        gate_thread.start()
        assert gate_started.wait(timeout=2.0)

        # "estop" is submitted first but is NOT in this server's estop_verbs,
        # so it should NOT jump ahead of "panic", submitted after it.
        estop_thread = threading.Thread(target=lambda: server.submit(dp.Request(id=1, verb="estop")))
        estop_thread.start()
        assert _wait_until(lambda: server.pending_count >= 1)

        panic_thread = threading.Thread(target=lambda: server.submit(dp.Request(id=2, verb="panic")))
        panic_thread.start()
        assert _wait_until(lambda: server.pending_count >= 2)

        release_gate.set()
        gate_thread.join(timeout=2.0)
        estop_thread.join(timeout=2.0)
        panic_thread.join(timeout=2.0)
    finally:
        server.stop()

    # "estop" keeps its FIFO arrival-order slot (this server does not
    # treat it as priority); "panic" -- this server's own estop verb --
    # still jumps ahead of it despite arriving second.
    assert order == ["gate", "panic", "estop"]
