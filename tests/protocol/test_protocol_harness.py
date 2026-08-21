"""tests/protocol/test_protocol_harness.py -- the protocol handler host
test harness.

Protocol::ProtocolHandler (src/protocol/protocol_handler.{h,cpp}) is
exercised entirely through a MockAdapter + RecordingSink
(tests/protocol/mock_adapter.h, protocol_shim.cpp) -- no kernel, no
motors, no transport, matching docs/design/protocol.md's own scope.

The wire grammar is the SPACE/`#id` grammar (docs/design/protocol.md
S2, stakeholder decision 2026-08-20, commit 5a5b6da) -- this file was
rewritten wholesale from an earlier colon-delimited, positional-id
grammar; see protocol_handler.h's own file header for the resolution
history of what changed and why.

Two kinds of coverage:

1. test_golden_vectors drives every scenario in golden_vectors.txt
   (docs/design/protocol.md S9.4's cross-language conformance fixture)
   through the handler and asserts the sink's captured output
   byte-for-byte.

2. The individual test_* functions below cover what a tidy golden
   vector never exercises: feed()'s byte-block-boundary contract
   (docs/design/protocol.md S2.1), the 240-byte overflow-discard rule,
   the lowercase-verb-is-another-robot's-reply drop (the DBG: flood
   incident's structural fix), blank/all-whitespace-line silence,
   unknown verbs, wrong arity, the id's three wire behaviors (omitted /
   `#0` / nonzero), the malformed-line `#id` recovery rule (including
   unknown verbs), and ESTOP's "no ack at all, ever" rule (spec S8.2).

Run with::

    uv run python -m pytest tests/protocol/test_protocol_harness.py -v -s
"""

import ctypes
import pathlib
import random
import subprocess

import pytest

# tests/protocol/test_protocol_harness.py -> protocol -> tests -> root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
_PACKAGE_DIR = _REPO_ROOT / "src" / "protocol"
_TEST_DIR = pathlib.Path(__file__).resolve().parent
_GOLDEN_VECTORS_PATH = _TEST_DIR / "golden_vectors.txt"

_SHIM_SOURCES = [
    _PACKAGE_DIR / "protocol_handler.cpp",
    _TEST_DIR / "protocol_shim.cpp",
]

# Protocol::Result's DECLARATION order (src/protocol/adapter.h) -- the
# shim passes/returns this ordinal, never a wire error code. See
# resultCode() in protocol_handler.cpp for the one place that maps
# between the two.
RESULT_OK = 0
RESULT_UNKNOWN = 1
RESULT_BADARG = 2
RESULT_RANGE = 3
RESULT_FULL = 4
RESULT_UNIMPLEMENTED = 5
RESULT_NOTREADY = 6
RESULT_BUSY = 7
RESULT_DUPLICATE_ID = 8

# Protocol::TlmMode's declaration order (adapter.h).
TLM_OFF = 0
TLM_POSE = 1
TLM_FULL = 2
TLM_NOW = 3
TLM_AUTO = 4
TLM_BUFFER = 5


def _compile_shared_lib(tmp_path, sources, include_dirs, out_name):
    lib_path = tmp_path / out_name
    cmd = ["/usr/bin/c++", "-std=c++20", "-Wall", "-Wextra", "-shared", "-fPIC"]
    for d in include_dirs:
        cmd += ["-I", str(d)]
    cmd += [str(s) for s in sources] + ["-o", str(lib_path)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    assert result.returncode == 0, (
        f"shim compile failed:\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}")
    return lib_path


def _load_shim(tmp_path):
    """Compile protocol_shim.cpp + the package into a shared library and
    bind ctypes signatures for every exported function."""
    lib_path = _compile_shared_lib(
        tmp_path, _SHIM_SOURCES, [_PACKAGE_DIR, _TEST_DIR],
        "libprotocol_shim.so")
    lib = ctypes.CDLL(str(lib_path))

    lib.phCreate.argtypes = []
    lib.phCreate.restype = ctypes.c_void_p
    lib.phDestroy.argtypes = [ctypes.c_void_p]
    lib.phDestroy.restype = None

    lib.phFeed.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    lib.phFeed.restype = None

    lib.phSendBanner.argtypes = [ctypes.c_void_p]
    lib.phSendBanner.restype = None
    lib.phSendReady.argtypes = [ctypes.c_void_p]
    lib.phSendReady.restype = None
    lib.phSendDebug.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lib.phSendDebug.restype = None

    lib.phMalformedCount.argtypes = [ctypes.c_void_p]
    lib.phMalformedCount.restype = ctypes.c_uint32

    lib.phSinkLength.argtypes = [ctypes.c_void_p]
    lib.phSinkLength.restype = ctypes.c_int
    lib.phSinkRead.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    lib.phSinkRead.restype = ctypes.c_int
    lib.phSinkClear.argtypes = [ctypes.c_void_p]
    lib.phSinkClear.restype = None

    lib.phSetIdentity.argtypes = [ctypes.c_void_p] + [ctypes.c_char_p] * 5
    lib.phSetIdentity.restype = None
    lib.phSetNow.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.phSetNow.restype = None
    lib.phSetStatus.argtypes = [
        ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int,
        ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_uint32,
        ctypes.c_char_p,
    ]
    lib.phSetStatus.restype = None
    lib.phSetGetOverride.argtypes = [
        ctypes.c_void_p, ctypes.c_char_p, ctypes.c_float]
    lib.phSetGetOverride.restype = None

    lib.phSetWheelsResult.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.phSetWheelsResult.restype = None
    lib.phSetStopResult.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.phSetStopResult.restype = None
    lib.phSetSetResult.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.phSetSetResult.restype = None
    lib.phSetTlmResult.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.phSetTlmResult.restype = None
    lib.phSetRunResult.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.phSetRunResult.restype = None
    lib.phSetRunHasResult.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.phSetRunHasResult.restype = None
    lib.phSetRunResultText.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lib.phSetRunResultText.restype = None

    lib.phWheelsCalls.argtypes = [ctypes.c_void_p]
    lib.phWheelsCalls.restype = ctypes.c_int
    lib.phLastWheelsLeft.argtypes = [ctypes.c_void_p]
    lib.phLastWheelsLeft.restype = ctypes.c_float
    lib.phLastWheelsRight.argtypes = [ctypes.c_void_p]
    lib.phLastWheelsRight.restype = ctypes.c_float
    lib.phLastWheelsDuration.argtypes = [ctypes.c_void_p]
    lib.phLastWheelsDuration.restype = ctypes.c_uint32
    lib.phLastWheelsId.argtypes = [ctypes.c_void_p]
    lib.phLastWheelsId.restype = ctypes.c_uint32

    lib.phStopCalls.argtypes = [ctypes.c_void_p]
    lib.phStopCalls.restype = ctypes.c_int
    lib.phLastStopId.argtypes = [ctypes.c_void_p]
    lib.phLastStopId.restype = ctypes.c_uint32

    lib.phEstopCalls.argtypes = [ctypes.c_void_p]
    lib.phEstopCalls.restype = ctypes.c_int

    lib.phGetCalls.argtypes = [ctypes.c_void_p]
    lib.phGetCalls.restype = ctypes.c_int

    lib.phSetCalls.argtypes = [ctypes.c_void_p]
    lib.phSetCalls.restype = ctypes.c_int
    lib.phLastSetValue.argtypes = [ctypes.c_void_p]
    lib.phLastSetValue.restype = ctypes.c_float
    lib.phLastSetId.argtypes = [ctypes.c_void_p]
    lib.phLastSetId.restype = ctypes.c_uint32
    lib.phLastSetNameMatches.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lib.phLastSetNameMatches.restype = ctypes.c_int

    lib.phTlmCalls.argtypes = [ctypes.c_void_p]
    lib.phTlmCalls.restype = ctypes.c_int
    lib.phLastTlmMode.argtypes = [ctypes.c_void_p]
    lib.phLastTlmMode.restype = ctypes.c_int

    lib.phRunCalls.argtypes = [ctypes.c_void_p]
    lib.phRunCalls.restype = ctypes.c_int
    lib.phLastRunNameMatches.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lib.phLastRunNameMatches.restype = ctypes.c_int
    lib.phLastRunArgc.argtypes = [ctypes.c_void_p]
    lib.phLastRunArgc.restype = ctypes.c_int
    lib.phLastRunArgMatches.argtypes = [
        ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p]
    lib.phLastRunArgMatches.restype = ctypes.c_int

    lib.phIdentityCalls.argtypes = [ctypes.c_void_p]
    lib.phIdentityCalls.restype = ctypes.c_int
    lib.phNowCalls.argtypes = [ctypes.c_void_p]
    lib.phNowCalls.restype = ctypes.c_int
    lib.phStatusCalls.argtypes = [ctypes.c_void_p]
    lib.phStatusCalls.restype = ctypes.c_int

    lib.phEmitTelemetry.argtypes = [
        ctypes.c_void_p, ctypes.c_int,
        ctypes.POINTER(ctypes.c_char_p),
        ctypes.POINTER(ctypes.c_int32),
        ctypes.POINTER(ctypes.c_int32),
    ]
    lib.phEmitTelemetry.restype = None

    return lib


def _feed(lib, handle, text):
    data = text.encode("ascii")
    lib.phFeed(handle, data, len(data))


def _sink_lines(lib, handle):
    """Everything the sink has captured so far, as a list of decoded
    lines (split on '\\n', with the trailing empty entry from the final
    terminator dropped)."""
    length = lib.phSinkLength(handle)
    if length == 0:
        return []
    buf = ctypes.create_string_buffer(length)
    n = lib.phSinkRead(handle, buf, length)
    assert n == length
    text = buf.raw[:length].decode("ascii")
    lines = text.split("\n")
    assert lines[-1] == "", f"sink output not newline-terminated: {text!r}"
    return lines[:-1]


# ---------------------------------------------------------------------------
# Golden vectors (spec S11.3) -- see golden_vectors.txt's own header
# comment for the file format.
# ---------------------------------------------------------------------------

def _parse_golden_vectors(path):
    """Returns a list of (setup_calls, actions, expected_out) blocks.
    setup_calls: [(key, [tokens...]), ...]
    actions: [("IN", text), ...] or [("EMIT", [tokens...]), ...]
    expected_out: [line, ...] (possibly empty, meaning "OUT NONE")
    """
    blocks = []
    setup_calls = []
    actions = []
    expected_out = []
    saw_out_none = False

    def flush():
        nonlocal setup_calls, actions, expected_out, saw_out_none
        if actions:
            blocks.append((setup_calls, actions, expected_out))
        setup_calls = []
        actions = []
        expected_out = []
        saw_out_none = False

    for raw_line in path.read_text().splitlines():
        if raw_line.strip() == "":
            flush()
            continue
        if raw_line.lstrip().startswith("#"):
            continue
        if raw_line.startswith("SETUP "):
            rest = raw_line[len("SETUP "):]
            key, _, tail = rest.partition(" ")
            setup_calls.append((key, tail.split(" ")))
        elif raw_line.startswith("IN "):
            actions.append(("IN", raw_line[len("IN "):]))
        elif raw_line.startswith("EMIT "):
            actions.append(("EMIT", raw_line[len("EMIT "):].split(" ")))
        elif raw_line == "DEBUG" or raw_line.startswith("DEBUG "):
            # DEBUG drives ProtocolHandler::sendDebug() directly (an
            # unsolicited emission, not fed through feed() -- there is
            # no wire form a host ever sends this on). Bare "DEBUG" with
            # nothing after it is the empty-text case, distinct from
            # "IN debug ..." a few lines below (which instead tests the
            # verb arriving INBOUND, i.e. dropped silently).
            text = raw_line[len("DEBUG"):]
            actions.append(("DEBUG", text[1:] if text.startswith(" ") else ""))
        elif raw_line.startswith("OUT "):
            value = raw_line[len("OUT "):]
            if value == "NONE":
                assert not expected_out, "OUT NONE must be the only OUT line"
                saw_out_none = True
            else:
                assert not saw_out_none, "OUT NONE must be the only OUT line"
                expected_out.append(value)
        else:
            raise ValueError(f"unrecognized golden-vector line: {raw_line!r}")
    flush()
    return blocks


class _GoldenVectorRunner:
    """Applies one golden-vector block's SETUP/action lines to a live
    handle. Keeps every encoded string alive for the handle's whole
    lifetime -- MockAdapter stores BORROWED pointers (mirroring
    Protocol::Identity's own contract, adapter.h), so anything passed
    to phSetIdentity/phSetStatus/phSetGetOverride must outlive every
    later call that might re-read it."""

    def __init__(self, lib, handle):
        self.lib = lib
        self.handle = handle
        self._keepalive = []

    def _keep(self, s):
        b = s.encode("ascii")
        self._keepalive.append(b)
        return b

    def apply_setup(self, key, tokens):
        lib, handle = self.lib, self.handle
        if key == "identity":
            name, serial, drivetrain, profile, version = tokens
            lib.phSetIdentity(
                handle, self._keep(name), self._keep(serial),
                self._keep(drivetrain), self._keep(profile),
                self._keep(version))
        elif key == "now":
            lib.phSetNow(handle, int(tokens[0]))
        elif key == "status":
            ready, active, connl, connr, otos, wedge, flags, tlm = tokens
            lib.phSetStatus(
                handle, int(ready), int(active), int(connl), int(connr),
                int(otos), int(wedge), int(flags), self._keep(tlm))
        elif key == "getoverride":
            name, value = tokens
            lib.phSetGetOverride(handle, self._keep(name), float(value))
        elif key == "setresult":
            lib.phSetSetResult(handle, int(tokens[0]))
        elif key == "wheelsresult":
            lib.phSetWheelsResult(handle, int(tokens[0]))
        elif key == "stopresult":
            lib.phSetStopResult(handle, int(tokens[0]))
        elif key == "runresult":
            lib.phSetRunResult(handle, int(tokens[0]))
        elif key == "runhasresult":
            lib.phSetRunHasResult(handle, int(tokens[0]))
        elif key == "runresulttext":
            lib.phSetRunResultText(handle, self._keep(tokens[0]))
        else:
            raise ValueError(f"unknown SETUP key: {key!r}")

    def apply_action(self, kind, payload):
        lib, handle = self.lib, self.handle
        if kind == "IN":
            _feed(lib, handle, payload + "\n")
        elif kind == "DEBUG":
            lib.phSendDebug(handle, self._keep(payload))
        elif kind == "EMIT":
            names, values, hexes = [], [], []
            for token in payload:
                name, hexflag, value = token.split(":")
                names.append(self._keep(name))
                hexes.append(int(hexflag))
                values.append(int(value))
            count = len(names)
            name_array = (ctypes.c_char_p * count)(*names)
            value_array = (ctypes.c_int32 * count)(*values)
            hex_array = (ctypes.c_int32 * count)(*hexes)
            lib.phEmitTelemetry(handle, count, name_array, value_array,
                                 hex_array)
        else:
            raise ValueError(f"unknown action kind: {kind!r}")


def test_golden_vectors(tmp_path):
    """Drives every scenario in golden_vectors.txt through the handler
    and asserts the sink's output matches byte-for-byte (spec S11.3)."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    runner = _GoldenVectorRunner(lib, handle)
    try:
        blocks = _parse_golden_vectors(_GOLDEN_VECTORS_PATH)
        assert blocks, "no golden vectors parsed -- fixture path or format broke"
        for index, (setup_calls, actions, expected_out) in enumerate(blocks):
            lib.phSinkClear(handle)
            for key, tokens in setup_calls:
                runner.apply_setup(key, tokens)
            for kind, payload in actions:
                runner.apply_action(kind, payload)
            actual = _sink_lines(lib, handle)
            assert actual == expected_out, (
                f"golden vector block {index} (actions={actions!r}) mismatch:\n"
                f"  expected: {expected_out!r}\n"
                f"  actual:   {actual!r}")
        print(f"golden vectors: {len(blocks)} blocks passed")
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# feed()'s byte-block-boundary contract (docs/design/protocol.md S2.1)
# ---------------------------------------------------------------------------

def test_feed_several_complete_lines_in_one_block(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetNow(handle, 111)
        _feed(lib, handle, "PING\nPING\nPING\n")
        assert lib.phNowCalls(handle) == 3
        assert _sink_lines(lib, handle) == ["pong 111"] * 3
        assert lib.phMalformedCount(handle) == 0
    finally:
        lib.phDestroy(handle)


def test_feed_block_ending_mid_line_buffers_the_remainder(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetNow(handle, 222)
        _feed(lib, handle, "PI")
        assert _sink_lines(lib, handle) == [], "dispatched before the line completed"
        assert lib.phNowCalls(handle) == 0
        _feed(lib, handle, "NG\n")
        assert _sink_lines(lib, handle) == ["pong 222"]
        assert lib.phNowCalls(handle) == 1
    finally:
        lib.phDestroy(handle)


def test_feed_fragment_alone_never_dispatches(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "WHEELS 100 100 1000")  # no terminator, ever
        assert _sink_lines(lib, handle) == []
        assert lib.phWheelsCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 0
    finally:
        lib.phDestroy(handle)


def test_feed_strips_lone_cr_before_terminator(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetNow(handle, 333)
        _feed(lib, handle, "PING\r\n")
        assert _sink_lines(lib, handle) == ["pong 333"]
        assert lib.phMalformedCount(handle) == 0
    finally:
        lib.phDestroy(handle)


def test_feed_overlong_line_discarded_not_truncated(tmp_path):
    """spec S2 / docs/design/protocol.md S2.1: a line over the 240-byte
    cap must be discarded to the next '\\n', never truncated into a
    prefix that still parses as something the host never sent. The
    line's first 23 bytes are a perfectly valid, correct-arity WHEELS
    command -- if the implementation truncated instead of discarding,
    a naive truncation could dispatch as WHEELS 100 100 1000 #5. It
    must not."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        overlong = "WHEELS 100 100 1000 #5" + ("X" * 300) + "\n"
        assert len(overlong) > 240
        _feed(lib, handle, overlong)
        assert lib.phWheelsCalls(handle) == 0, (
            "the valid-looking WHEELS prefix must NOT have dispatched")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 1

        # The parser must resync cleanly on the next line.
        lib.phSetNow(handle, 444)
        _feed(lib, handle, "PING\n")
        assert _sink_lines(lib, handle) == ["pong 444"]
    finally:
        lib.phDestroy(handle)


def test_feed_exactly_240_bytes_is_accepted(tmp_path):
    """The boundary companion to the overflow test above: a line whose
    TOTAL wire length (content + '\\n') is exactly 240 bytes -- spec
    S2's own stated maximum -- must be accepted and dispatched
    normally, not discarded."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        name = "n" * 235  # "GET " (4) + 235 == 239 content bytes;
                           # + '\n' == 240 total, exactly the cap.
        line = f"GET {name}"
        assert len(line) + 1 == 240
        lib.phSetGetOverride(handle, name.encode("ascii"), 1.5)
        _feed(lib, handle, line + "\n")
        assert lib.phMalformedCount(handle) == 0
        assert _sink_lines(lib, handle) == [f"get {name} 1.500000"]
    finally:
        lib.phDestroy(handle)


def test_feed_241_bytes_overflows(tmp_path):
    """One byte over test_feed_exactly_240_bytes_is_accepted's boundary
    must overflow -- proving the cap is exactly 240, not off by one in
    either direction."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        name = "n" * 236  # "GET " (4) + 236 == 240 content bytes;
                           # + '\n' == 241 total, one over the cap.
        line = f"GET {name}"
        assert len(line) + 1 == 241
        lib.phSetGetOverride(handle, name.encode("ascii"), 1.5)
        _feed(lib, handle, line + "\n")
        assert lib.phMalformedCount(handle) == 1
        assert lib.phGetCalls(handle) == 0, "must never have reached the adapter"
        assert _sink_lines(lib, handle) == []
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# Blank / all-whitespace lines are ignored SILENTLY (spec S2) -- new rule
# under the space grammar (a colon-delimited empty line was previously
# just "unknown verb", i.e. malformed). Expressed here as explicit
# Python string literals rather than in golden_vectors.txt, because a
# text-fixture line consisting only of trailing whitespace is exactly
# the kind of content an editor's "trim trailing whitespace" setting
# can silently destroy with no visible diff.
# ---------------------------------------------------------------------------

def test_blank_line_is_silently_ignored_not_malformed(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "\n")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 0

        # Several blank lines in a row, still silent.
        _feed(lib, handle, "\n\n\n")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 0

        # Parser must still work normally right after.
        lib.phSetNow(handle, 666)
        _feed(lib, handle, "PING\n")
        assert _sink_lines(lib, handle) == ["pong 666"]
    finally:
        lib.phDestroy(handle)


def test_all_whitespace_line_is_silently_ignored_not_malformed(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "     \n")  # spaces only, no verb at all
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 0

        lib.phSetNow(handle, 777)
        _feed(lib, handle, "PING\n")
        assert _sink_lines(lib, handle) == ["pong 777"]
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# A run of spaces is ONE separator; leading/trailing whitespace on the
# line is ignored (spec S2).
# ---------------------------------------------------------------------------

def test_space_run_between_fields_is_one_separator(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS   100   100   1000   #5\n")
        assert lib.phWheelsCalls(handle) == 1
        assert lib.phLastWheelsLeft(handle) == pytest.approx(100.0)
        assert lib.phLastWheelsRight(handle) == pytest.approx(100.0)
        assert lib.phLastWheelsDuration(handle) == 1000
        assert lib.phLastWheelsId(handle) == 5
        assert _sink_lines(lib, handle) == ["ok #5"]
        assert lib.phMalformedCount(handle) == 0
    finally:
        lib.phDestroy(handle)


def test_leading_and_trailing_line_whitespace_is_ignored(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetNow(handle, 888)
        _feed(lib, handle, "   PING   \n")
        assert _sink_lines(lib, handle) == ["pong 888"]
        assert lib.phMalformedCount(handle) == 0
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# Case is direction (spec S2.1) -- the DBG: flood incident's structural fix
# ---------------------------------------------------------------------------

def test_lowercase_verb_dropped_silently_not_malformed(tmp_path):
    """A lowercase-led inbound line is another robot's reply, overheard
    on a shared radio channel (hardware-bench-testing.md's DBG: flood
    incident). It must be dropped SILENTLY: no reply, and NOT counted
    malformed -- counting it would still trip a fault bit on a clean
    connect, per spec S9.5's identical reasoning for relay control-plane
    lines."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        # Exactly the incident's own repro: a robot's own dbg output
        # arriving as if it were inbound.
        _feed(lib, handle, "dbg something happened\n")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 0

        # A robot's own ack, echoed back on the same shared channel.
        _feed(lib, handle, "ok #5\n")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 0

        # Parser must still work normally right after.
        lib.phSetNow(handle, 555)
        _feed(lib, handle, "PING\n")
        assert _sink_lines(lib, handle) == ["pong 555"]
    finally:
        lib.phDestroy(handle)


def test_mixed_case_verb_is_unknown_not_dropped(tmp_path):
    """A verb starting UPPERCASE but not matching any known command
    (e.g. a typo) is a genuinely unknown command, not a foreign reply --
    it must be counted malformed like any other unknown verb."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "Ping\n")  # starts uppercase, not the literal PING
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 1
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# Unknown verb / wrong arity, and spec S2's malformed-line #id recovery:
# "If the line's last token is a well-formed nonzero #id, reply
# err #<id> <code> -- including unknown verbs." This is a real behavior
# CHANGE from the colon grammar: there, an unknown verb's fields (even
# if one of them happened to look like an id) were never trustworthy,
# because a colon-positional id could sit anywhere depending on an
# unknown verb's unknowable arity. The new grammar's SELF-MARKING
# trailing id (spec S8.2) makes it trustworthy regardless.
# ---------------------------------------------------------------------------

def test_unknown_verb_no_reply_when_no_recoverable_id(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "FOO\n")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 1

        # Fields present, but the last token is not `#`-shaped -- still
        # nothing to recover an id from.
        _feed(lib, handle, "FOO 1 2 3\n")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 2
    finally:
        lib.phDestroy(handle)


def test_unknown_verb_with_recoverable_id_gets_err_unknown(tmp_path):
    """The inverted case: an unknown verb whose line's LAST token is a
    well-formed nonzero #id DOES get an err reply -- ERR_UNKNOWN (code
    1), spec S2's "including unknown verbs" framing, made concrete."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "FOO 1 2 3 #42\n")
        assert _sink_lines(lib, handle) == ["err #42 1"]
        assert lib.phMalformedCount(handle) == 1
        lib.phSinkClear(handle)

        # A bare unknown verb with JUST a trailing id and no other
        # fields works the same way.
        _feed(lib, handle, "BAR #7\n")
        assert _sink_lines(lib, handle) == ["err #7 1"]
        assert lib.phMalformedCount(handle) == 2
    finally:
        lib.phDestroy(handle)


def test_unknown_verb_trailing_id_zero_gets_no_reply(tmp_path):
    """`#0` is well-formed but not NONZERO, so spec S2's recovery rule
    ("well-formed nonzero #id") does not fire -- no reply, matching
    S8.2's "id 0 means no ack" spirit even for a verb the dispatcher has
    never heard of."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "FOO #0\n")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 1
    finally:
        lib.phDestroy(handle)


def test_wrong_arity_known_verb_recovers_id_too(tmp_path):
    """The same recovery rule applies to a KNOWN verb with wrong arity
    -- PING takes zero fields, so a trailing #id-shaped token is itself
    the "extra" field that makes the line malformed, and ERR_BADARG
    (code 2) is recoverable against it exactly like the unknown-verb
    case above."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "PING #5\n")
        assert _sink_lines(lib, handle) == ["err #5 2"]
        assert lib.phMalformedCount(handle) == 1
    finally:
        lib.phDestroy(handle)


def test_wrong_arity_rejected_no_best_effort_parse(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "PING extra\n")  # PING takes 0 fields
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 1

        _feed(lib, handle, "WHEELS 100 100\n")  # needs 3 or 4 fields
        assert _sink_lines(lib, handle) == []
        assert lib.phWheelsCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 2

        _feed(lib, handle, "SET name\n")  # needs 2 or 3 fields
        assert _sink_lines(lib, handle) == []
        assert lib.phSetCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 3
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# ESTOP: no ack at all, EVER (spec S8.2) -- the one deliberate exception
# to the malformed-line #id recovery rule above (protocol_handler.h
# ambiguity note #2).
# ---------------------------------------------------------------------------

def test_estop_produces_no_ack_at_all(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "ESTOP\n")
        assert lib.phSinkLength(handle) == 0, (
            "ESTOP must never write anything to the sink, ever")
        assert lib.phEstopCalls(handle) == 1
        assert lib.phMalformedCount(handle) == 0
    finally:
        lib.phDestroy(handle)


def test_estop_wrong_arity_still_produces_no_reply(tmp_path):
    """Even a malformed ESTOP carries no id (spec S3.1: `ESTOP | -`), so
    there is still nothing to ack -- wrong arity is silent here too,
    just like every other id-less verb."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "ESTOP 1\n")
        assert lib.phSinkLength(handle) == 0
        assert lib.phEstopCalls(handle) == 0, "must never have reached the adapter"
        assert lib.phMalformedCount(handle) == 1
    finally:
        lib.phDestroy(handle)


def test_estop_with_trailing_id_still_never_acks(tmp_path):
    """The sharpest version of the ESTOP carve-out: `ESTOP #5` has a
    perfectly recoverable, well-formed nonzero id per spec S2's generic
    rule -- every OTHER verb in this suite gets an err reply in this
    exact shape (see test_wrong_arity_known_verb_recovers_id_too above).
    ESTOP must not, because spec S5.4/S8.2 state its silence in
    stronger, more specific terms ("never carries an id and is never
    acked ... must not queue behind anything, including an ack") that
    this handler treats as overriding S2's general rule."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "ESTOP #5\n")
        assert lib.phSinkLength(handle) == 0, (
            "ESTOP must never ack, even with a recoverable id")
        assert lib.phEstopCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# The id's three wire behaviors (spec S8.2, protocol_handler.h's
# "Resolved-by-the-new-grammar" note): OMITTED (bare ack), explicit `#0`
# (silent), explicit nonzero (id-carrying ack). Exercised on both
# SET and WHEELS, whose id is optional.
# ---------------------------------------------------------------------------

def test_wheels_omitted_id_executes_and_acks_bare(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS 50 50 200\n")
        assert lib.phWheelsCalls(handle) == 1
        assert lib.phLastWheelsId(handle) == 0
        assert _sink_lines(lib, handle) == ["ok"], (
            "an omitted id acks ONCE, BARE -- no #id token in the reply")
    finally:
        lib.phDestroy(handle)


def test_wheels_explicit_zero_id_executes_silently(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS 50 50 200 #0\n")
        assert lib.phWheelsCalls(handle) == 1, "the adapter is still called"
        assert lib.phLastWheelsId(handle) == 0
        assert _sink_lines(lib, handle) == [], "#0 means no ack wanted"
    finally:
        lib.phDestroy(handle)


def test_wheels_nonzero_id_is_acked_with_that_id(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS 50 50 200 #42\n")
        assert lib.phLastWheelsId(handle) == 42
        assert _sink_lines(lib, handle) == ["ok #42"]
    finally:
        lib.phDestroy(handle)


def test_wheels_decodes_signed_speeds_and_passes_them_as_floats(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS -100 200 500 #9\n")
        assert lib.phWheelsCalls(handle) == 1
        assert lib.phLastWheelsLeft(handle) == pytest.approx(-100.0)
        assert lib.phLastWheelsRight(handle) == pytest.approx(200.0)
        assert lib.phLastWheelsDuration(handle) == 500
        assert _sink_lines(lib, handle) == ["ok #9"]
    finally:
        lib.phDestroy(handle)


def test_wheels_id_missing_hash_prefix_is_malformed_not_an_extra_field(tmp_path):
    """A 4th WHEELS token that is NOT `#`-shaped (e.g. a bare "9", no
    leading '#') is not a legal id spelling -- spec S8.2's id grammar is
    `'#' [0-9]+`, so this is a malformed line (WHEELS has no other use
    for a 4th positional field), not a WHEELS with id 9."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "WHEELS 50 50 200 9\n")
        assert lib.phWheelsCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == [], (
            "the malformed last token \"9\" is not itself a recoverable "
            "id (no leading '#'), so there is nothing to ack against")
    finally:
        lib.phDestroy(handle)


def test_id_rejects_leading_plus_sign(tmp_path):
    """The id's own grammar (`'#' [0-9]+`) allows no sign at all, unlike
    spec S2.2's general "every wire value is ... optionally signed" rule
    for ordinary integer fields -- "#+5" must NOT be treated as id 5."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS 50 50 200 #+5\n")
        assert lib.phWheelsCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == []
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# STOP: id REQUIRED, and `#0` is itself malformed on a required-id verb
# (spec S8.2: "#0 is legal only where the id is optional ... on
# MOVE/GOTO/STOP it is malformed").
# ---------------------------------------------------------------------------

def test_stop_requires_a_well_formed_nonzero_id(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetStopResult(handle, RESULT_OK)
        _feed(lib, handle, "STOP #7\n")
        assert lib.phStopCalls(handle) == 1
        assert lib.phLastStopId(handle) == 7
        assert _sink_lines(lib, handle) == ["ok #7"]
    finally:
        lib.phDestroy(handle)


def test_stop_explicit_zero_id_is_malformed_not_silent(tmp_path):
    """Unlike SET/WHEELS (where `#0` means "no ack wanted" and the
    command still runs), STOP's id is required, so `#0` there is
    MALFORMED -- the command must NOT run, and (since the only
    candidate id token IS the zero itself) there is nothing to recover
    an err reply against either."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "STOP #0\n")
        assert lib.phStopCalls(handle) == 0, "STOP must not have executed"
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == []
    finally:
        lib.phDestroy(handle)


def test_stop_missing_id_is_malformed(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "STOP\n")
        assert lib.phStopCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == []
    finally:
        lib.phDestroy(handle)


def test_stop_id_without_hash_prefix_is_malformed(tmp_path):
    """STOP's field grammar is exactly `#<id>` -- a bare "5" with no
    leading '#' is not a legal spelling of it at all."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "STOP 5\n")
        assert lib.phStopCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == []
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# SET: value decode is the handler's job (spec S7.2), not the adapter's
# ---------------------------------------------------------------------------

def test_set_malformed_value_never_reaches_the_adapter(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "SET group.alpha notanumber\n")
        assert lib.phSetCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == ["err 2"], (
            "id omitted -- bare err, no #id token")
    finally:
        lib.phDestroy(handle)


@pytest.mark.parametrize("value", ["nan", "inf", "-inf", "1e10"])
def test_set_rejects_nan_inf_and_exponents(tmp_path, value):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, f"SET group.alpha {value}\n")
        assert lib.phSetCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == ["err 2"]
    finally:
        lib.phDestroy(handle)


def test_set_unknown_name_is_the_adapters_call_not_the_handlers(tmp_path):
    """docs/design/protocol.md S6: "which names are valid is entirely
    the adapter's business" -- the mock's onSet() always answers (it
    doesn't consult the GET field table), so an unrecognized name is
    only "unknown" if the ADAPTER says kUnknown."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetSetResult(handle, RESULT_UNKNOWN)
        _feed(lib, handle, "SET no.such.field 1.0\n")
        assert lib.phSetCalls(handle) == 1
        assert lib.phLastSetNameMatches(handle, b"no.such.field")
        assert lib.phMalformedCount(handle) == 0, (
            "a well-formed line the adapter rejects is not a malformed line")
        assert _sink_lines(lib, handle) == ["err 1"]
    finally:
        lib.phDestroy(handle)


def test_set_id_without_hash_prefix_is_malformed(tmp_path):
    """A 3rd SET token that is not `#`-shaped is not a legal id -- SET
    has no other use for a 3rd positional field, so the whole line is
    malformed (not "SET with an odd id")."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "SET group.alpha 1.0 9\n")
        assert lib.phSetCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == []
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# GET: unknown name has no wire outcome at all (spec S7.1, stated
# explicitly there as of the 2026-08-20 grammar switch)
# ---------------------------------------------------------------------------

def test_get_unknown_name_is_silent_and_not_malformed(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "GET no.such.field\n")
        assert lib.phGetCalls(handle) == 1  # the adapter WAS asked
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 0
    finally:
        lib.phDestroy(handle)


def test_bare_get_dumps_every_field(tmp_path):
    """spec S7.1: "A bare GET dumps every field." MockAdapter's fixed
    4-row table (mock_adapter.h) stands in for the real 80-row one."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "GET\n")
        assert _sink_lines(lib, handle) == [
            "get group.alpha 1.500000",
            "get group.beta -2.250000",
            "get group.gamma 0.000000",
            "get group.delta 100.000000",
        ]
    finally:
        lib.phDestroy(handle)


def test_get_wrong_arity_recovers_id_via_the_generic_rule(tmp_path):
    """GET's own grammar has no id column at all (`GET | [name]`, spec
    S3.1), but the generic malformed-line recovery rule (spec S2) does
    not carve out verbs without one: a 2-field GET is wrong arity, and
    if its last token happens to be a well-formed nonzero #id, that id
    is still recoverable for the err reply."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "GET group.alpha extra #5\n")
        assert lib.phGetCalls(handle) == 0, "wrong arity -- never reached onGet()"
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == ["err #5 2"]
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# TLM: no wire outcome either (spec S3.1 -- no id field at all)
# ---------------------------------------------------------------------------

def test_tlm_valid_mode_calls_adapter_with_no_reply(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "TLM FULL\n")
        assert lib.phTlmCalls(handle) == 1
        assert lib.phLastTlmMode(handle) == TLM_FULL
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 0
    finally:
        lib.phDestroy(handle)


def test_tlm_unknown_mode_is_malformed(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "TLM WARP\n")
        assert lib.phTlmCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == []
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# Telemetry: thdr: fires once per distinct column set (spec S6.2)
# ---------------------------------------------------------------------------

def test_emit_telemetry_header_reprinted_only_on_column_change(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    runner = _GoldenVectorRunner(lib, handle)
    try:
        runner.apply_action("EMIT", ["seq:0:1", "now:0:1000", "x:0:5"])
        runner.apply_action("EMIT", ["seq:0:2", "now:0:1001", "x:0:6"])
        assert _sink_lines(lib, handle) == [
            "thdr seq now x",
            "t 1 1000 5",
            "t 2 1001 6",
        ]

        lib.phSinkClear(handle)
        # A DIFFERENT column set (mode change, e.g. POSE -> FULL) must
        # emit a fresh thdr: before its own t:.
        runner.apply_action(
            "EMIT", ["seq:0:3", "now:0:1002", "x:0:7", "y:0:8"])
        assert _sink_lines(lib, handle) == ["thdr seq now x y", "t 3 1002 7 8"]
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# Chunk-split equivalence (docs/design/protocol.md S2.1) -- the single
# most valuable invariant feed() has, and the one a future MicroPython/
# JavaScript port is most likely to get wrong: a real transport hands
# feed() arbitrary fragments, and reassembly must be byte-identical to
# handing it the whole line in one call, no matter where the cuts fall.
#
# Checked against every FEED()-DRIVEN golden-vector block (a block whose
# actions are all "IN" -- an "EMIT" block drives emitTelemetry()
# directly, not feed(), so it says nothing about chunking and is
# skipped): once fed as one block, once byte-at-a-time, and several
# times with random cut points, using a FIXED seed so a failure
# reproduces exactly instead of flaking.
# ---------------------------------------------------------------------------

def _block_wire_bytes(actions):
    """Concatenate one golden-vector block's "IN" action text (each with
    its own '\n' terminator) into the single byte string a real
    transport would hand to feed() across that block's whole action
    sequence. Returns None for a block that isn't purely feed()-driven
    (e.g. it contains an "EMIT" action)."""
    parts = []
    for kind, payload in actions:
        if kind != "IN":
            return None
        parts.append((payload + "\n").encode("ascii"))
    return b"".join(parts)


def _run_block_feed(lib, setup_calls, wire_bytes, chunk_sizes):
    """Fresh handle; apply `setup_calls`; feed `wire_bytes` split into
    consecutive pieces of `chunk_sizes` bytes each (any bytes left over
    after `chunk_sizes` is exhausted go in one final feed() call); return
    everything the sink captured."""
    handle = lib.phCreate()
    try:
        runner = _GoldenVectorRunner(lib, handle)
        for key, tokens in setup_calls:
            runner.apply_setup(key, tokens)
        pos = 0
        for size in chunk_sizes:
            if pos >= len(wire_bytes):
                break
            chunk = wire_bytes[pos:pos + size]
            lib.phFeed(handle, chunk, len(chunk))
            pos += len(chunk)
        if pos < len(wire_bytes):
            remainder = wire_bytes[pos:]
            lib.phFeed(handle, remainder, len(remainder))
        length = lib.phSinkLength(handle)
        if length == 0:
            return b""
        buf = ctypes.create_string_buffer(length)
        n = lib.phSinkRead(handle, buf, length)
        assert n == length
        return buf.raw[:length]
    finally:
        lib.phDestroy(handle)


def _random_chunk_sizes(rng, total):
    """Split `total` bytes into randomly-sized pieces (1-5 bytes each),
    so a run crosses MANY feed()-boundary positions, not just one."""
    sizes = []
    remaining = total
    while remaining > 0:
        size = rng.randint(1, 5)
        sizes.append(size)
        remaining -= size
    return sizes


def test_feed_chunk_split_equivalence_golden_vectors(tmp_path):
    """For every feed()-driven golden-vector block: one-shot feed(),
    byte-at-a-time feed(), and several fixed-seed random chunkings must
    all produce byte-identical sink output. This is the property most
    likely to be quietly violated by a hand-rolled reassembly loop, in
    this implementation or in a later port of it, so it belongs in the
    shared fixture story rather than only in ad hoc block-boundary
    tests."""
    lib = _load_shim(tmp_path)
    blocks = _parse_golden_vectors(_GOLDEN_VECTORS_PATH)
    assert blocks, "no golden vectors parsed -- fixture path or format broke"
    rng = random.Random(20260820)  # fixed seed: a failure must reproduce
    checked = 0
    for index, (setup_calls, actions, _expected_out) in enumerate(blocks):
        wire_bytes = _block_wire_bytes(actions)
        if not wire_bytes:
            continue  # EMIT-driven block, or a block with no IN actions
        checked += 1

        baseline = _run_block_feed(lib, setup_calls, wire_bytes,
                                    [len(wire_bytes)])

        byte_at_a_time = _run_block_feed(
            lib, setup_calls, wire_bytes, [1] * len(wire_bytes))
        assert byte_at_a_time == baseline, (
            f"golden vector block {index}: byte-at-a-time feed diverged "
            f"from one-shot feed\n  one-shot:       {baseline!r}\n"
            f"  byte-at-a-time: {byte_at_a_time!r}")

        for trial in range(5):
            chunk_sizes = _random_chunk_sizes(rng, len(wire_bytes))
            chunked = _run_block_feed(lib, setup_calls, wire_bytes,
                                       chunk_sizes)
            assert chunked == baseline, (
                f"golden vector block {index} trial {trial}: random "
                f"chunking {chunk_sizes!r} diverged from one-shot feed\n"
                f"  one-shot: {baseline!r}\n  chunked:  {chunked!r}")

    assert checked > 0, "no feed()-driven golden vector blocks found to check"
    print(f"chunk-split equivalence: {checked} feed()-driven golden vector "
          f"blocks, each checked one-shot + byte-at-a-time + 5 random "
          f"chunkings (fixed seed)")


# ---------------------------------------------------------------------------
# debug: robot-to-host only (docs/design/protocol.md's debug section).
# sendDebug() is driven directly, never through feed() -- there is no
# wire form a host ever sends this on.
# ---------------------------------------------------------------------------

def test_send_debug_basic_text(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSendDebug(handle, b"something happened")
        assert _sink_lines(lib, handle) == ["debug something happened"]
    finally:
        lib.phDestroy(handle)


def test_send_debug_null_and_empty_text_are_the_same_case(tmp_path):
    """sendDebug(nullptr) and sendDebug("") are documented as the SAME
    case -- both emit the bare "debug\\n" line, no trailing space before
    the terminator."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSendDebug(handle, None)
        assert _sink_lines(lib, handle) == ["debug"]
        lib.phSinkClear(handle)

        lib.phSendDebug(handle, b"")
        assert _sink_lines(lib, handle) == ["debug"]
    finally:
        lib.phDestroy(handle)


def test_send_debug_strips_embedded_newline_and_cr(tmp_path):
    """'\\n'/'\\r' bytes in the text must never reach the sink -- they
    could forge a second line the far end would parse as a separate,
    unintended reply. Stripped, not rejected (sendDebug is void, with no
    channel to report a rejected call through)."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSendDebug(handle, b"hello\nworld\r\n")
        assert _sink_lines(lib, handle) == ["debug helloworld"]
    finally:
        lib.phDestroy(handle)


def test_send_debug_text_that_is_entirely_newlines_is_the_empty_case(tmp_path):
    """A text consisting ONLY of '\\n'/'\\r' bytes strips down to nothing
    -- must collapse onto the same bare "debug\\n" shape as an empty or
    null text, not leave a dangling separator space ("debug \\n")."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSendDebug(handle, b"\n\r\n\r")
        assert _sink_lines(lib, handle) == ["debug"]
    finally:
        lib.phDestroy(handle)


def test_send_debug_exactly_240_bytes_is_not_truncated(tmp_path):
    """Boundary companion to the truncation test below: "debug " (6) +
    233 bytes of text + '\\n' (1) == 240 bytes exactly, the wire's own
    cap -- must be emitted in full, not truncated."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        text = "z" * 233
        line = f"debug {text}"
        assert len(line) + 1 == 240
        lib.phSendDebug(handle, text.encode("ascii"))
        assert _sink_lines(lib, handle) == [line]
    finally:
        lib.phDestroy(handle)


def test_send_debug_241_bytes_is_truncated_not_overflowed(tmp_path):
    """One byte over the boundary above: the 234th character must be
    dropped, not overflow the line -- truncated to fit, never rejected
    outright (sendDebug has no channel to report a rejection through)."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        text = "z" * 234
        lib.phSendDebug(handle, text.encode("ascii"))
        expected = "debug " + "z" * 233  # the 234th 'z' is dropped
        assert _sink_lines(lib, handle) == [expected]
    finally:
        lib.phDestroy(handle)


def test_debug_verb_inbound_is_dropped_silently_not_malformed(tmp_path):
    """A `debug ...` line arriving as if it were INBOUND -- e.g. an echo
    on a shared radio channel, or another robot's own debug output --
    must be dropped SILENTLY: it is lowercase-led, so it can never
    dispatch as a command (spec S2.1), and this is the structural fix
    the v5 DBG:-flood incident needed. This is the debug-SPECIFIC
    instance of that general rule (test_lowercase_verb_dropped_silently_
    not_malformed already covers the general case with a synthetic
    "dbg" verb; this one uses the LITERAL verb this ticket adds)."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "debug something happened\n")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 0

        lib.phSetNow(handle, 321)
        _feed(lib, handle, "PING\n")
        assert _sink_lines(lib, handle) == ["pong 321"]
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# RUN: invocation by name (docs/design/protocol.md's RUN section). The
# handler only parses -- name + raw argument tokens -- and delegates
# resolution, conversion, invocation and stringification to the
# adapter (MockAdapter here).
# ---------------------------------------------------------------------------

def test_run_zero_args_calls_adapter_with_empty_argv(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_OK)
        _feed(lib, handle, "RUN blink\n")
        assert lib.phRunCalls(handle) == 1
        assert lib.phLastRunNameMatches(handle, b"blink")
        assert lib.phLastRunArgc(handle) == 0
        assert _sink_lines(lib, handle) == ["ok"]
    finally:
        lib.phDestroy(handle)


def test_run_passes_positional_args_in_order(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_OK)
        _feed(lib, handle, "RUN add 19 23 #9\n")
        assert lib.phRunCalls(handle) == 1
        assert lib.phLastRunNameMatches(handle, b"add")
        assert lib.phLastRunArgc(handle) == 2
        assert lib.phLastRunArgMatches(handle, 0, b"19")
        assert lib.phLastRunArgMatches(handle, 1, b"23")
        assert _sink_lines(lib, handle) == ["ok #9"]
    finally:
        lib.phDestroy(handle)


def test_run_omitted_id_void_return_acks_bare(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_OK)
        lib.phSetRunHasResult(handle, 0)
        _feed(lib, handle, "RUN blink\n")
        assert _sink_lines(lib, handle) == ["ok"]
    finally:
        lib.phDestroy(handle)


def test_run_omitted_id_with_return_value_is_bare_ret(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_OK)
        lib.phSetRunHasResult(handle, 1)
        lib.phSetRunResultText(handle, b"42")
        _feed(lib, handle, "RUN add 19 23\n")
        assert _sink_lines(lib, handle) == ["ret 42"]
    finally:
        lib.phDestroy(handle)


def test_run_nonzero_id_with_return_value_is_ret_with_id(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_OK)
        lib.phSetRunHasResult(handle, 1)
        lib.phSetRunResultText(handle, b"42")
        _feed(lib, handle, "RUN add 19 23 #9\n")
        assert _sink_lines(lib, handle) == ["ret 42 #9"]
    finally:
        lib.phDestroy(handle)


def test_run_zero_id_still_calls_adapter_but_suppresses_ret(tmp_path):
    """`#0` means "no ack wanted" -- the function still RUNS (the
    adapter is still called), but NOTHING is emitted, including a `ret`
    the function would otherwise have returned: a result is a reply,
    and #0 suppresses replies."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_OK)
        lib.phSetRunHasResult(handle, 1)
        lib.phSetRunResultText(handle, b"42")
        _feed(lib, handle, "RUN add 19 23 #0\n")
        assert lib.phRunCalls(handle) == 1, "the function must still run"
        assert _sink_lines(lib, handle) == [], "#0 suppresses even a ret"
    finally:
        lib.phDestroy(handle)


def test_run_zero_id_suppresses_err_too(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_UNKNOWN)
        _feed(lib, handle, "RUN no_such_fn #0\n")
        assert lib.phRunCalls(handle) == 1
        assert _sink_lines(lib, handle) == []
    finally:
        lib.phDestroy(handle)


def test_run_unknown_function_is_err_1(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_UNKNOWN)
        _feed(lib, handle, "RUN no_such_function #4\n")
        assert lib.phRunCalls(handle) == 1
        assert _sink_lines(lib, handle) == ["err #4 1"]
    finally:
        lib.phDestroy(handle)


def test_run_bad_arg_is_err_2(tmp_path):
    """Wrong arity, or an argument that fails to convert to its target's
    declared type, is the ADAPTER's own call (kBadArg) -- the handler
    holds no arity table for any registered function."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_BADARG)
        _feed(lib, handle, "RUN add one two #4\n")
        assert lib.phRunCalls(handle) == 1
        assert _sink_lines(lib, handle) == ["err #4 2"]
    finally:
        lib.phDestroy(handle)


def test_run_no_function_name_at_all_is_malformed(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "RUN\n")
        assert lib.phRunCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == []
    finally:
        lib.phDestroy(handle)


def test_run_only_an_id_token_is_no_function_name_but_recovers_the_id(tmp_path):
    """"RUN #7" -- the ONLY field present is consumed as the id, leaving
    nothing to be the function name. Still malformed, but (per the
    generic recovery rule) the well-formed nonzero id is recoverable for
    the err reply."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "RUN #7\n")
        assert lib.phRunCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == ["err #7 2"]
    finally:
        lib.phDestroy(handle)


def test_run_last_field_hash_non_digit_is_malformed_whole_line(tmp_path):
    """A last field beginning with '#' is ALWAYS the id slot under this
    grammar, even against RUN's own open arity -- so a function whose
    real LAST argument needs to literally start with '#' cannot be
    called that way; the whole line is malformed instead, the same as
    SET/WHEELS's own "3rd token present but not a well-formed id"
    rule."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "RUN foo #abc\n")
        assert lib.phRunCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == []
    finally:
        lib.phDestroy(handle)


def test_run_hash_prefixed_non_last_arg_is_an_ordinary_argument(tmp_path):
    """The '#'-reserved-for-id rule applies ONLY to the line's LAST
    field -- a '#'-led token anywhere else is just an ordinary
    argument."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_OK)
        _feed(lib, handle, "RUN foo #abc 5\n")
        assert lib.phRunCalls(handle) == 1
        assert lib.phLastRunArgc(handle) == 2
        assert lib.phLastRunArgMatches(handle, 0, b"#abc")
        assert lib.phLastRunArgMatches(handle, 1, b"5")
        assert _sink_lines(lib, handle) == ["ok"]
    finally:
        lib.phDestroy(handle)


def test_run_too_many_args_is_rejected_without_calling_adapter(tmp_path):
    """kMaxRunArgs is a firmware resource limit (the fixed argv[] array
    handleRun() builds), not a claim about any real function's arity --
    exceeding it is rejected the same way any other wrong arity is,
    BEFORE the adapter is ever called."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        args = " ".join(str(i) for i in range(17))  # kMaxRunArgs == 16
        _feed(lib, handle, f"RUN foo {args} #4\n")
        assert lib.phRunCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == ["err #4 2"]
    finally:
        lib.phDestroy(handle)


def test_run_at_kmaxrunargs_is_accepted(tmp_path):
    """The boundary companion to the test above: exactly kMaxRunArgs
    (16) arguments must be accepted and passed through in full."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_OK)
        args = " ".join(str(i) for i in range(16))
        _feed(lib, handle, f"RUN foo {args} #4\n")
        assert lib.phRunCalls(handle) == 1
        assert lib.phLastRunArgc(handle) == 16
        assert lib.phLastRunArgMatches(handle, 15, b"15")
        assert _sink_lines(lib, handle) == ["ok #4"]
    finally:
        lib.phDestroy(handle)


def test_run_result_text_is_sanitized_before_reaching_the_sink(tmp_path):
    """The ADAPTER's own returned text is untrusted content, exactly
    like sendDebug()'s text -- '\\n'/'\\r' bytes in it must never reach
    the sink, or a malicious/buggy registered function could forge a
    second line."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_OK)
        lib.phSetRunHasResult(handle, 1)
        lib.phSetRunResultText(handle, b"line1\nline2\r\n")
        _feed(lib, handle, "RUN foo #5\n")
        assert _sink_lines(lib, handle) == ["ret line1line2 #5"]
    finally:
        lib.phDestroy(handle)
