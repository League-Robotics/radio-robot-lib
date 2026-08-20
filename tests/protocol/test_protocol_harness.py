"""tests/protocol/test_protocol_harness.py -- the protocol handler host
test harness (docs/plan.md Step 3).

Protocol::ProtocolHandler (src/protocol/protocol_handler.{h,cpp}) is
exercised entirely through a MockAdapter + RecordingSink
(tests/protocol/mock_adapter.h, protocol_shim.cpp) -- no kernel, no
motors, no transport, matching docs/design/protocol.md Step 3's scope.

Two kinds of coverage:

1. test_golden_vectors drives every scenario in golden_vectors.txt
   (spec S11.3's cross-language conformance fixture) through the
   handler and asserts the sink's captured output byte-for-byte.

2. The individual test_* functions below cover what a tidy golden
   vector never exercises: feed()'s byte-block-boundary contract
   (docs/design/protocol.md S2.1), the 240-byte overflow-discard rule,
   the lowercase-verb-is-another-robot's-reply drop (the DBG: flood
   incident's structural fix), unknown verbs, wrong arity, and ESTOP's
   "no ack at all" rule (spec S8.2).

Run with::

    uv run python -m pytest tests/protocol/test_protocol_harness.py -v -s
"""

import ctypes
import pathlib
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
        else:
            raise ValueError(f"unknown SETUP key: {key!r}")

    def apply_action(self, kind, payload):
        lib, handle = self.lib, self.handle
        if kind == "IN":
            _feed(lib, handle, payload + "\n")
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
        assert _sink_lines(lib, handle) == ["pong:111"] * 3
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
        assert _sink_lines(lib, handle) == ["pong:222"]
        assert lib.phNowCalls(handle) == 1
    finally:
        lib.phDestroy(handle)


def test_feed_fragment_alone_never_dispatches(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "WHEELS:100:100:1000")  # no terminator, ever
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
        assert _sink_lines(lib, handle) == ["pong:333"]
        assert lib.phMalformedCount(handle) == 0
    finally:
        lib.phDestroy(handle)


def test_feed_overlong_line_discarded_not_truncated(tmp_path):
    """spec S2 / docs/design/protocol.md S2.1: a line over the 240-byte
    cap must be discarded to the next '\\n', never truncated into a
    prefix that still parses as something the host never sent. The
    line's first 22 bytes are a perfectly valid, correct-arity WHEELS
    command -- if the implementation truncated instead of discarding,
    this would dispatch as WHEELS:100:100:1000:5. It must not."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        overlong = "WHEELS:100:100:1000:5" + ("X" * 300) + "\n"
        assert len(overlong) > 240
        _feed(lib, handle, overlong)
        assert lib.phWheelsCalls(handle) == 0, (
            "the valid-looking WHEELS prefix must NOT have dispatched")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 1

        # The parser must resync cleanly on the next line.
        lib.phSetNow(handle, 444)
        _feed(lib, handle, "PING\n")
        assert _sink_lines(lib, handle) == ["pong:444"]
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
        name = "n" * 235  # "GET:" (4) + 235 == 239 content bytes;
                           # + '\n' == 240 total, exactly the cap.
        line = f"GET:{name}"
        assert len(line) + 1 == 240
        lib.phSetGetOverride(handle, name.encode("ascii"), 1.5)
        _feed(lib, handle, line + "\n")
        assert lib.phMalformedCount(handle) == 0
        assert _sink_lines(lib, handle) == [f"get:{name}:1.500000"]
    finally:
        lib.phDestroy(handle)


def test_feed_241_bytes_overflows(tmp_path):
    """One byte over test_feed_exactly_240_bytes_is_accepted's boundary
    must overflow -- proving the cap is exactly 240, not off by one in
    either direction."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        name = "n" * 236  # "GET:" (4) + 236 == 240 content bytes;
                           # + '\n' == 241 total, one over the cap.
        line = f"GET:{name}"
        assert len(line) + 1 == 241
        lib.phSetGetOverride(handle, name.encode("ascii"), 1.5)
        _feed(lib, handle, line + "\n")
        assert lib.phMalformedCount(handle) == 1
        assert lib.phGetCalls(handle) == 0, "must never have reached the adapter"
        assert _sink_lines(lib, handle) == []
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
        # Exactly the incident's own repro: a robot's own dbg: output
        # arriving as if it were inbound.
        _feed(lib, handle, "dbg:something happened\n")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 0

        # A robot's own ack, echoed back on the same shared channel.
        _feed(lib, handle, "ok:5\n")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 0

        # Parser must still work normally right after.
        lib.phSetNow(handle, 555)
        _feed(lib, handle, "PING\n")
        assert _sink_lines(lib, handle) == ["pong:555"]
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
# Unknown verb / wrong arity
# ---------------------------------------------------------------------------

def test_unknown_verb_no_reply(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "FOO\n")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 1

        _feed(lib, handle, "FOO:1:2:3\n")  # even WITH fields, still no id
                                            # is knowable for a verb the
                                            # dispatcher has never heard of
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 2
    finally:
        lib.phDestroy(handle)


def test_wrong_arity_rejected_no_best_effort_parse(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "PING:extra\n")  # PING takes 0 fields
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 1

        _feed(lib, handle, "WHEELS:100:100\n")  # needs 3 or 4 fields
        assert _sink_lines(lib, handle) == []
        assert lib.phWheelsCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 2

        _feed(lib, handle, "SET:name\n")  # needs 2 or 3 fields
        assert _sink_lines(lib, handle) == []
        assert lib.phSetCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 3
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# ESTOP: no ack at all (spec S8.2)
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
        _feed(lib, handle, "ESTOP:1\n")
        assert lib.phSinkLength(handle) == 0
        assert lib.phEstopCalls(handle) == 0, "must never have reached the adapter"
        assert lib.phMalformedCount(handle) == 1
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# Optional trailing id resolution (SET/WHEELS) -- this file's own
# reconciliation of spec S7.1's worked example against S8.2's literal
# words; see protocol_handler.h's ambiguity note #1.
# ---------------------------------------------------------------------------

def test_wheels_omitted_id_defaults_to_zero_and_is_acked(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS:50:50:200\n")
        assert lib.phWheelsCalls(handle) == 1
        assert lib.phLastWheelsId(handle) == 0
        assert _sink_lines(lib, handle) == ["ok:0"]
    finally:
        lib.phDestroy(handle)


def test_wheels_explicit_zero_id_suppresses_the_ack(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS:50:50:200:0\n")
        assert lib.phWheelsCalls(handle) == 1, "the adapter is still called"
        assert lib.phLastWheelsId(handle) == 0
        assert _sink_lines(lib, handle) == [], "id 0 means no ack wanted"
    finally:
        lib.phDestroy(handle)


def test_wheels_nonzero_id_is_acked_with_that_id(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS:50:50:200:42\n")
        assert lib.phLastWheelsId(handle) == 42
        assert _sink_lines(lib, handle) == ["ok:42"]
    finally:
        lib.phDestroy(handle)


def test_wheels_decodes_signed_speeds_and_passes_them_as_floats(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS:-100:200:500:9\n")
        assert lib.phWheelsCalls(handle) == 1
        assert lib.phLastWheelsLeft(handle) == pytest.approx(-100.0)
        assert lib.phLastWheelsRight(handle) == pytest.approx(200.0)
        assert lib.phLastWheelsDuration(handle) == 500
        assert _sink_lines(lib, handle) == ["ok:9"]
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# SET: value decode is the handler's job (spec S7.2), not the adapter's
# ---------------------------------------------------------------------------

def test_set_malformed_value_never_reaches_the_adapter(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "SET:group.alpha:notanumber\n")
        assert lib.phSetCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == ["err:0:2"]
    finally:
        lib.phDestroy(handle)


@pytest.mark.parametrize("value", ["nan", "inf", "-inf", "1e10"])
def test_set_rejects_nan_inf_and_exponents(tmp_path, value):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, f"SET:group.alpha:{value}\n")
        assert lib.phSetCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == ["err:0:2"]
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
        _feed(lib, handle, "SET:no.such.field:1.0\n")
        assert lib.phSetCalls(handle) == 1
        assert lib.phLastSetNameMatches(handle, b"no.such.field")
        assert lib.phMalformedCount(handle) == 0, (
            "a well-formed line the adapter rejects is not a malformed line")
        assert _sink_lines(lib, handle) == ["err:0:1"]
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# GET: unknown name has no wire outcome at all (ambiguity note #2)
# ---------------------------------------------------------------------------

def test_get_unknown_name_is_silent_and_not_malformed(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "GET:no.such.field\n")
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
            "get:group.alpha:1.500000",
            "get:group.beta:-2.250000",
            "get:group.gamma:0.000000",
            "get:group.delta:100.000000",
        ]
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# TLM: no wire outcome either (spec S3.1 -- no id field at all)
# ---------------------------------------------------------------------------

def test_tlm_valid_mode_calls_adapter_with_no_reply(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "TLM:FULL\n")
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
        _feed(lib, handle, "TLM:WARP\n")
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
        pose_cols = ["seq", "now", "x"]
        runner.apply_action("EMIT", ["seq:0:1", "now:0:1000", "x:0:5"])
        runner.apply_action("EMIT", ["seq:0:2", "now:0:1001", "x:0:6"])
        assert _sink_lines(lib, handle) == [
            "thdr:seq:now:x",
            "t:1:1000:5",
            "t:2:1001:6",
        ]

        lib.phSinkClear(handle)
        # A DIFFERENT column set (mode change, e.g. POSE -> FULL) must
        # emit a fresh thdr: before its own t:.
        runner.apply_action(
            "EMIT", ["seq:0:3", "now:0:1002", "x:0:7", "y:0:8"])
        assert _sink_lines(lib, handle) == ["thdr:seq:now:x:y", "t:3:1002:7:8"]
    finally:
        lib.phDestroy(handle)
