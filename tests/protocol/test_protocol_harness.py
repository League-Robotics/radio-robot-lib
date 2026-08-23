"""tests/protocol/test_protocol_harness.py -- the protocol handler host
test harness.

Protocol::ProtocolHandler (src/protocol/protocol_handler.{h,cpp}) is
exercised entirely through a MockAdapter + RecordingSink
(tests/protocol/mock_adapter.h, protocol_shim.cpp) -- no kernel, no
motors, no transport, matching docs/design/protocol.md's own scope.

Rewritten 2026-08-22 for the six stakeholder-directed changes
(docs/design/protocol.md §8.9):

1. **A decode failure (unknown verb, wrong arity, unparseable field) now
   NACKs and does NOT advance the sequence.** Only a MERITS rejection
   (the line decoded fine but the adapter refused its content) still
   acks-then-errs. Before this change, every in-order id advanced the
   sequence and acked, with the two kinds of rejection distinguished
   only by error code.
2. **`Result::kDuplicateId` (`ERR_DUPLICATE_ID`, code 11) is deleted**
   from `Protocol::Result` entirely -- it was already unreachable.
3. **`PING` joins `ESTOP`/`HELLO` as unsequenced** -- no mandatory id,
   always answers `pong`, even while the stream is stalled on a gap.
4. **`Adapter::lastDone()`/`lastDoneReason()`** replace the handler's own
   dead `lastDone_` field -- the handler polls the Adapter fresh on
   every `ack`/`nack`, and MockAdapter's canned `lastDoneToReturn`/
   `lastDoneReasonToReturn` let a test drive that piggyback directly.
5. **Every `ack`/`nack` now carries a THIRD token, the completion
   reason** (`none`/`stop`/`timeout`/`estop`/`aborted`) --
   `ack <id> <lastDone> <reason>` / `nack <id> <lastDone> <reason>`.
6. **`WHEELS` is renamed `WHEELS_V`**, joined by five new motion verbs
   (`WHEELS_X`/`MOVE_X`/`MOVE_V`/`GO_TO_R`/`GO_TO_W`,
   docs/design/motion-api.md §9.1) and `STOP`'s own optional `now`
   token (`STOP [now] #<id>`).

See protocol_handler.h's own file header for the full state machine.
Motion-verb dispatch through a REAL step()-driven adapter (as opposed to
MockAdapter's instantly-answering canned Results) is covered separately
in test_motion_reliability.py, including the flagship "dropped command
in a square tour" scenario the stakeholder's own directive describes.

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
# between the two. kDuplicateId is GONE (2026-08-22) -- there is no
# ordinal 8 any more.
RESULT_OK = 0
RESULT_UNKNOWN = 1
RESULT_BADARG = 2
RESULT_RANGE = 3
RESULT_FULL = 4
RESULT_UNIMPLEMENTED = 5
RESULT_NOTREADY = 6
RESULT_BUSY = 7

# Protocol::TlmMode's declaration order (adapter.h). kHdr (2026-08-23,
# docs/design/protocol.md §10.5) is never passed to onTlm() -- see the
# TLM HDR tests below -- but is listed here for symmetry with the C++
# enum.
TLM_OFF = 0
TLM_POSE = 1
TLM_FULL = 2
TLM_NOW = 3
TLM_AUTO = 4
TLM_BUFFER = 5
TLM_HDR = 6

# Protocol::DoneReason's declaration order (adapter.h, 2026-08-22).
DONE_NONE = 0
DONE_STOP = 1
DONE_TIMEOUT = 2
DONE_ESTOP = 3
DONE_ABORTED = 4

_DONE_REASON_NAME = {
    DONE_NONE: "none",
    DONE_STOP: "stop",
    DONE_TIMEOUT: "timeout",
    DONE_ESTOP: "estop",
    DONE_ABORTED: "aborted",
}


def _ack(n, last_done=0, reason=DONE_NONE):
    return f"ack {n} {last_done} {_DONE_REASON_NAME[reason]}"


def _nack(n, last_done=0, reason=DONE_NONE):
    return f"nack {n} {last_done} {_DONE_REASON_NAME[reason]}"


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
    lib.phSetWheelsXResult.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.phSetWheelsXResult.restype = None
    lib.phSetMoveXResult.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.phSetMoveXResult.restype = None
    lib.phSetMoveVResult.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.phSetMoveVResult.restype = None
    lib.phSetGoToRResult.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.phSetGoToRResult.restype = None
    lib.phSetGoToWResult.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.phSetGoToWResult.restype = None
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

    lib.phSetLastDone.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.phSetLastDone.restype = None
    lib.phSetLastDoneReason.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.phSetLastDoneReason.restype = None

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

    lib.phWheelsXCalls.argtypes = [ctypes.c_void_p]
    lib.phWheelsXCalls.restype = ctypes.c_int
    lib.phLastWheelsXLeft.argtypes = [ctypes.c_void_p]
    lib.phLastWheelsXLeft.restype = ctypes.c_float
    lib.phLastWheelsXRight.argtypes = [ctypes.c_void_p]
    lib.phLastWheelsXRight.restype = ctypes.c_float
    lib.phLastWheelsXCruise.argtypes = [ctypes.c_void_p]
    lib.phLastWheelsXCruise.restype = ctypes.c_float
    lib.phLastWheelsXTimeout.argtypes = [ctypes.c_void_p]
    lib.phLastWheelsXTimeout.restype = ctypes.c_uint32

    lib.phMoveXCalls.argtypes = [ctypes.c_void_p]
    lib.phMoveXCalls.restype = ctypes.c_int
    lib.phLastMoveXDistance.argtypes = [ctypes.c_void_p]
    lib.phLastMoveXDistance.restype = ctypes.c_float
    lib.phLastMoveXRotation.argtypes = [ctypes.c_void_p]
    lib.phLastMoveXRotation.restype = ctypes.c_float
    lib.phLastMoveXCruise.argtypes = [ctypes.c_void_p]
    lib.phLastMoveXCruise.restype = ctypes.c_float
    lib.phLastMoveXTimeout.argtypes = [ctypes.c_void_p]
    lib.phLastMoveXTimeout.restype = ctypes.c_uint32

    lib.phMoveVCalls.argtypes = [ctypes.c_void_p]
    lib.phMoveVCalls.restype = ctypes.c_int
    lib.phLastMoveVVx.argtypes = [ctypes.c_void_p]
    lib.phLastMoveVVx.restype = ctypes.c_float
    lib.phLastMoveVOmega.argtypes = [ctypes.c_void_p]
    lib.phLastMoveVOmega.restype = ctypes.c_float
    lib.phLastMoveVDuration.argtypes = [ctypes.c_void_p]
    lib.phLastMoveVDuration.restype = ctypes.c_uint32

    lib.phGoToRCalls.argtypes = [ctypes.c_void_p]
    lib.phGoToRCalls.restype = ctypes.c_int
    lib.phLastGoToRX.argtypes = [ctypes.c_void_p]
    lib.phLastGoToRX.restype = ctypes.c_float
    lib.phLastGoToRY.argtypes = [ctypes.c_void_p]
    lib.phLastGoToRY.restype = ctypes.c_float
    lib.phLastGoToRSpeed.argtypes = [ctypes.c_void_p]
    lib.phLastGoToRSpeed.restype = ctypes.c_float
    lib.phLastGoToRArrive.argtypes = [ctypes.c_void_p]
    lib.phLastGoToRArrive.restype = ctypes.c_float
    lib.phLastGoToRTimeout.argtypes = [ctypes.c_void_p]
    lib.phLastGoToRTimeout.restype = ctypes.c_uint32

    lib.phGoToWCalls.argtypes = [ctypes.c_void_p]
    lib.phGoToWCalls.restype = ctypes.c_int
    lib.phLastGoToWX.argtypes = [ctypes.c_void_p]
    lib.phLastGoToWX.restype = ctypes.c_float
    lib.phLastGoToWY.argtypes = [ctypes.c_void_p]
    lib.phLastGoToWY.restype = ctypes.c_float
    lib.phLastGoToWSpeed.argtypes = [ctypes.c_void_p]
    lib.phLastGoToWSpeed.restype = ctypes.c_float
    lib.phLastGoToWArrive.argtypes = [ctypes.c_void_p]
    lib.phLastGoToWArrive.restype = ctypes.c_float
    lib.phLastGoToWTimeout.argtypes = [ctypes.c_void_p]
    lib.phLastGoToWTimeout.restype = ctypes.c_uint32

    lib.phStopCalls.argtypes = [ctypes.c_void_p]
    lib.phStopCalls.restype = ctypes.c_int
    lib.phLastStopId.argtypes = [ctypes.c_void_p]
    lib.phLastStopId.restype = ctypes.c_uint32
    lib.phLastStopImmediate.argtypes = [ctypes.c_void_p]
    lib.phLastStopImmediate.restype = ctypes.c_int

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
# Golden vectors -- see golden_vectors.txt's own header comment for the
# file format.
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
        elif raw_line == "IN":
            actions.append(("IN", ""))
        elif raw_line.startswith("EMIT "):
            actions.append(("EMIT", raw_line[len("EMIT "):].split(" ")))
        elif raw_line == "DEBUG" or raw_line.startswith("DEBUG "):
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
    lifetime -- MockAdapter stores BORROWED pointers, so anything passed
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
        elif key == "wheelsxresult":
            lib.phSetWheelsXResult(handle, int(tokens[0]))
        elif key == "movexresult":
            lib.phSetMoveXResult(handle, int(tokens[0]))
        elif key == "movevresult":
            lib.phSetMoveVResult(handle, int(tokens[0]))
        elif key == "gotorresult":
            lib.phSetGoToRResult(handle, int(tokens[0]))
        elif key == "gotowresult":
            lib.phSetGoToWResult(handle, int(tokens[0]))
        elif key == "stopresult":
            lib.phSetStopResult(handle, int(tokens[0]))
        elif key == "runresult":
            lib.phSetRunResult(handle, int(tokens[0]))
        elif key == "runhasresult":
            lib.phSetRunHasResult(handle, int(tokens[0]))
        elif key == "runresulttext":
            lib.phSetRunResultText(handle, self._keep(tokens[0]))
        elif key == "lastdone":
            lib.phSetLastDone(handle, int(tokens[0]))
        elif key == "lastdonereason":
            lib.phSetLastDoneReason(handle, int(tokens[0]))
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
    and asserts the sink's output matches byte-for-byte.

    Every block runs against a freshly HELLO-reset sequence: HELLO is
    unsequenced and resets expectedNext_ to 1, so feeding one before
    each block's own SETUP/actions lets every block in the fixture use
    "#1" (or a short run starting there) as its own id(s), independent
    of whichever earlier block ran before it. HELLO does NOT reset the
    Adapter's own lastDone()/lastDoneReason() any more (2026-08-22,
    docs/design/protocol.md §8.9) -- MockAdapter's own canned values
    persist exactly like any other SETUP field, which is why blocks that
    care about a non-default reason state it explicitly."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    runner = _GoldenVectorRunner(lib, handle)
    try:
        blocks = _parse_golden_vectors(_GOLDEN_VECTORS_PATH)
        assert blocks, "no golden vectors parsed -- fixture path or format broke"
        for index, (setup_calls, actions, expected_out) in enumerate(blocks):
            _feed(lib, handle, "HELLO\n")  # reset the sequence
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
# feed()'s byte-block-boundary contract (docs/design/protocol.md §2.1).
# Uses STATUS/ID rather than the old PING-as-filler-verb convention --
# PING is unsequenced now (2026-08-22), so it can no longer stand in for
# "a generic sequenced command" the way it used to; a dedicated PING
# section below covers PING's own new behavior.
# ---------------------------------------------------------------------------

def test_feed_several_complete_lines_in_one_block(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetNow(handle, 111)
        _feed(lib, handle, "PING\nPING\nPING\n")
        assert lib.phNowCalls(handle) == 3
        assert _sink_lines(lib, handle) == ["pong 111", "pong 111", "pong 111"]
        assert lib.phMalformedCount(handle) == 0
    finally:
        lib.phDestroy(handle)


def test_feed_several_sequenced_lines_in_one_block(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS_V 1 1 100 #1\nWHEELS_V 2 2 100 #2\n"
                            "WHEELS_V 3 3 100 #3\n")
        assert lib.phWheelsCalls(handle) == 3
        assert _sink_lines(lib, handle) == [_ack(1), _ack(2), _ack(3)]
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
        _feed(lib, handle, "WHEELS_V 100 100 1000")  # no terminator, ever
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
    line's first bytes are a perfectly valid, correct-arity WHEELS_V
    command -- if the implementation truncated instead of discarding, a
    naive truncation could dispatch as WHEELS_V 100 100 1000 #5. It must
    not."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        overlong = "WHEELS_V 100 100 1000 #5" + ("X" * 300) + "\n"
        assert len(overlong) > 240
        _feed(lib, handle, overlong)
        assert lib.phWheelsCalls(handle) == 0, (
            "the valid-looking WHEELS_V prefix must NOT have dispatched")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 1

        # The parser must resync cleanly on the next line -- the discard
        # above never advanced the sequence, so id #1 is still in order.
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS_V 1 1 100 #1\n")
        assert _sink_lines(lib, handle) == [_ack(1)]
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
        name = "n" * 232  # "GET " (4) + 232 + " #1" (3) == 239 content
                           # bytes; + '\n' == 240 total, exactly the cap.
        line = f"GET {name} #1"
        assert len(line) + 1 == 240
        lib.phSetGetOverride(handle, name.encode("ascii"), 1.5)
        _feed(lib, handle, line + "\n")
        assert lib.phMalformedCount(handle) == 0
        assert _sink_lines(lib, handle) == [_ack(1), f"get {name} 1.500000"]
    finally:
        lib.phDestroy(handle)


def test_feed_241_bytes_overflows(tmp_path):
    """One byte over test_feed_exactly_240_bytes_is_accepted's boundary
    must overflow -- proving the cap is exactly 240, not off by one in
    either direction."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        name = "n" * 233  # one longer than the exactly-240 case above
        line = f"GET {name} #1"
        assert len(line) + 1 == 241
        lib.phSetGetOverride(handle, name.encode("ascii"), 1.5)
        _feed(lib, handle, line + "\n")
        assert lib.phMalformedCount(handle) == 1
        assert lib.phGetCalls(handle) == 0, "must never have reached the adapter"
        assert _sink_lines(lib, handle) == []
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# Blank / all-whitespace lines are ignored SILENTLY (spec S2).
# ---------------------------------------------------------------------------

def test_blank_line_is_silently_ignored_not_malformed(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "\n")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 0

        _feed(lib, handle, "\n\n\n")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 0

        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS_V 1 1 100 #1\n")
        assert _sink_lines(lib, handle) == [_ack(1)]
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
        _feed(lib, handle, "WHEELS_V   100   100   1000   #1\n")
        assert lib.phWheelsCalls(handle) == 1
        assert lib.phLastWheelsLeft(handle) == pytest.approx(100.0)
        assert lib.phLastWheelsRight(handle) == pytest.approx(100.0)
        assert lib.phLastWheelsDuration(handle) == 1000
        assert lib.phLastWheelsId(handle) == 1
        assert _sink_lines(lib, handle) == [_ack(1)]
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
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "dbg something happened\n")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 0

        _feed(lib, handle, "ack 5 0 none\n")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 0

        lib.phSetNow(handle, 555)
        _feed(lib, handle, "PING\n")
        assert _sink_lines(lib, handle) == ["pong 555"]
    finally:
        lib.phDestroy(handle)


def test_mixed_case_verb_is_unknown_not_dropped(tmp_path):
    """A verb starting UPPERCASE but not matching any known command
    (e.g. a typo) is a genuinely unknown command, not a foreign reply.
    With no id at all present, it cannot be sequence-classified either
    way -- malformed, no reply."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "Ping\n")  # starts uppercase, not the literal PING
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 1
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# PING (2026-08-22): unsequenced, no id, maximally forgiving -- joins
# ESTOP/HELLO's own exemption set (docs/design/protocol.md §8.3,
# protocol_handler.h ambiguity note #4).
# ---------------------------------------------------------------------------

def test_ping_bare_no_id_replies_pong(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetNow(handle, 38472)
        _feed(lib, handle, "PING\n")
        assert _sink_lines(lib, handle) == ["pong 38472"]
        assert lib.phMalformedCount(handle) == 0
    finally:
        lib.phDestroy(handle)


def test_ping_tolerates_trailing_content_including_an_old_style_id(tmp_path):
    """Maximally forgiving, like ESTOP: an old-style host still
    appending "#<id>" to PING out of habit from before this change keeps
    working -- the id-shaped token is never treated as one."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetNow(handle, 111)
        _feed(lib, handle, "PING #7\n")
        assert _sink_lines(lib, handle) == ["pong 111"]
        assert lib.phMalformedCount(handle) == 0
        lib.phSinkClear(handle)

        _feed(lib, handle, "PING extra junk here\n")
        assert _sink_lines(lib, handle) == ["pong 111"]
        assert lib.phMalformedCount(handle) == 0
    finally:
        lib.phDestroy(handle)


def test_ping_never_advances_or_is_gated_by_the_sequence(tmp_path):
    """PING is OUTSIDE the sequence entirely -- it neither consumes an
    id nor waits behind a gap. This is the property the stakeholder's
    own direction is about: "it is liveness and must answer while the
    stream is stalled in a gap.\""""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        # Create a gap: #5 arrives when #1 is expected.
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS_V 100 100 1000 #5\n")
        assert lib.phWheelsCalls(handle) == 0, "gap: must not have executed"
        lib.phSinkClear(handle)

        lib.phSetNow(handle, 999)
        _feed(lib, handle, "PING\n")
        assert _sink_lines(lib, handle) == ["pong 999"]
        lib.phSinkClear(handle)

        # PING did not consume or disturb the outstanding gap -- the
        # SAME missing id (#1) is still exactly what the sequence
        # expects, so supplying it now resolves the gap normally (proof
        # that PING left expectedNext_ untouched at 1, not advanced to
        # some other value by having been processed in between).
        _feed(lib, handle, "WHEELS_V 1 1 100 #1\n")
        assert lib.phWheelsCalls(handle) == 1
        assert _sink_lines(lib, handle) == [_ack(1)]
    finally:
        lib.phDestroy(handle)


def test_ping_multiple_in_one_feed_each_reply_independently(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetNow(handle, 42)
        _feed(lib, handle, "PING\nPING\nPING\n")
        assert lib.phNowCalls(handle) == 3
        assert _sink_lines(lib, handle) == ["pong 42"] * 3
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# Decode failure is a NAK (2026-08-22, docs/design/protocol.md §8.9) --
# the central behavior change: an unrecognized verb or wrong arity, IN
# ORDER, now NACKS and does NOT advance the sequence, instead of the
# pre-2026-08-22 "ack then err" shape.
# ---------------------------------------------------------------------------

def test_unknown_verb_no_reply_when_id_is_missing_or_unparseable(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "FOO\n")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 1

        _feed(lib, handle, "FOO 1 2 3\n")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 2
    finally:
        lib.phDestroy(handle)


def test_unknown_verb_in_order_nacks_and_does_not_advance(tmp_path):
    """An unknown verb whose id IS in order is a DECODE FAILURE
    (2026-08-22): it NACKs -- naming the SAME id, since it was never
    accepted -- plus err 1 (ERR_UNKNOWN). The sequence does not advance,
    so a SECOND unknown-verb line with the SAME id nacks identically
    (proving the state truly did not move), and only a well-formed line
    carrying that same id finally advances it."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "FOO 1 2 3 #1\n")
        assert _sink_lines(lib, handle) == [_nack(1), "err 1 #1"]
        assert lib.phMalformedCount(handle) == 1
        lib.phSinkClear(handle)

        # Still #1 -- the sequence never moved.
        _feed(lib, handle, "BAR #1\n")
        assert _sink_lines(lib, handle) == [_nack(1), "err 1 #1"]
        assert lib.phMalformedCount(handle) == 2
        lib.phSinkClear(handle)

        # A well-formed line carrying the SAME id finally advances it.
        _feed(lib, handle, "STATUS #1\n")
        text = _sink_lines(lib, handle)
        assert text[0] == _ack(1)
        assert text[1].endswith(" next=2"), text[1]
    finally:
        lib.phDestroy(handle)


def test_unknown_verb_id_zero_is_a_stale_retransmit_not_a_decode_failure(tmp_path):
    """`#0` is well-formed but always stale (ids start at 1, so 0 is
    always `< expectedNext_`) -- stale-retransmit handling runs BEFORE
    the verb is even looked up, so an "unknown verb" with id 0 is
    treated as an ordinary retransmit, not a decode failure: it gets the
    ordinary retransmit ack, no err, and is not counted malformed."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "FOO #0\n")
        assert _sink_lines(lib, handle) == [_ack(0)]
        assert lib.phMalformedCount(handle) == 0, (
            "an out-of-order/retransmit line's content is never "
            "inspected, so it is never counted malformed")
    finally:
        lib.phDestroy(handle)


def test_wrong_arity_known_verb_in_order_nacks_and_does_not_advance(tmp_path):
    """ID takes exactly one field -- its own mandatory id -- so an EXTRA
    field beyond that is a decode failure: it NACKs (holding the SAME
    id) plus err 2 (ERR_BADARG), rather than acking and moving on."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "ID extra #1\n")
        assert _sink_lines(lib, handle) == [_nack(1), "err 2 #1"]
        assert lib.phMalformedCount(handle) == 1
    finally:
        lib.phDestroy(handle)


def test_wrong_arity_rejected_no_best_effort_parse(tmp_path):
    """Wrong arity is a rejection, not a best-effort parse (spec S2.2) --
    and it is now a NACK (2026-08-22), so each attempt below leaves the
    sequence exactly where it started; the id only advances once a
    well-formed line finally supplies it."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "ID extra #1\n")  # ID takes 0 data fields
        assert _sink_lines(lib, handle) == [_nack(1), "err 2 #1"]
        assert lib.phMalformedCount(handle) == 1
        lib.phSinkClear(handle)

        _feed(lib, handle, "WHEELS_V 100 100 #1\n")  # needs 3 data fields;
                                                       # STILL #1
        assert _sink_lines(lib, handle) == [_nack(1), "err 2 #1"]
        assert lib.phWheelsCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 2
        lib.phSinkClear(handle)

        _feed(lib, handle, "SET name #1\n")  # needs 2 data fields; STILL #1
        assert _sink_lines(lib, handle) == [_nack(1), "err 2 #1"]
        assert lib.phSetCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 3
        lib.phSinkClear(handle)

        # Finally, a well-formed #1 -- the sequence advances for the
        # first time in this whole test.
        lib.phSetSetResult(handle, RESULT_OK)
        _feed(lib, handle, "SET name 1.0 #1\n")
        assert _sink_lines(lib, handle) == [_ack(1)]
        assert lib.phSetCalls(handle) == 1
    finally:
        lib.phDestroy(handle)


def test_decode_failure_out_of_order_line_is_never_examined_at_all(tmp_path):
    """A NUMERIC gap (id > expectedNext_) is a completely different case
    from a decode failure on an IN-ORDER id -- the verb is never even
    looked up, so an out-of-order line is never counted malformed, decode
    failure or not."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "TOTALLY_BOGUS_VERB #99\n")
        assert _sink_lines(lib, handle) == [_nack(1)]
        assert lib.phMalformedCount(handle) == 0, (
            "an out-of-order line's content is never inspected")
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# ESTOP: outside the sequence entirely, maximally forgiving, replies
# `estop` (unaffected by any of the 2026-08-22 changes).
# ---------------------------------------------------------------------------

def test_estop_executes_and_replies_estop(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "ESTOP\n")
        assert _sink_lines(lib, handle) == ["estop"]
        assert lib.phEstopCalls(handle) == 1
        assert lib.phMalformedCount(handle) == 0
    finally:
        lib.phDestroy(handle)


def test_estop_wrong_arity_still_executes_and_replies(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "ESTOP 1\n")
        assert _sink_lines(lib, handle) == ["estop"]
        assert lib.phEstopCalls(handle) == 1
        assert lib.phMalformedCount(handle) == 0
    finally:
        lib.phDestroy(handle)


def test_estop_with_trailing_id_shaped_token_still_executes_and_replies(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "ESTOP #5\n")
        assert _sink_lines(lib, handle) == ["estop"]
        assert lib.phEstopCalls(handle) == 1
        assert lib.phMalformedCount(handle) == 0
    finally:
        lib.phDestroy(handle)


def test_estop_executes_even_while_the_stream_is_stalled_on_a_gap(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS_V 100 100 1000 #5\n")
        assert lib.phWheelsCalls(handle) == 0, "gap: must not have executed"
        lib.phSinkClear(handle)

        _feed(lib, handle, "ESTOP\n")
        assert _sink_lines(lib, handle) == ["estop"]
        assert lib.phEstopCalls(handle) == 1
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# HELLO: resets the sequence (docs/design/protocol.md §8.3). Does NOT
# touch the Adapter's own lastDone()/lastDoneReason() any more
# (2026-08-22) -- that state moved off the handler entirely.
# ---------------------------------------------------------------------------

def test_hello_resets_the_sequence(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS_V 100 100 1000 #1\n")
        assert lib.phWheelsCalls(handle) == 1
        lib.phSinkClear(handle)

        lib.phSetIdentity(handle, b"tovez", b"SN1", b"differential", b"p",
                           b"6.0.0")
        _feed(lib, handle, "HELLO\n")
        assert _sink_lines(lib, handle) == ["device NEZHA2 robot tovez SN1"]
        lib.phSinkClear(handle)

        _feed(lib, handle, "WHEELS_V 50 50 500 #1\n")
        assert lib.phWheelsCalls(handle) == 2
        assert lib.phLastWheelsId(handle) == 1
        assert _sink_lines(lib, handle) == [_ack(1)]
    finally:
        lib.phDestroy(handle)


def test_hello_wrong_arity_is_malformed_with_no_reply(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "HELLO extra\n")
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 1
    finally:
        lib.phDestroy(handle)


def test_hello_does_not_reset_the_adapters_lastdone(tmp_path):
    """2026-08-22: lastDone()/lastDoneReason() moved OFF the handler
    entirely (adapter.h) -- a HELLO reset has no business reaching into
    the Adapter to clear something it does not own. MockAdapter's own
    canned lastDoneToReturn/lastDoneReasonToReturn persist across a
    HELLO exactly like any other SETUP field would."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetLastDone(handle, 42)
        lib.phSetLastDoneReason(handle, DONE_STOP)
        _feed(lib, handle, "HELLO\n")
        lib.phSinkClear(handle)

        _feed(lib, handle, "STATUS #1\n")
        assert _sink_lines(lib, handle)[0] == _ack(1, 42, DONE_STOP)
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# The reliability layer's three sequence cases (docs/design/protocol.md
# S8.1), exercised directly: in order, stale retransmit, gap.
# ---------------------------------------------------------------------------

def test_pipelining_three_in_order_ids_in_one_feed(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle,
              "WHEELS_V 10 10 100 #1\nWHEELS_V 20 20 100 #2\n"
              "WHEELS_V 30 30 100 #3\n")
        assert lib.phWheelsCalls(handle) == 3
        assert _sink_lines(lib, handle) == [_ack(1), _ack(2), _ack(3)]

        lib.phSinkClear(handle)
        _feed(lib, handle, "WHEELS_V 40 40 100 #4\n")
        assert lib.phWheelsCalls(handle) == 4
        assert _sink_lines(lib, handle) == [_ack(4)]
    finally:
        lib.phDestroy(handle)


def test_cumulative_ack_proves_the_host_is_caught_up(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS_V 10 10 100 #1\n")
        _feed(lib, handle, "WHEELS_V 20 20 100 #2\n")
        _feed(lib, handle, "WHEELS_V 30 30 100 #3\n")  # its own ack ignored
        lib.phSinkClear(handle)

        _feed(lib, handle, "WHEELS_V 40 40 100 #4\n")
        _feed(lib, handle, "WHEELS_V 50 50 100 #5\n")
        assert _sink_lines(lib, handle) == [_ack(4), _ack(5)]
        assert lib.phWheelsCalls(handle) == 5
    finally:
        lib.phDestroy(handle)


def test_gap_stalls_the_stream_until_the_missing_id_arrives(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS_V 1 1 100 #1\n")
        _feed(lib, handle, "WHEELS_V 2 2 100 #2\n")
        _feed(lib, handle, "WHEELS_V 3 3 100 #3\n")
        assert _sink_lines(lib, handle) == [_ack(1), _ack(2), _ack(3)]
        assert lib.phWheelsCalls(handle) == 3
        lib.phSinkClear(handle)

        _feed(lib, handle, "WHEELS_V 5 5 100 #5\n")  # gap: 4 never arrived
        assert _sink_lines(lib, handle) == [_nack(4)]
        assert lib.phWheelsCalls(handle) == 3, "must NOT have executed"
        lib.phSinkClear(handle)

        _feed(lib, handle, "WHEELS_V 6 6 100 #6\n")
        assert _sink_lines(lib, handle) == [_nack(4)]
        assert lib.phWheelsCalls(handle) == 3
        lib.phSinkClear(handle)

        _feed(lib, handle, "WHEELS_V 7 7 100 #7\n")
        assert _sink_lines(lib, handle) == [_nack(4)]
        assert lib.phWheelsCalls(handle) == 3
        lib.phSinkClear(handle)

        _feed(lib, handle, "WHEELS_V 4 4 100 #4\n")
        assert _sink_lines(lib, handle) == [_ack(4)]
        assert lib.phWheelsCalls(handle) == 4
        assert lib.phLastWheelsLeft(handle) == pytest.approx(4.0), (
            "the #4 that just arrived must be what executed, not a "
            "buffered 5/6/7")

        lib.phSinkClear(handle)
        _feed(lib, handle, "WHEELS_V 5 5 100 #5\n")
        assert lib.phWheelsCalls(handle) == 5
        assert lib.phLastWheelsLeft(handle) == pytest.approx(5.0)
        assert _sink_lines(lib, handle) == [_ack(5)]
    finally:
        lib.phDestroy(handle)


def test_duplicate_does_not_re_execute(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS_V 1 1 100 #1\n")
        _feed(lib, handle, "WHEELS_V 2 2 100 #2\n")
        assert lib.phWheelsCalls(handle) == 2
        lib.phSinkClear(handle)

        _feed(lib, handle, "WHEELS_V 2 2 100 #2\n")
        assert lib.phWheelsCalls(handle) == 2, "must NOT have re-executed"
        assert _sink_lines(lib, handle) == [_ack(2)], (
            "retransmit ack echoes the highest ALREADY-accepted id"
        )
    finally:
        lib.phDestroy(handle)


def test_in_order_but_rejected_on_merits_gets_both_ack_and_err(tmp_path):
    """A MERITS rejection (the line decoded fine; the adapter refused
    its CONTENT) is NOT a decode failure -- it still gets both lines:
    the ack (it arrived, decoded, the sequence advances) and
    err <code> #<id>. This is the OTHER half of the 2026-08-22
    distinction (docs/design/protocol.md §8.9) -- kept distinct from a
    decode failure on purpose."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_RANGE)
        _feed(lib, handle, "WHEELS_V 99999 0 100 #1\n")
        assert lib.phWheelsCalls(handle) == 1, "the adapter WAS called"
        assert _sink_lines(lib, handle) == [_ack(1), "err 3 #1"]
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# Periodic emission: emitTelemetry() also emits the current reliability
# line (docs/design/protocol.md §8.5), now including the reason token,
# polling the Adapter fresh every call (2026-08-22).
# ---------------------------------------------------------------------------

def test_emit_telemetry_appends_ack_when_caught_up(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    runner = _GoldenVectorRunner(lib, handle)
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS_V 1 1 100 #1\n")
        lib.phSinkClear(handle)

        runner.apply_action("EMIT", ["seq:0:1", "now:0:1000", "x:0:5"])
        assert _sink_lines(lib, handle) == [
            "thdr seq now x",
            "t 1 1000 5",
            _ack(1),
        ]
    finally:
        lib.phDestroy(handle)


def test_emit_telemetry_appends_nack_while_a_gap_is_outstanding(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    runner = _GoldenVectorRunner(lib, handle)
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS_V 5 5 100 #5\n")  # gap: expects #1
        lib.phSinkClear(handle)

        runner.apply_action("EMIT", ["seq:0:1", "now:0:1000", "x:0:5"])
        assert _sink_lines(lib, handle) == [
            "thdr seq now x",
            "t 1 1000 5",
            _nack(1),
        ]

        lib.phSinkClear(handle)
        runner.apply_action("EMIT", ["seq:0:2", "now:0:1001", "x:0:6"])
        assert _sink_lines(lib, handle) == ["t 2 1001 6", _nack(1)]

        _feed(lib, handle, "WHEELS_V 1 1 100 #1\n")
        lib.phSinkClear(handle)
        runner.apply_action("EMIT", ["seq:0:3", "now:0:1002", "x:0:7"])
        assert _sink_lines(lib, handle) == ["t 3 1002 7", _ack(1)]
    finally:
        lib.phDestroy(handle)


def test_emit_telemetry_reflects_the_adapters_current_lastdone(tmp_path):
    """2026-08-22: the piggyback polls the Adapter fresh on every call --
    changing MockAdapter's canned lastDone/lastDoneReason between two
    emitTelemetry() calls must be visible on the SECOND one immediately,
    with no caching anywhere in ProtocolHandler."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    runner = _GoldenVectorRunner(lib, handle)
    try:
        runner.apply_action("EMIT", ["seq:0:1", "now:0:1"])
        assert _sink_lines(lib, handle)[-1] == _ack(0, 0, DONE_NONE)
        lib.phSinkClear(handle)

        lib.phSetLastDone(handle, 9)
        lib.phSetLastDoneReason(handle, DONE_TIMEOUT)
        runner.apply_action("EMIT", ["seq:0:2", "now:0:2"])
        assert _sink_lines(lib, handle)[-1] == _ack(0, 9, DONE_TIMEOUT)
    finally:
        lib.phDestroy(handle)


def test_status_reports_next(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "STATUS #1\n")
        text = _sink_lines(lib, handle)
        assert text[0] == _ack(1)
        assert text[1].endswith(" next=2"), text[1]

        lib.phSinkClear(handle)
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS_V 1 1 100 #2\n")
        _feed(lib, handle, "WHEELS_V 1 1 100 #3\n")
        lib.phSinkClear(handle)
        _feed(lib, handle, "STATUS #4\n")
        text = _sink_lines(lib, handle)
        assert text[1].endswith(" next=5"), text[1]
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# WHEELS_V (the 2026-08-22 rename of WHEELS -- docs/design/motion-api.md
# §9.2 confirms WHEELS *is* wheels_v; same fields, same meaning).
# ---------------------------------------------------------------------------

def test_wheels_v_missing_id_is_malformed_no_reply(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS_V 50 50 200\n")
        assert lib.phWheelsCalls(handle) == 0, "id is mandatory"
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 1
    finally:
        lib.phDestroy(handle)


def test_wheels_v_id_zero_is_a_stale_retransmit_never_executes(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS_V 50 50 200 #0\n")
        assert lib.phWheelsCalls(handle) == 0, "must NOT have executed"
        assert _sink_lines(lib, handle) == [_ack(0)]
    finally:
        lib.phDestroy(handle)


def test_wheels_v_in_order_id_executes_and_is_acked(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS_V 50 50 200 #1\n")
        assert lib.phLastWheelsId(handle) == 1
        assert _sink_lines(lib, handle) == [_ack(1)]
    finally:
        lib.phDestroy(handle)


def test_wheels_v_decodes_signed_speeds_and_passes_them_as_floats(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS_V -100 200 500 #1\n")
        assert lib.phWheelsCalls(handle) == 1
        assert lib.phLastWheelsLeft(handle) == pytest.approx(-100.0)
        assert lib.phLastWheelsRight(handle) == pytest.approx(200.0)
        assert lib.phLastWheelsDuration(handle) == 500
        assert _sink_lines(lib, handle) == [_ack(1)]
    finally:
        lib.phDestroy(handle)


def test_wheels_v_id_missing_hash_prefix_is_malformed_no_reply(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "WHEELS_V 50 50 200 9\n")
        assert lib.phWheelsCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == []
    finally:
        lib.phDestroy(handle)


def test_id_rejects_leading_plus_sign(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS_V 50 50 200 #+5\n")
        assert lib.phWheelsCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == []
    finally:
        lib.phDestroy(handle)


def test_old_wheels_verb_name_is_now_unknown(tmp_path):
    """The bare "WHEELS" verb (pre-rename) no longer exists -- it is
    now an ordinary unknown verb, i.e. a decode failure: NACK plus err,
    the sequence held in place."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "WHEELS 100 100 1000 #1\n")
        assert lib.phWheelsCalls(handle) == 0
        assert _sink_lines(lib, handle) == [_nack(1), "err 1 #1"]
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# WHEELS_X, MOVE_X, MOVE_V, GO_TO_R, GO_TO_W (docs/design/motion-api.md
# §9.1) -- decode-level coverage through MockAdapter. Motion dispatched
# to a REAL step()-driven adapter is test_motion_reliability.py's job;
# this file only proves the handler decodes each verb's own fields
# correctly and in the right order (fixed-arity parsing, milliradian
# integers for rotation/omega) and routes it to the right Adapter method.
# ---------------------------------------------------------------------------

def test_wheels_x_decodes_all_four_fields_in_order(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsXResult(handle, RESULT_OK)
        _feed(lib, handle, "WHEELS_X -150 150 200 5000 #1\n")
        assert lib.phWheelsXCalls(handle) == 1
        assert lib.phLastWheelsXLeft(handle) == pytest.approx(-150.0)
        assert lib.phLastWheelsXRight(handle) == pytest.approx(150.0)
        assert lib.phLastWheelsXCruise(handle) == pytest.approx(200.0)
        assert lib.phLastWheelsXTimeout(handle) == 5000
        assert _sink_lines(lib, handle) == [_ack(1)]
    finally:
        lib.phDestroy(handle)


def test_wheels_x_wrong_arity_is_a_decode_failure(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "WHEELS_X 100 100 200 #1\n")  # missing timeout
        assert lib.phWheelsXCalls(handle) == 0
        assert _sink_lines(lib, handle) == [_nack(1), "err 2 #1"]
    finally:
        lib.phDestroy(handle)


def test_wheels_x_merits_rejection_still_acks_then_errs(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetWheelsXResult(handle, RESULT_RANGE)
        _feed(lib, handle, "WHEELS_X 100 100 200 5000 #1\n")
        assert lib.phWheelsXCalls(handle) == 1
        assert _sink_lines(lib, handle) == [_ack(1), "err 3 #1"]
    finally:
        lib.phDestroy(handle)


def test_move_x_decodes_milliradian_rotation_as_an_ordinary_signed_field(tmp_path):
    """Rotation is a milliradian INTEGER on the wire (docs/design/
    motion-api.md §9.1) -- the conversion to/from degrees is a language
    binding's job, not this handler's; the wire decode itself is just an
    ordinary signed int32 field, positive or negative."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetMoveXResult(handle, RESULT_OK)
        _feed(lib, handle, "MOVE_X 400 -1571 200 5000 #1\n")
        assert lib.phMoveXCalls(handle) == 1
        assert lib.phLastMoveXDistance(handle) == pytest.approx(400.0)
        assert lib.phLastMoveXRotation(handle) == pytest.approx(-1571.0)
        assert lib.phLastMoveXCruise(handle) == pytest.approx(200.0)
        assert lib.phLastMoveXTimeout(handle) == 5000
        assert _sink_lines(lib, handle) == [_ack(1)]
    finally:
        lib.phDestroy(handle)


def test_move_x_wrong_arity_is_a_decode_failure(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "MOVE_X 400 0 200 #1\n")  # missing timeout
        assert lib.phMoveXCalls(handle) == 0
        assert _sink_lines(lib, handle) == [_nack(1), "err 2 #1"]
    finally:
        lib.phDestroy(handle)


def test_move_v_decodes_v_x_and_omega_in_order(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetMoveVResult(handle, RESULT_OK)
        _feed(lib, handle, "MOVE_V 150 -500 1000 #1\n")
        assert lib.phMoveVCalls(handle) == 1
        assert lib.phLastMoveVVx(handle) == pytest.approx(150.0)
        assert lib.phLastMoveVOmega(handle) == pytest.approx(-500.0)
        assert lib.phLastMoveVDuration(handle) == 1000
        assert _sink_lines(lib, handle) == [_ack(1)]
    finally:
        lib.phDestroy(handle)


def test_move_v_missing_id_is_malformed(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "MOVE_V 150 0 1000\n")
        assert lib.phMoveVCalls(handle) == 0
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 1
    finally:
        lib.phDestroy(handle)


def test_go_to_r_decodes_all_five_fields_in_order(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetGoToRResult(handle, RESULT_OK)
        _feed(lib, handle, "GO_TO_R 300 -150 150 10 5000 #1\n")
        assert lib.phGoToRCalls(handle) == 1
        assert lib.phLastGoToRX(handle) == pytest.approx(300.0)
        assert lib.phLastGoToRY(handle) == pytest.approx(-150.0)
        assert lib.phLastGoToRSpeed(handle) == pytest.approx(150.0)
        assert lib.phLastGoToRArrive(handle) == pytest.approx(10.0)
        assert lib.phLastGoToRTimeout(handle) == 5000
        assert _sink_lines(lib, handle) == [_ack(1)]
    finally:
        lib.phDestroy(handle)


def test_go_to_w_decodes_all_five_fields_in_order(tmp_path):
    """Identical wire shape to GO_TO_R -- the handler distinguishes them
    purely by verb name, dispatching to a DIFFERENT Adapter method."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetGoToWResult(handle, RESULT_OK)
        _feed(lib, handle, "GO_TO_W 300 -150 150 10 5000 #1\n")
        assert lib.phGoToWCalls(handle) == 1
        assert lib.phGoToRCalls(handle) == 0, "must not have dispatched to R"
        assert lib.phLastGoToWX(handle) == pytest.approx(300.0)
        assert lib.phLastGoToWY(handle) == pytest.approx(-150.0)
        assert lib.phLastGoToWSpeed(handle) == pytest.approx(150.0)
        assert lib.phLastGoToWArrive(handle) == pytest.approx(10.0)
        assert lib.phLastGoToWTimeout(handle) == 5000
        assert _sink_lines(lib, handle) == [_ack(1)]
    finally:
        lib.phDestroy(handle)


def test_go_to_r_wrong_arity_is_a_decode_failure(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "GO_TO_R 300 -150 150 10 #1\n")  # missing timeout
        assert lib.phGoToRCalls(handle) == 0
        assert _sink_lines(lib, handle) == [_nack(1), "err 2 #1"]
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# STOP: `STOP [now] #<id>` (docs/design/motion-api.md §3.7/§9.1). `#0`
# is simply the ordinary stale-retransmit case, same as every other
# sequenced verb.
# ---------------------------------------------------------------------------

def test_stop_in_order_id_executes_and_is_acked(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetStopResult(handle, RESULT_OK)
        _feed(lib, handle, "STOP #1\n")
        assert lib.phStopCalls(handle) == 1
        assert lib.phLastStopId(handle) == 1
        assert lib.phLastStopImmediate(handle) == 0
        assert _sink_lines(lib, handle) == [_ack(1)]
    finally:
        lib.phDestroy(handle)


def test_stop_now_reaches_onstop_with_immediate_true(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetStopResult(handle, RESULT_OK)
        _feed(lib, handle, "STOP now #1\n")
        assert lib.phStopCalls(handle) == 1
        assert lib.phLastStopImmediate(handle) == 1
        assert _sink_lines(lib, handle) == [_ack(1)]
    finally:
        lib.phDestroy(handle)


def test_stop_token_other_than_now_is_a_decode_failure(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "STOP later #1\n")
        assert lib.phStopCalls(handle) == 0
        assert _sink_lines(lib, handle) == [_nack(1), "err 2 #1"]
    finally:
        lib.phDestroy(handle)


def test_stop_id_zero_is_a_stale_retransmit_not_malformed(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "STOP #0\n")
        assert lib.phStopCalls(handle) == 0, "must NOT have executed"
        assert lib.phMalformedCount(handle) == 0
        assert _sink_lines(lib, handle) == [_ack(0)]
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
# SET: value decode is the handler's job (spec S7.2), not the adapter's.
# A malformed VALUE is now a decode failure (NACK), not "ack then err"
# (2026-08-22) -- this is the sharpest illustration of the whole change:
# the value field fails to parse, so the line never "arrived intact".
# ---------------------------------------------------------------------------

def test_set_success_is_the_ack_alone(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetSetResult(handle, RESULT_OK)
        _feed(lib, handle, "SET group.alpha 1.0 #1\n")
        assert lib.phSetCalls(handle) == 1
        assert _sink_lines(lib, handle) == [_ack(1)]
    finally:
        lib.phDestroy(handle)


def test_set_malformed_value_nacks_and_does_not_advance(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "SET group.alpha notanumber #1\n")
        assert lib.phSetCalls(handle) == 0, "must NOT have reached onSet()"
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == [_nack(1), "err 2 #1"]
        lib.phSinkClear(handle)

        # The resend, well-formed, finally advances the sequence.
        lib.phSetSetResult(handle, RESULT_OK)
        _feed(lib, handle, "SET group.alpha 1.0 #1\n")
        assert lib.phSetCalls(handle) == 1
        assert _sink_lines(lib, handle) == [_ack(1)]
    finally:
        lib.phDestroy(handle)


@pytest.mark.parametrize("value", ["nan", "inf", "-inf", "1e10"])
def test_set_rejects_nan_inf_and_exponents(tmp_path, value):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, f"SET group.alpha {value} #1\n")
        assert lib.phSetCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == [_nack(1), "err 2 #1"]
    finally:
        lib.phDestroy(handle)


def test_set_unknown_name_is_the_adapters_call_a_merits_rejection(tmp_path):
    """docs/design/protocol.md §7: "which names are valid is entirely
    the adapter's business" -- the mock's onSet() always answers, so an
    unrecognized name is only "unknown" if the ADAPTER says kUnknown.
    This is a MERITS rejection (the line decoded fine), so it still
    acks then errs -- unaffected by the decode-failure-is-a-NAK change."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetSetResult(handle, RESULT_UNKNOWN)
        _feed(lib, handle, "SET no.such.field 1.0 #1\n")
        assert lib.phSetCalls(handle) == 1
        assert lib.phLastSetNameMatches(handle, b"no.such.field")
        assert lib.phMalformedCount(handle) == 0, (
            "a well-formed line the adapter rejects is not a malformed line")
        assert _sink_lines(lib, handle) == [_ack(1), "err 1 #1"]
    finally:
        lib.phDestroy(handle)


def test_set_id_without_hash_prefix_is_malformed(tmp_path):
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
# GET: unknown name is acked (it decoded fine and was answered with an
# empty result) -- not an error, and not a decode failure.
# ---------------------------------------------------------------------------

def test_get_missing_id_is_malformed_never_reaches_the_adapter(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "GET group.alpha\n")
        assert lib.phGetCalls(handle) == 0, "id is mandatory"
        assert _sink_lines(lib, handle) == []
        assert lib.phMalformedCount(handle) == 1
    finally:
        lib.phDestroy(handle)


def test_get_unknown_name_is_acked_but_produces_no_get_line(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "GET no.such.field #1\n")
        assert lib.phGetCalls(handle) == 1  # the adapter WAS asked
        assert _sink_lines(lib, handle) == [_ack(1)]
        assert lib.phMalformedCount(handle) == 0
    finally:
        lib.phDestroy(handle)


def test_bare_get_dumps_every_field(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "GET #1\n")
        assert _sink_lines(lib, handle) == [
            _ack(1),
            "get group.alpha 1.500000",
            "get group.beta -2.250000",
            "get group.gamma 0.000000",
            "get group.delta 100.000000",
        ]
    finally:
        lib.phDestroy(handle)


def test_get_wrong_arity_is_a_decode_failure(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "GET group.alpha extra #1\n")
        assert lib.phGetCalls(handle) == 0, "wrong arity -- never reached onGet()"
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == [_nack(1), "err 2 #1"]
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# TLM: the adapter's own Result still never surfaces on the wire
# (unchanged), but an unknown MODE token is now a decode failure.
# ---------------------------------------------------------------------------

def test_tlm_valid_mode_calls_adapter_and_is_acked(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "TLM FULL #1\n")
        assert lib.phTlmCalls(handle) == 1
        assert lib.phLastTlmMode(handle) == TLM_FULL
        assert _sink_lines(lib, handle) == [_ack(1)]
        assert lib.phMalformedCount(handle) == 0
    finally:
        lib.phDestroy(handle)


def test_tlm_unknown_mode_is_a_decode_failure(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "TLM WARP #1\n")
        assert lib.phTlmCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == [_nack(1), "err 2 #1"]
    finally:
        lib.phDestroy(handle)


def test_tlm_missing_id_is_malformed_never_reaches_adapter(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "TLM FULL\n")
        assert lib.phTlmCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == []
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# TLM HDR: requesting a fresh header (docs/design/protocol.md §10.5,
# sprint 003 ticket 003). Reuses the existing `TLM <mode> #id` slot, so
# it is sequenced and acked like any other TLM form -- but execTlm()
# special-cases it entirely inside the handler (clearing
# everEmittedHeader_ directly) and never forwards it to
# Adapter::onTlm() at all, since a header-recovery request is not a
# subscription change.
# ---------------------------------------------------------------------------

def test_tlm_hdr_is_accepted_and_acked_like_any_sequenced_tlm_form(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "TLM HDR #1\n")
        assert _sink_lines(lib, handle) == [_ack(1)]
        assert lib.phMalformedCount(handle) == 0
    finally:
        lib.phDestroy(handle)


def test_tlm_hdr_never_reaches_the_adapter_and_leaves_status_tlm_unchanged(
        tmp_path):
    """AC: 'the current subscription mode ... is unchanged after TLM
    HDR -- verified by checking mode state before and after.' Two
    independent checks: (1) STATUS's own tlm= field, read before and
    after the TLM HDR request; (2) the adapter's own onTlm() call
    record (tlmCalls/lastTlmMode) proving execTlm() actually took the
    "never call onTlm()" branch, not merely that nothing downstream
    happened to react to it."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        # `s.tlm` is a BORROWED pointer (protocol_shim.cpp's phSetStatus,
        # same contract as every other canned string field on
        # MockAdapter) -- keep the encoded bytes alive in a local so it
        # outlives every STATUS call below, not just this one.
        tlm_bytes = b"full"
        lib.phSetStatus(handle, 1, 1, 1, 1, 1, 0, 0, tlm_bytes)

        _feed(lib, handle, "TLM FULL #1\n")
        assert lib.phTlmCalls(handle) == 1
        assert lib.phLastTlmMode(handle) == TLM_FULL
        lib.phSinkClear(handle)

        _feed(lib, handle, "STATUS #2\n")
        before = _sink_lines(lib, handle)
        assert "tlm=full" in before[1], before
        lib.phSinkClear(handle)

        _feed(lib, handle, "TLM HDR #3\n")
        assert _sink_lines(lib, handle) == [_ack(3)]
        assert lib.phTlmCalls(handle) == 1, "TLM HDR must never call onTlm()"
        assert lib.phLastTlmMode(handle) == TLM_FULL, (
            "the current mode must be exactly as TLM FULL left it")
        lib.phSinkClear(handle)

        _feed(lib, handle, "STATUS #4\n")
        after = _sink_lines(lib, handle)
        assert "tlm=full" in after[1], after
    finally:
        lib.phDestroy(handle)


def test_tlm_hdr_forces_a_fresh_thdr_before_the_next_t_frame(tmp_path):
    """The core recovery scenario (docs/design/protocol.md §10.5): a
    host that already holds the header loses it (a dropped thdr line, or
    a mid-stream reconnect) and has no remembered header any more.
    Simulated here on the HANDLER side by clearing everEmittedHeader_
    through TLM HDR itself -- the very next emitTelemetry() call must
    re-emit thdr before its next t frame, even though the column set has
    not changed at all (contrast against the immediately preceding EMIT,
    same column set, which correctly does NOT repeat thdr)."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    runner = _GoldenVectorRunner(lib, handle)
    try:
        # Establish the header once -- the ordinary first-frame case.
        runner.apply_action("EMIT", ["seq:0:1", "now:0:1000", "x:0:5"])
        assert _sink_lines(lib, handle) == [
            "thdr seq now x", "t 1 1000 5", _ack(0),
        ]
        lib.phSinkClear(handle)

        # Same column set again -- thdr must NOT repeat (baseline §10.2
        # behavior, the contrast case for what follows).
        runner.apply_action("EMIT", ["seq:0:2", "now:0:1001", "x:0:6"])
        assert _sink_lines(lib, handle) == ["t 2 1001 6", _ack(0)]
        lib.phSinkClear(handle)

        # Simulate losing the header and ask for a fresh one.
        _feed(lib, handle, "TLM HDR #1\n")
        assert _sink_lines(lib, handle) == [_ack(1)]
        lib.phSinkClear(handle)

        # The VERY NEXT emitTelemetry() call re-emits thdr before t, even
        # though the column set is still identical to the last frame.
        runner.apply_action("EMIT", ["seq:0:3", "now:0:1002", "x:0:7"])
        assert _sink_lines(lib, handle) == [
            "thdr seq now x", "t 3 1002 7", _ack(1),
        ]
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
            _ack(0),
            "t 2 1001 6",
            _ack(0),
        ]

        lib.phSinkClear(handle)
        runner.apply_action(
            "EMIT", ["seq:0:3", "now:0:1002", "x:0:7", "y:0:8"])
        assert _sink_lines(lib, handle) == [
            "thdr seq now x y", "t 3 1002 7 8", _ack(0),
        ]
    finally:
        lib.phDestroy(handle)


# ---------------------------------------------------------------------------
# Chunk-split equivalence (docs/design/protocol.md S2.1) -- one-shot
# feed(), byte-at-a-time feed(), and several fixed-seed random chunkings
# must all produce byte-identical sink output for every feed()-driven
# golden-vector block.
# ---------------------------------------------------------------------------

def _block_wire_bytes(actions):
    parts = []
    for kind, payload in actions:
        if kind != "IN":
            return None
        parts.append((payload + "\n").encode("ascii"))
    return b"".join(parts)


def _run_block_feed(lib, setup_calls, wire_bytes, chunk_sizes):
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
    sizes = []
    remaining = total
    while remaining > 0:
        size = rng.randint(1, 5)
        sizes.append(size)
        remaining -= size
    return sizes


def test_feed_chunk_split_equivalence_golden_vectors(tmp_path):
    lib = _load_shim(tmp_path)
    blocks = _parse_golden_vectors(_GOLDEN_VECTORS_PATH)
    assert blocks, "no golden vectors parsed -- fixture path or format broke"
    rng = random.Random(20260822)  # fixed seed: a failure must reproduce
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
# Entirely unaffected by any of the 2026-08-22 changes.
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
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSendDebug(handle, b"hello\nworld\r\n")
        assert _sink_lines(lib, handle) == ["debug helloworld"]
    finally:
        lib.phDestroy(handle)


def test_send_debug_text_that_is_entirely_newlines_is_the_empty_case(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSendDebug(handle, b"\n\r\n\r")
        assert _sink_lines(lib, handle) == ["debug"]
    finally:
        lib.phDestroy(handle)


def test_send_debug_exactly_240_bytes_is_not_truncated(tmp_path):
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
# RUN: invocation by name. Its MERITS-rejection codes (unknown function /
# bad arg, the adapter's own call) are unaffected by the decode-failure
# change; its STRUCTURAL decode failures (no function name at all; too
# many raw tokens) now NACK.
# ---------------------------------------------------------------------------

def test_run_zero_args_calls_adapter_with_empty_argv(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_OK)
        _feed(lib, handle, "RUN blink #1\n")
        assert lib.phRunCalls(handle) == 1
        assert lib.phLastRunNameMatches(handle, b"blink")
        assert lib.phLastRunArgc(handle) == 0
        assert _sink_lines(lib, handle) == [_ack(1)]
    finally:
        lib.phDestroy(handle)


def test_run_passes_positional_args_in_order(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_OK)
        _feed(lib, handle, "RUN add 19 23 #1\n")
        assert lib.phRunCalls(handle) == 1
        assert lib.phLastRunNameMatches(handle, b"add")
        assert lib.phLastRunArgc(handle) == 2
        assert lib.phLastRunArgMatches(handle, 0, b"19")
        assert lib.phLastRunArgMatches(handle, 1, b"23")
        assert _sink_lines(lib, handle) == [_ack(1)]
    finally:
        lib.phDestroy(handle)


def test_run_void_return_is_the_ack_alone(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_OK)
        lib.phSetRunHasResult(handle, 0)
        _feed(lib, handle, "RUN blink #1\n")
        assert _sink_lines(lib, handle) == [_ack(1)]
    finally:
        lib.phDestroy(handle)


def test_run_with_return_value_is_ack_then_ret(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_OK)
        lib.phSetRunHasResult(handle, 1)
        lib.phSetRunResultText(handle, b"42")
        _feed(lib, handle, "RUN add 19 23 #1\n")
        assert _sink_lines(lib, handle) == [_ack(1), "ret 42 #1"]
    finally:
        lib.phDestroy(handle)


def test_run_id_zero_is_a_stale_retransmit_never_executes(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_OK)
        lib.phSetRunHasResult(handle, 1)
        lib.phSetRunResultText(handle, b"42")
        _feed(lib, handle, "RUN add 19 23 #0\n")
        assert lib.phRunCalls(handle) == 0, "the function must NOT run"
        assert _sink_lines(lib, handle) == [_ack(0)]
    finally:
        lib.phDestroy(handle)


def test_run_unknown_function_is_a_merits_rejection(tmp_path):
    """RUN's own grammar (name + args) was satisfied -- an unrecognized
    function name is the ADAPTER's own call, not a decode failure, so
    this still acks then errs."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_UNKNOWN)
        _feed(lib, handle, "RUN no_such_function #1\n")
        assert lib.phRunCalls(handle) == 1
        assert _sink_lines(lib, handle) == [_ack(1), "err 1 #1"]
    finally:
        lib.phDestroy(handle)


def test_run_bad_arg_is_a_merits_rejection(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_BADARG)
        _feed(lib, handle, "RUN add one two #1\n")
        assert lib.phRunCalls(handle) == 1
        assert _sink_lines(lib, handle) == [_ack(1), "err 2 #1"]
    finally:
        lib.phDestroy(handle)


def test_run_no_function_name_at_all_is_malformed_no_reply(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "RUN\n")
        assert lib.phRunCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == []
    finally:
        lib.phDestroy(handle)


def test_run_only_an_id_token_is_a_decode_failure_now(tmp_path):
    """"RUN #1" -- the ONLY field present is consumed as the mandatory
    id, leaving nothing to be the function name. Before 2026-08-22 this
    still acked (the ack-always-first rule); it is now a STRUCTURAL
    decode failure (no function name at all is exactly the "line did not
    arrive intact" case), so it NACKS and holds the sequence in place."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        _feed(lib, handle, "RUN #1\n")
        assert lib.phRunCalls(handle) == 0
        assert _sink_lines(lib, handle) == [_nack(1), "err 2 #1"]
    finally:
        lib.phDestroy(handle)


def test_run_last_field_hash_non_digit_is_malformed_no_reply(tmp_path):
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
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_OK)
        _feed(lib, handle, "RUN foo #abc 5 #1\n")
        assert lib.phRunCalls(handle) == 1
        assert lib.phLastRunArgc(handle) == 2
        assert lib.phLastRunArgMatches(handle, 0, b"#abc")
        assert lib.phLastRunArgMatches(handle, 1, b"5")
        assert _sink_lines(lib, handle) == [_ack(1)]
    finally:
        lib.phDestroy(handle)


def test_run_too_many_args_is_a_decode_failure(tmp_path):
    """kMaxRunArgs is a firmware resource limit, not a claim about any
    real function's arity -- exceeding it is a STRUCTURAL decode
    failure now (2026-08-22), not a merits rejection, since the adapter
    is never even called."""
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        args = " ".join(str(i) for i in range(17))  # kMaxRunArgs == 16
        _feed(lib, handle, f"RUN foo {args} #1\n")
        assert lib.phRunCalls(handle) == 0
        assert lib.phMalformedCount(handle) == 1
        assert _sink_lines(lib, handle) == [_nack(1), "err 2 #1"]
    finally:
        lib.phDestroy(handle)


def test_run_at_kmaxrunargs_is_accepted(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_OK)
        args = " ".join(str(i) for i in range(16))
        _feed(lib, handle, f"RUN foo {args} #1\n")
        assert lib.phRunCalls(handle) == 1
        assert lib.phLastRunArgc(handle) == 16
        assert lib.phLastRunArgMatches(handle, 15, b"15")
        assert _sink_lines(lib, handle) == [_ack(1)]
    finally:
        lib.phDestroy(handle)


def test_run_result_text_is_sanitized_before_reaching_the_sink(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.phCreate()
    try:
        lib.phSetRunResult(handle, RESULT_OK)
        lib.phSetRunHasResult(handle, 1)
        lib.phSetRunResultText(handle, b"line1\nline2\r\n")
        _feed(lib, handle, "RUN foo #1\n")
        assert _sink_lines(lib, handle) == [_ack(1), "ret line1line2 #1"]
    finally:
        lib.phDestroy(handle)
