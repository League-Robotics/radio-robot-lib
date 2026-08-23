"""tests/host/rogo/test_daemon_protocol.py -- `rogo.daemon_protocol`:
the framed request/reply codec shared by `daemon.py`/`daemon_client.py`
(ticket 004). Covers encode/decode round-trips for an ordinary
request/reply pair, an error-reply shape, and malformed-input decoding
(confirms the codec fails closed with `ProtocolError`, never a raw
`json.JSONDecodeError`/`KeyError`/`TypeError`) -- ticket 004's own
Testing plan. No socket/subprocess/transport import anywhere in this
file: everything here operates on plain strings.
"""

from __future__ import annotations

import json

import pytest

from rogo import daemon_protocol as dp


# ---------------------------------------------------------------------------
# Ordinary request/reply round-trip.
# ---------------------------------------------------------------------------

def test_encode_request_produces_one_self_delimited_line_with_no_trailing_newline():
    line = dp.encode_request(dp.Request(id=7, verb="drive", params={"speed": 100}))
    assert "\n" not in line
    assert json.loads(line) == {"id": 7, "verb": "drive", "params": {"speed": 100}}


def test_decode_request_round_trips_an_encoded_request():
    original = dp.Request(id=42, verb="goto", params={"x": 1.5, "y": -2.0})
    decoded = dp.decode_request(dp.encode_request(original))
    assert decoded == original


def test_request_params_default_to_empty_mapping():
    request = dp.Request(id=1, verb="estop")
    assert request.params == {}
    decoded = dp.decode_request(dp.encode_request(request))
    assert decoded.params == {}


def test_encode_reply_produces_one_self_delimited_line_with_no_trailing_newline():
    line = dp.encode_reply(dp.Reply.ok(3, result={"ok": True}))
    assert "\n" not in line
    assert json.loads(line) == {"id": 3, "result": {"ok": True}}


def test_decode_reply_round_trips_an_ok_reply_and_echoes_the_correlation_id():
    request = dp.Request(id=99, verb="ping")
    reply = dp.Reply.ok(request.id, result="pong")

    request_line = dp.encode_request(request)
    reply_line = dp.encode_reply(reply)

    decoded_request = dp.decode_request(request_line)
    decoded_reply = dp.decode_reply(reply_line)

    # The reply's id unambiguously pairs it back to the request it answers.
    assert decoded_reply.id == decoded_request.id
    assert decoded_reply.result == "pong"
    assert decoded_reply.error is None


def test_decode_reply_round_trips_a_none_result():
    reply = dp.Reply.ok(5, result=None)
    decoded = dp.decode_reply(dp.encode_reply(reply))
    assert decoded == reply


# ---------------------------------------------------------------------------
# Error-reply shape.
# ---------------------------------------------------------------------------

def test_reply_fail_builds_a_structured_error():
    reply = dp.Reply.fail(11, "unreachable target", type="UnreachableTargetError")
    assert reply.error == dp.ReplyError(message="unreachable target", type="UnreachableTargetError")
    assert reply.result is None


def test_encode_reply_error_shape_omits_result_key():
    line = dp.encode_reply(dp.Reply.fail(11, "boom"))
    data = json.loads(line)
    assert data == {"id": 11, "error": {"message": "boom", "type": "Error"}}
    assert "result" not in data


def test_decode_reply_round_trips_an_error_reply():
    original = dp.Reply.fail(23, "bad params", type="ValueError")
    decoded = dp.decode_reply(dp.encode_reply(original))
    assert decoded == original
    assert decoded.error is not None
    assert decoded.error.message == "bad params"
    assert decoded.error.type == "ValueError"


def test_reply_fail_defaults_error_type_to_error():
    reply = dp.Reply.fail(1, "something went wrong")
    assert reply.error.type == "Error"


# ---------------------------------------------------------------------------
# The codec can express an estop/halt request -- ticket 005's server core
# escalates it, but the codec itself just needs a distinguishable verb.
# ---------------------------------------------------------------------------

def test_codec_can_express_an_estop_request():
    request = dp.Request(id=2, verb="estop")
    decoded = dp.decode_request(dp.encode_request(request))
    assert decoded.verb == "estop"


# ---------------------------------------------------------------------------
# Malformed input -- decode must fail closed with ProtocolError, not an
# unhandled exception a caller can't catch cleanly.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("line", [
    "",
    "not json at all",
    "{not even valid json",
    "[1, 2, 3]",  # valid JSON, but not an object
    '"just a string"',
    "42",
])
def test_decode_request_fails_closed_on_malformed_json(line):
    with pytest.raises(dp.ProtocolError):
        dp.decode_request(line)


@pytest.mark.parametrize("line", [
    "",
    "not json at all",
    "[1, 2, 3]",
])
def test_decode_reply_fails_closed_on_malformed_json(line):
    with pytest.raises(dp.ProtocolError):
        dp.decode_reply(line)


@pytest.mark.parametrize("payload", [
    {},  # missing id and verb
    {"id": 1},  # missing verb
    {"verb": "drive"},  # missing id
    {"id": "not-an-int", "verb": "drive"},
    {"id": 1, "verb": ""},
    {"id": 1, "verb": 42},
    {"id": 1, "verb": "drive", "params": "not-an-object"},
    {"id": True, "verb": "drive"},  # bool is not an int id, even though bool is an int subclass
])
def test_decode_request_fails_closed_on_missing_or_mistyped_fields(payload):
    with pytest.raises(dp.ProtocolError):
        dp.decode_request(json.dumps(payload))


@pytest.mark.parametrize("payload", [
    {},  # missing id
    {"id": "not-an-int"},
    {"id": True},
    {"id": 1, "error": "not-an-object"},
    {"id": 1, "error": {}},  # missing message
    {"id": 1, "error": {"message": 42}},
    {"id": 1, "error": {"message": "boom", "type": 42}},
])
def test_decode_reply_fails_closed_on_missing_or_mistyped_fields(payload):
    with pytest.raises(dp.ProtocolError):
        dp.decode_reply(json.dumps(payload))


def test_decode_request_rejects_malformed_input_without_raising_json_decode_error():
    # A caller catching ProtocolError must never see json.JSONDecodeError,
    # KeyError, or TypeError leak past this module's own boundary.
    try:
        dp.decode_request("{not even valid json")
    except dp.ProtocolError:
        pass
    except Exception as exc:  # pragma: no cover -- defensive, should never trip
        pytest.fail(f"expected ProtocolError, got {type(exc).__name__}: {exc}")


def test_encode_request_rejects_empty_verb():
    with pytest.raises(ValueError):
        dp.encode_request(dp.Request(id=1, verb=""))


# ---------------------------------------------------------------------------
# No I/O dependency -- this module must not import socket/subprocess/any
# transport module (ticket 004's own acceptance criterion).
# ---------------------------------------------------------------------------

def test_module_has_no_transport_dependency():
    assert not hasattr(dp, "socket")
    assert not hasattr(dp, "subprocess")
    assert not hasattr(dp, "select")

    with open(dp.__file__, encoding="utf-8") as f:
        source_text = f.read()
    for forbidden in ("import socket", "import subprocess", "import select"):
        assert forbidden not in source_text, f"unexpected transport dependency: {forbidden!r}"
