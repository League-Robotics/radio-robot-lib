"""tests/host/robot_v6/test_codec.py -- robot_v6.codec: format a
command line, parse a reply line. Pure Python, no C++, no subprocess --
these are the fastest tests in this directory on purpose.
"""

from __future__ import annotations

import pathlib

import pytest

from robot_v6 import codec

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_GOLDEN_VECTORS_PATH = _REPO_ROOT / "tests" / "protocol" / "golden_vectors.txt"


# ---------------------------------------------------------------------------
# encode_command()
# ---------------------------------------------------------------------------

def test_encode_command_with_id():
    assert codec.encode_command("WHEELS_V", 100, 100, 1000, seq_id=1) == (
        "WHEELS_V 100 100 1000 #1")


def test_encode_command_without_id_for_unsequenced_verbs():
    assert codec.encode_command("PING") == "PING"
    assert codec.encode_command("HELLO") == "HELLO"
    assert codec.encode_command("ESTOP") == "ESTOP"


def test_encode_command_negative_and_zero_id():
    assert codec.encode_command("STATUS", seq_id=0) == "STATUS #0"
    assert codec.encode_command("MOVE_X", 400, -1571, 200, 5000, seq_id=3) == (
        "MOVE_X 400 -1571 200 5000 #3")


def test_encode_command_rejects_negative_id():
    with pytest.raises(ValueError):
        codec.encode_command("SET", "x", 1, seq_id=-1)


def test_encode_command_rejects_illegal_verb():
    with pytest.raises(ValueError):
        codec.encode_command("", seq_id=1)
    with pytest.raises(ValueError):
        codec.encode_command("TWO WORDS", seq_id=1)


def test_encode_command_float_field_has_no_exponent():
    # protocol.md S2: "No exponents, no NaN, no inf" -- a Python float
    # small enough to `repr()` in scientific notation must still come
    # out as a plain, LOSSLESS decimal on the wire -- not merely
    # exponent-free but also not silently rounded to 0 by a fixed
    # `.6f`-style formatter (a real bug caught while writing this test:
    # see _format_field()'s own docstring for the fix).
    line = codec.encode_command("SET", "x", 1e-8, seq_id=1)
    numeric_field = line.split()[2]
    assert "e" not in numeric_field.lower()
    assert numeric_field == "0.00000001"

    line2 = codec.encode_command("SET", "wheel_control.pid_kp", 0.03, seq_id=1)
    assert line2 == "SET wheel_control.pid_kp 0.03 #1"

    line3 = codec.encode_command("SET", "x", 1e18, seq_id=1)
    numeric_field3 = line3.split()[2]
    assert "e" not in numeric_field3.lower()
    assert numeric_field3 == "1000000000000000000"


def test_encode_command_rejects_bool_field():
    with pytest.raises(TypeError):
        codec.encode_command("SET", "x", True, seq_id=1)


def test_encode_command_rejects_non_finite_float():
    with pytest.raises(ValueError):
        codec.encode_command("SET", "x", float("nan"), seq_id=1)
    with pytest.raises(ValueError):
        codec.encode_command("SET", "x", float("inf"), seq_id=1)
    with pytest.raises(ValueError):
        codec.encode_command("SET", "x", float("-inf"), seq_id=1)


# ---------------------------------------------------------------------------
# parse_reply()
# ---------------------------------------------------------------------------

def test_parse_reply_ack_has_no_id_bare_leading_field():
    reply = codec.parse_reply("ack 1 0 none")
    assert reply.verb == "ack"
    assert reply.fields == ("1", "0", "none")
    assert reply.id is None, "ack's own leading field is NOT a '#id' token"


def test_parse_reply_nack_same_shape_as_ack():
    reply = codec.parse_reply("nack 3 2 stop")
    assert reply == codec.Reply(verb="nack", fields=("3", "2", "stop"), id=None)


def test_parse_reply_err_extracts_trailing_id():
    reply = codec.parse_reply("err 2 #1")
    assert reply.verb == "err"
    assert reply.fields == ("2",)
    assert reply.id == 1


def test_parse_reply_ret_extracts_trailing_id():
    reply = codec.parse_reply("ret 42 #7")
    assert reply.fields == ("42",)
    assert reply.id == 7


def test_parse_reply_no_id_present():
    reply = codec.parse_reply("pong 38472")
    assert reply.fields == ("38472",)
    assert reply.id is None


def test_parse_reply_multi_field_banner():
    reply = codec.parse_reply("device NEZHA2 robot testbot SN001")
    assert reply.verb == "device"
    assert reply.fields == ("NEZHA2", "robot", "testbot", "SN001")
    assert reply.id is None


def test_parse_reply_collapses_space_runs_and_trims():
    # protocol.md S2: "a run of spaces is ONE separator; leading/
    # trailing whitespace on the line is ignored" -- str.split() with
    # no argument already gives this for free; pinned as a regression.
    reply = codec.parse_reply("  ack   1   0   none  ")
    assert reply == codec.Reply(verb="ack", fields=("1", "0", "none"), id=None)


def test_parse_reply_blank_line_raises():
    with pytest.raises(ValueError):
        codec.parse_reply("")
    with pytest.raises(ValueError):
        codec.parse_reply("    ")


def test_parse_reply_bare_verb_no_fields():
    reply = codec.parse_reply("estop")
    assert reply == codec.Reply(verb="estop", fields=(), id=None)


def test_parse_reply_debug_text_kept_as_one_field_per_token():
    # `debug` is a rest-of-line verb on the wire, but this module has no
    # verb table -- it just splits on whitespace like every other reply,
    # which is a faithful (if word-tokenized rather than rest-of-line)
    # representation for a caller that wants to re-join it.
    reply = codec.parse_reply("debug something happened")
    assert reply.verb == "debug"
    assert reply.fields == ("something", "happened")
    assert " ".join(reply.fields) == "something happened"


def test_parse_reply_a_trailing_hash_token_that_is_not_all_digits_is_not_an_id():
    # protocol.md S9.1: the id grammar is strictly '#' [0-9]+ -- a field
    # that merely starts with '#' but isn't purely digits after it
    # (e.g. a hypothetical malformed echo) must not be misread as one.
    reply = codec.parse_reply("ret #notanumber #7")
    assert reply.fields == ("#notanumber",)
    assert reply.id == 7


# ---------------------------------------------------------------------------
# parse_kv_fields()
# ---------------------------------------------------------------------------

def test_parse_kv_fields():
    reply = codec.parse_reply(
        "status ready=1 active=0 connL=1 connR=1 otos=1 wedge=0 flags=d8 "
        "tlm=pose next=2")
    kv = codec.parse_kv_fields(reply)
    assert kv["ready"] == "1"
    assert kv["flags"] == "d8"
    assert kv["next"] == "2"
    assert len(kv) == 9


# ---------------------------------------------------------------------------
# Round trip: encode_command()'s own output must parse back the way it
# was built, for both the id and no-id shapes.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("verb,fields,seq_id", [
    ("WHEELS_V", (100, 100, 1000), 1),
    ("GO_TO_W", (300, -150, 150, 10, 5000), 99),
    ("SET", ("wheel_control.pid_kp", 0.03), 42),
    ("STOP", (), 0),
])
def test_encode_then_parse_round_trips_fields_and_id(verb, fields, seq_id):
    line = codec.encode_command(verb, *fields, seq_id=seq_id)
    reply = codec.parse_reply(line)
    assert reply.verb == verb
    assert reply.id == seq_id
    assert reply.fields == tuple(codec._format_field(f) for f in fields)


# ---------------------------------------------------------------------------
# Golden-vector reuse (docs/design/protocol.md's "archetype" framing):
# every `OUT <wire line>` in tests/protocol/golden_vectors.txt is a
# REAL reply line the C++ handler is proven to emit. This module has no
# adapter/sequencing semantics of its own to replay the SETUP/IN
# choreography against (that fixture format is built around driving a
# mock C++ adapter through ctypes, not around a standalone codec), so
# what is reused here is narrower and still real: every literal OUT
# line must parse without raising, and the id/no-id split this codec
# makes must agree with the '#' convention every vector already
# follows (a line ending in a WELL-FORMED '#<digits>' token gets an id;
# everything else does not, INCLUDING every ack/nack line, which the
# fixture never spells with one).
# ---------------------------------------------------------------------------

def _golden_out_lines() -> list[str]:
    lines = []
    for raw in _GOLDEN_VECTORS_PATH.read_text().splitlines():
        raw = raw.strip()
        if raw.startswith("OUT ") and raw != "OUT NONE":
            lines.append(raw[len("OUT "):])
    return lines


@pytest.mark.parametrize("line", _golden_out_lines())
def test_golden_vector_reply_lines_parse_without_raising(line):
    reply = codec.parse_reply(line)
    assert reply.verb == line.split()[0]
    if reply.verb in ("ack", "nack"):
        assert reply.id is None, (
            f"{line!r}: ack/nack must never be read as carrying a '#id'")
    last_token = line.split()[-1]
    if last_token.startswith("#") and last_token[1:].isdigit():
        assert reply.id == int(last_token[1:])
    else:
        assert reply.id is None
