"""daemon_protocol.py -- the framed request/reply wire codec shared by
`daemon.py` (ticket 005, the server) and `daemon_client.py` (ticket
008, the client). Ticket 004's own scope (sprint.md's Architecture
Step 3, `rogo.daemon_protocol`'s own row): a PURE codec, no
socket/pipe/subprocess I/O of its own -- mirrors how `robot_v6.codec`
is a pure line codec kept separate from `robot_v6.transport` (see
codec.py's own module docstring), so the daemon's two ends can never
independently drift on the wire shape, and this module is unit-testable
with no process or socket involved at all.

This is a DIFFERENT wire than protocol-v6 (`robot_v6.codec`): that one
speaks to the robot's firmware in a terse space-delimited grammar this
codebase does not control the far end of. This one speaks between two
Python processes this codebase controls BOTH ends of, so it uses plain
newline-delimited JSON instead of inventing a second terse grammar for
no benefit -- one JSON object per line, in both directions (the issue's
own Requirement 1 says "one request line in, one JSON reply line out";
this module makes REQUESTS JSON too, since the daemon's own command
grammar needs to carry structured, possibly-nested arguments -- e.g. a
`goto` call's target pose -- that a space-delimited line cannot express
without reinventing JSON's own escaping rules badly).

Wire shape (this module's only "spec" -- see ticket 004's own
Documentation-updates note: this is host-tooling internal, not part of
the protocol-v6 wire `docs/design/protocol.md` governs)::

    request-line ::= JSON object '\\n', with keys:
                      "id"     -- integer correlation id, chosen by the
                                  client, echoed verbatim on the
                                  matching reply (this module's only
                                  request/reply pairing mechanism --
                                  there is no other ordering guarantee
                                  a client may rely on: an estop request
                                  MAY be answered before an
                                  earlier-sent, still-running request's
                                  own reply, exactly the priority
                                  ticket 005's server core implements).
                      "verb"   -- non-empty string, the dispatch key
                                  (e.g. "drive", "estop", "config_get")
                                  -- an ordinary Python identifier-ish
                                  token, NOT a protocol-v6 wire verb;
                                  ticket 005's dispatch table decides
                                  what verbs exist and which of them
                                  (at minimum "estop"/"halt") jump the
                                  server's priority queue. This module
                                  has no verb table of its own -- exactly
                                  robot_v6.codec's own "no verb table"
                                  choice (codec.py's module docstring),
                                  for the identical reason: a generic
                                  codec needs neither to encode/decode
                                  a line.
                      "params" -- object, the verb's named arguments;
                                  defaults to `{}` when a request
                                  carries no arguments at all.

    reply-line   ::= JSON object '\\n', with keys:
                      "id"     -- the SAME integer the matching request
                                  carried, echoed back unchanged.
                      "result" -- present on success; any JSON value
                                  (including `null`) the dispatched verb
                                  returned.
                      "error"  -- present on failure INSTEAD of
                                  "result"; an object with a "message"
                                  string and a "type" string (the
                                  raising exception's class name, or
                                  "Error" when the caller building the
                                  Reply does not supply one) -- enough
                                  for a client to report what went
                                  wrong without this module inventing an
                                  error-code taxonomy no caller has
                                  asked for yet.

Framing choice: newline-delimited (one complete JSON object per line),
not length-prefixed -- this module hands a caller a single already-
assembled line's text in and out; ANY transport that can deliver
"one line at a time" (a Unix socket via `Transport.read_lines()`-style
reassembly, or a subprocess's stdio pipes) can carry this protocol
unchanged, exactly the "two transports, same protocol" split the
issue's own Requirement 2 asks for. Newlines never appear inside a
value here because `json.dumps()` always escapes them (`\\n` inside a
JSON string, never a literal newline byte) -- so a line boundary is
unambiguous.

Every function below raises `ProtocolError` -- and ONLY
`ProtocolError` -- on malformed input, never lets `json.JSONDecodeError`,
`KeyError`, or `TypeError` escape past this module's own boundary: a
caller (the daemon server reading a possibly-adversarial client line,
or a client reading a possibly-truncated reply) gets exactly one
exception type to catch regardless of which way a line is malformed,
matching `robot_v6.codec.parse_reply()`'s own "raises ValueError, one
type, for any malformed line" contract on the robot-facing wire.
"""

from __future__ import annotations

import dataclasses
import json


class ProtocolError(ValueError):
    """Raised by `decode_request()`/`decode_reply()` when a line is not
    valid JSON, or is valid JSON that does not match this module's
    request/reply shape (missing/mistyped field). The one exception
    type both decode functions ever raise -- see module docstring."""


@dataclasses.dataclass(frozen=True)
class Request:
    """One parsed (or about-to-be-encoded) request. `params` defaults
    to an empty mapping so a verb with no arguments (e.g. "estop") can
    be constructed as `Request(id=1, verb="estop")` without a caller
    spelling out `params={}` itself."""

    id: int  # noqa: A003 -- matches this module's own wire vocabulary
    verb: str
    params: dict[str, object] = dataclasses.field(default_factory=dict)


@dataclasses.dataclass(frozen=True)
class ReplyError:
    """The structured payload of a failed `Reply` -- see module
    docstring's "error" key."""

    message: str
    type: str = "Error"


@dataclasses.dataclass(frozen=True)
class Reply:
    """One parsed (or about-to-be-encoded) reply. Exactly one of
    `result`/`error` is meaningful for a given reply: `error is None`
    means success (`result` is the verb's return value, which may
    itself legitimately be `None`); `error is not None` means failure
    and `result` is not consulted. Use the `ok()`/`fail()`
    constructors below rather than setting both fields by hand."""

    id: int  # noqa: A003 -- matches this module's own wire vocabulary
    result: object = None
    error: ReplyError | None = None

    @classmethod
    def ok(cls, request_id: int, result: object = None) -> "Reply":
        """Build a success reply echoing `request_id`."""
        return cls(id=request_id, result=result, error=None)

    @classmethod
    def fail(cls, request_id: int, message: str, *, type: str = "Error") -> "Reply":  # noqa: A002
        """Build a failure reply echoing `request_id`. `type` is
        informational only (typically the raising exception's class
        name) -- this module attaches no behavior to any particular
        value."""
        return cls(id=request_id, result=None, error=ReplyError(message=message, type=type))


def encode_request(request: Request) -> str:
    """Format one request as a single JSON-object line, WITHOUT a
    trailing '\\n' (the transport owns line termination, matching
    `robot_v6.codec.encode_command()`'s own "no trailing newline"
    convention -- see that module's docstring)."""
    if not request.verb or not isinstance(request.verb, str):
        raise ValueError(f"not a legal verb token: {request.verb!r}")
    payload = {"id": request.id, "verb": request.verb, "params": dict(request.params)}
    return json.dumps(payload, separators=(",", ":"))


def decode_request(line: str) -> Request:
    """Parse one already-delimited request line (no trailing '\\n').
    Raises `ProtocolError` on anything that is not valid JSON, is not a
    JSON object, or is missing/mistypes a required field -- never lets
    a raw `json.JSONDecodeError`/`KeyError`/`TypeError` escape (module
    docstring)."""
    data = _decode_json_object(line)

    if "id" not in data:
        raise ProtocolError("request missing required field: 'id'")
    request_id = data["id"]
    if not isinstance(request_id, int) or isinstance(request_id, bool):
        raise ProtocolError(f"request 'id' must be an integer, got {request_id!r}")

    if "verb" not in data:
        raise ProtocolError("request missing required field: 'verb'")
    verb = data["verb"]
    if not isinstance(verb, str) or not verb:
        raise ProtocolError(f"request 'verb' must be a non-empty string, got {verb!r}")

    params = data.get("params", {})
    if not isinstance(params, dict):
        raise ProtocolError(f"request 'params' must be an object, got {params!r}")

    return Request(id=request_id, verb=verb, params=params)


def encode_reply(reply: Reply) -> str:
    """Format one reply as a single JSON-object line, WITHOUT a
    trailing '\\n' (see `encode_request()`'s own note)."""
    payload: dict[str, object] = {"id": reply.id}
    if reply.error is not None:
        payload["error"] = {"message": reply.error.message, "type": reply.error.type}
    else:
        payload["result"] = reply.result
    return json.dumps(payload, separators=(",", ":"))


def decode_reply(line: str) -> Reply:
    """Parse one already-delimited reply line (no trailing '\\n').
    Raises `ProtocolError` on anything that is not valid JSON, is not a
    JSON object, or is missing/mistypes a required field -- same
    fail-closed contract as `decode_request()`."""
    data = _decode_json_object(line)

    if "id" not in data:
        raise ProtocolError("reply missing required field: 'id'")
    reply_id = data["id"]
    if not isinstance(reply_id, int) or isinstance(reply_id, bool):
        raise ProtocolError(f"reply 'id' must be an integer, got {reply_id!r}")

    error_data = data.get("error")
    if error_data is not None:
        if not isinstance(error_data, dict) or "message" not in error_data:
            raise ProtocolError(
                f"reply 'error' must be an object with a 'message' field, got {error_data!r}"
            )
        message = error_data["message"]
        error_type = error_data.get("type", "Error")
        if not isinstance(message, str):
            raise ProtocolError(f"reply 'error.message' must be a string, got {message!r}")
        if not isinstance(error_type, str):
            raise ProtocolError(f"reply 'error.type' must be a string, got {error_type!r}")
        return Reply(id=reply_id, result=None, error=ReplyError(message=message, type=error_type))

    return Reply(id=reply_id, result=data.get("result"))


def _decode_json_object(line: str) -> dict:
    """Shared first step for `decode_request()`/`decode_reply()`: parse
    `line` as JSON and confirm it is an object, raising `ProtocolError`
    (never a raw `json.JSONDecodeError`) either way."""
    try:
        data = json.loads(line)
    except json.JSONDecodeError as exc:
        raise ProtocolError(f"not valid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise ProtocolError(f"expected a JSON object, got {type(data).__name__}")
    return data
