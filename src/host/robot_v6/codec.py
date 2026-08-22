"""codec.py -- the protocol-v6 line codec: format a command line, parse
a reply line. Deliberately tiny (docs/design/protocol.md S2): the whole
wire grammar is a split on runs of whitespace plus the trailing
'#<id>' convention (S2.2/S8.6), so this module holds no verb table and
no per-verb field-count knowledge at all -- a generic caller needs
neither to format or parse a line, unlike protocol_handler.cpp's own
fixed-arity kCommandTable, which exists there only because the C++ side
also has to DISPATCH, not just encode/decode.

Grammar (docs/design/protocol.md S2)::

    line  ::= sp? verb ( sp field )* sp? '\\n'
    sp    ::= ' '+
    verb  ::= [A-Za-z][A-Za-z0-9_]*
    field ::= any bytes except ' ' and '\\n'
    id    ::= '#' [0-9]+        -- mandatory trailing token on every
                                    SEQUENCED command, and on `err`/
                                    `ret` replies (S8.6: "the id is
                                    always the LAST token"). `ack`/
                                    `nack`'s own leading field ("n" in
                                    S8.1's own notation) is a BARE,
                                    un-prefixed integer -- see
                                    parse_reply()'s own note below for
                                    why that is not an inconsistency
                                    this module needs to paper over.

Commands (host -> robot) are UPPERCASE; replies (robot -> host) are
lowercase (S2.1). This module does not enforce or check that itself --
it only formats/parses whatever verb spelling it is given, verbatim.
"""

from __future__ import annotations

import dataclasses
import decimal
import math


@dataclasses.dataclass(frozen=True)
class Reply:
    """One parsed reply line.

    `fields` excludes a trailing '#<id>' token when the line ends with
    one; `id` is that token's own value, or None when the line carries
    none (every unsequenced reply -- `pong`, `estop`, `device ...`,
    `debug ...`, `thdr`/`t`, `ack`/`nack` -- has no such token).

    NOTE: on `ack <n> <lastDone> <reason>` / `nack <n> <lastDone>
    <reason>`, the leading `n` is a BARE sequence number, not a
    '#'-prefixed id -- it lands in `fields[0]` like any other field,
    and `.id` stays None for both verbs. This is not an oversight: `n`
    identifies which SEQUENCED command the ack/nack is ABOUT, whereas
    `.id` (everywhere else it appears) identifies the correlation id of
    the reply LINE ITSELF -- an ack/nack line has no id of its own to
    carry, only an opinion about someone else's.
    """

    verb: str
    fields: tuple[str, ...]
    id: int | None  # noqa: A003 -- matches the wire's own vocabulary


def _format_field(value: object) -> str:
    """Render one Python value as a single wire field.

    Ints and strings pass through via `str()`. Floats are rendered
    WITHOUT scientific notation -- the wire grammar has no exponent
    syntax at all (docs/design/protocol.md S2). `bool` is rejected
    outright: `True`/`False` would otherwise silently become the field
    text "True"/"False", which is not a legal wire integer at all (a
    caller that means 0/1 must say so). NaN/+-inf are rejected too --
    S2's own "no NaN, no inf" is a caller obligation this codec can at
    least catch at the point of construction, rather than emitting
    "nan"/"inf" text the wire grammar has no defined meaning for.

    A naive `f"{value:.6f}"` fallback for a scientific-notation `repr()`
    (e.g. `1e-08`) would silently ROUND AWAY any digit outside a fixed
    six-decimal window -- `1e-08` becomes `"0"`. `decimal.Decimal`
    fed `repr(value)` (the shortest string that round-trips to the same
    float) instead re-renders the SAME digits in fixed-point notation
    losslessly, however small or large `value` is.
    """
    if isinstance(value, bool):
        raise TypeError("bool is not a wire field type -- pass 0/1 explicitly")
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            raise ValueError(f"non-finite float is not a legal wire field: {value!r}")
        text = repr(value)
        if "e" in text or "E" in text:
            text = format(decimal.Decimal(text), "f")
        return text
    return str(value)


def encode_command(verb: str, *fields: object, seq_id: int | None = None) -> str:
    """Format one command line, WITHOUT the trailing '\\n' (a Transport
    owns line termination, not this module -- see transport.py).

    `seq_id`, when given, becomes the mandatory trailing '#<id>' token
    (docs/design/protocol.md S2.2/S8). Pass `seq_id=None` for the three
    unsequenced verbs (`HELLO`/`ESTOP`/`PING`) -- this module does not
    know which verbs those are (that is Session's own table,
    reliability.py), it only honors whatever the caller asks for.
    """
    if not verb or " " in verb:
        raise ValueError(f"not a legal verb token: {verb!r}")
    tokens = [verb, *(_format_field(f) for f in fields)]
    if seq_id is not None:
        if seq_id < 0:
            raise ValueError("a sequence id is never negative")
        tokens.append(f"#{seq_id}")
    return " ".join(tokens)


def parse_reply(line: str) -> Reply:
    """Parse one already-delimited reply line: no '\\n', and any
    trailing '\\r' already stripped by the Transport (docs/design/
    protocol.md S2's own "a lone CR before the terminator is a
    terminal artifact" rule is a Transport-layer concern, not this
    module's -- see transport.py's own line-reassembly code).

    Raises ValueError on a blank/all-whitespace line -- callers
    (Session.pump(), reliability.py) filter those out before calling
    this, mirroring protocol.md S2's "a blank line is ignored
    silently, not malformed" rule on the wire's own receiving end.
    """
    tokens = line.split()
    if not tokens:
        raise ValueError("blank line has no verb")
    verb, rest = tokens[0], tokens[1:]
    seq_id = None
    if rest and rest[-1].startswith("#") and rest[-1][1:].isdigit():
        seq_id = int(rest[-1][1:])
        rest = rest[:-1]
    return Reply(verb=verb, fields=tuple(rest), id=seq_id)


def parse_kv_fields(reply: Reply) -> dict[str, str]:
    """`status`'s own payload shape: `key=value` tokens, order not
    guaranteed (docs/design/protocol.md S6's own "k=v, order not
    guaranteed, unknown keys ignored"). A tiny, separate helper rather
    than something `parse_reply()` does unconditionally, since most
    replies are NOT key=value (`id`, `ver`, `ack`, `t`, ...).
    """
    return dict(field.split("=", 1) for field in reply.fields)
