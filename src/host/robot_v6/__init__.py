"""robot_v6 -- a protocol-v6 host client library: the line codec
(`codec.py`), the transport abstraction (`transport.py`: TCP, pipe/
stdio, and lazily-imported serial), and the host half of the
reliability layer (`reliability.py`: sequencing, pipelining, cumulative
ack/nack, resend-on-nack).

This package is a LIBRARY, not a CLI or a daemon -- `rogo` itself (the
command-line tool and its server) is a later piece of work built on top
of this one. See docs/design/protocol.md for the wire this package
implements the host side of, and tools/sim/ for a robot-less peer to
develop and test it against.
"""

from .codec import Reply, encode_command, parse_kv_fields, parse_reply
from .reliability import DoneEvent, PendingBufferFull, Session, UNSEQUENCED_VERBS
from .transport import (
    PipeTransport,
    SerialTransport,
    SocketTransport,
    StdioTransport,
    Transport,
    TransportClosed,
)

__all__ = [
    "Reply",
    "encode_command",
    "parse_kv_fields",
    "parse_reply",
    "DoneEvent",
    "PendingBufferFull",
    "Session",
    "UNSEQUENCED_VERBS",
    "PipeTransport",
    "SerialTransport",
    "SocketTransport",
    "StdioTransport",
    "Transport",
    "TransportClosed",
]
