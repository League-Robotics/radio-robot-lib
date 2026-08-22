# WiFi link — dual-plane TCP-REPL + UDP-protocol over one AT module

**Status:** built and bench-proven. The reference implementation is
MicroPython (`nezha-upy`: `src/core/wifi_at.py` + a thin native UART
shim), proven live on the robot `tovez` 2026-08-21/22
(`nezha-upy/docs/bench-log-tovez-wifi-2026-08-21.md`, cited throughout
as *bench §N*). This document is the porting authority for
implementing the same link in another language or on another host —
what the ports are, what the module dialect is, and the operational
characteristics that were measured rather than designed.

The protocol the UDP plane carries is specified separately in
[protocol.md](protocol.md); this document is everything *under* it —
how bytes get to and from the module, and how two planes share it.

---

## 1. The shape, in one picture

One WiFi module, spoken to over one UART in AT command mode, carrying
two independent planes at the same well-known port number:

```
   host (bench Mac, phone, relay, …)
      │ TCP :7654                          │ UDP :7654
      ▼                                    ▼
  ┌──────────────────────────────────────────────────────────┐
  │  ESP-AT module (CIPMUX=1: one server + one UDP socket)   │
  │  ELECFREAKS Planet X WiFi — Ai-WB2-12F (BL602),          │
  │  Ai-Thinker "Combo AT" dialect                           │
  └──────────────────────────────────────────────────────────┘
      │ UART 115200, +IPD-framed, both planes interleaved
      ▼
  ┌──────────────────────────────────────────────────────────┐
  │  WifiAtLink (the state machine this document specifies)  │
  │    bring-up: RST → configure → join → DHCP → sockets     │
  │    demux: +IPD by link id → REPL ring | protocol engine  │
  │    send: ONE AT+CIPSEND per datagram, prompt-then-payload│
  └──────────────────────────────────────────────────────────┘
      │ link N (TCP client)          │ link 4 (UDP)
      ▼                              ▼
  stdio REPL mirror            ProtocolHandler
  (the ONE shared REPL)        (protocol.md — its own handler
                                instance for this transport)
```

**The two planes are one wire.** Every byte of both planes arrives
interleaved on the same UART inside `+IPD` frames; the link id inside
each frame is the only thing that separates a REPL keystroke from a
protocol command. The demux is therefore the heart of a port, not a
detail.

---

## 2. Port map

| where | port | proto | role |
|---|---|---|---|
| robot | **7654** | TCP (server) | REPL mirror — a raw pipe onto the robot's one interactive REPL |
| robot | **7654** | UDP (local port) | protocol plane — one datagram is one protocol line, no framing |
| host | **7655** | UDP (local port, **fixed**) | the host's own end of the protocol plane |

Both robot planes deliberately share the number 7654: one number to
remember, and the transport (TCP vs UDP) is the plane selector.

**The host's port is fixed at 7655 so the robot never has to discover
it twice.** The robot learns the host's address from the first
datagram it receives (§6); because the host end is a well-known port,
a host *restart* needs no re-discovery — only a host IP change does.
A port must bind its host-side UDP socket to 7655 specifically, not an
ephemeral port, or the robot will learn an address that stops existing
when the host process does.

---

## 3. The load-bearing mode decision: CIPMUX=1, never passthrough

The module supports a transparent passthrough mode (`AT+CIPMODE=1`)
that turns the UART into a verbatim byte pipe to a single socket. An
earlier C++ exploration used it and measured it working well
(2032/2032 bytes byte-perfect, 30/30 UDP round trips at a 49.5 ms
median — the median being the AT firmware's own packetization timer).

**This design rejects passthrough, on purpose.** Passthrough is one
pipe; this link needs two planes at once. `CIPMODE=0` + `CIPMUX=1`
(multi-connection command mode) is what lets a TCP REPL session and
the UDP protocol plane coexist on one module — the dual-plane
concurrency was bench-proven with a TCP session held open for its full
requested window while a complete protocol conversation ran over UDP
(bench §22, §32). The price is that every outbound datagram costs an
`AT+CIPSEND` exchange (§7); the packetization latency passthrough
paid per datagram is paid here as command overhead instead.

A port that only ever needs one plane could choose passthrough — but
it is then implementing a different, smaller design than this one.

---

## 4. The UART underneath

- **115200 baud**, 8N1. On the reference robot the module sits on an
  RJ11 jack wired TX=P8, RX=P1, driven by the nRF52's **second** UARTE
  (UARTE1) — the stock MicroPython port exposes only one UART and
  retargets it, so the second one needs a native shim.
- **250-byte TX and RX ring buffers** at the driver level.
- **Every operation is non-blocking.** `write()` queues at most what
  fits the TX buffer *right now* and returns the count; the caller
  retries the remainder on a later tick. Reads drain what is buffered.
  Nothing at this layer sleeps, polls in a loop, or waits.
- **Bytes enter the state machine in exactly one place.** All demux
  consumers (AT reply matchers, `+IPD` parser, status-line
  accumulator) pull from one per-byte feed; two readers on one buffer
  is the classic way to lose bytes to the wrong consumer.

---

## 5. Bring-up: the state machine

States, in order: `configure → join → address → server → ready`, with
`backoff` reachable from any failure. All driven from a single
periodic pump (the reference calls it every ~24 ms from its scheduled
main-context pump — never from an interrupt or GC hook), each call
doing one bounded step.

### 5.1 AT command/await mechanics

One command in flight at a time: write `<command>\r\n` (one UART
write), then arm an *expect* matcher and a deadline. Every subsequent
inbound byte feeds incremental substring matchers — match on the
expect token, reject on any of `ERROR`, `FAIL`, `busy`. Matchers are
byte-at-a-time with no line buffer, so a token is recognized however
it is split across reads and whatever noise precedes it (a dropped
byte elsewhere in the reply must not cost the token).

Timeouts: ordinary commands **4000 ms**; the join **15000 ms**;
backoff delay **5000 ms** before restarting the whole sequence.

### 5.2 The configure sequence (verbatim)

| command | expect | timeout [ms] | on failure |
|---|---|---|---|
| `AT+RST` | `ready` | 6000 | tolerate |
| `AT` | `OK` | 2000 | tolerate (absorbs boot-banner stragglers) |
| `ATE0` | `OK` | 4000 | tolerate |
| `AT+CIPMODE=0` | `OK` | 4000 | tolerate |
| `AT+CIPSERVER=0` | `OK` | 4000 | tolerate |
| `AT+CIPCLOSE=5` | `OK` | 4000 | tolerate |
| `AT+CIPCLOSE` | `OK` | 4000 | tolerate |
| `AT+CWMODE=1` | `OK` | 4000 | **backoff** |
| `AT+CIPMUX=1` | `OK` | 4000 | **backoff** |
| `AT+CIPDINFO=1` | `OK` | 4000 | tolerate |

Two things here are load-bearing:

1. **`AT+RST` comes first because the module keeps state across host
   resets.** It is powered from the robot's own rail, not the MCU: an
   nRF52 reflash or reset leaves the module's AP join, server, and
   sockets fully intact. The RST wipes them so bring-up starts from a
   known state. (Bench discipline corollary: physically power-cycle
   the module before a bring-up *session* too — a stale auto-rejoin
   already in progress can race the RST.)
2. **`AT+CIPDINFO=1` is what makes peer-learning possible** — it makes
   every `+IPD` carry the sender's ip/port inline (§6). Without it the
   UDP plane cannot learn who to reply to.

The teardown commands (`CIPSERVER=0`, `CIPCLOSE=5`, `CIPCLOSE`) are
tolerated-failure because on a genuinely fresh module they answer
`ERROR` (nothing to close) — that is expected, not a fault.

### 5.3 Join: poll before commanding (measured landmine)

**Query `AT+CWJAP?` first** — up to 6 polls at 1500 ms each (~9 s) —
expecting `+CWJAP:"<ssid>"`, and only *then* fall back to an explicit
`AT+CWJAP="<ssid>","<password>"`. The module auto-rejoins its last AP
after `AT+RST`; firing an explicit join into an in-progress auto-join
answers `busy`/`ERROR`, and was observed producing a
join → backoff → RST near-livelock. Letting the auto-join land first
is the fix, not a nicety.

**Measured join timing: 6–170 s** from a fresh RST to a reachable
address (bench §7). The variance is the module's own; the link never
spontaneously dropped once ready in any session. A port's supervisor
must tolerate this whole range — a 30 s "surely it's up by now"
timeout will produce false failures.

### 5.4 Address: DHCP

`AT+CWDHCP=1,1`, tolerant of any outcome (match, reject, or timeout
all advance — the auto-join usually already has DHCP running). Static
addressing (`AT+CIPSTA`) is a documented extension point, not
implemented. Consequence: **the robot's address is whatever DHCP
grants**, and any per-robot "expected IP" recorded in config is a
convention enforced by DHCP reservation, not by the robot — the bench
measured exactly this divergence (module landed on `.1.196` while the
fleet convention said `.4.11`, bench §8). Confirm the live address
fresh each session; do not bake it in.

### 5.5 Sockets

1. `AT+CIPSERVER=1,7654` — the TCP REPL server (strict; backoff on
   failure).
2. `AT+CIPSTART=4,"UDP","255.255.255.255",7655,7654,2` — the protocol
   socket, opened on **link id 4** explicitly: remote
   `255.255.255.255:7655` as a placeholder (no peer is known yet),
   local port 7654, **UDP mode 2** (remote endpoint updates to the
   last sender). Strict; backoff on failure.

Link id 4 is pinned by convention so the demux can route by a
constant. (The reference code names it `V5_LINK` — a historical name
from the protocol generation it was built under; the plane today
carries the current [protocol.md](protocol.md) line protocol. Port the
constant, not the name.)

Then the link is `ready` and stays there; there is no periodic
re-verification state.

---

## 6. Inbound demux — the heart of the port

All inbound UART bytes flow one at a time through this priority
ladder:

1. **Payload capture.** If a `+IPD` header was just completed,
   the next `<len>` bytes are that frame's payload, captured
   verbatim (binary-safe — payload bytes must never leak into the
   matchers below).
2. **`+IPD` header parse.** An incremental parser recognizing both
   header forms:
   - `+IPD,<link>,<len>:` (plain), and
   - `+IPD,<link>,<len>,"<ip>",<port>:` (the CIPDINFO=1 extended form).
   Malformed headers reset the parser and resynchronize; an IP field
   longer than 15 chars is malformed.
3. **Status lines.** Remaining bytes accumulate into `\n`-terminated
   lines (`\r` stripped, 96-byte cap, overlong lines discarded), for:
   - `<link>,CONNECT` — a TCP client attached. **Newest client wins**:
     the REPL routing switches to this link unconditionally, so a
     stale abandoned session cannot shadow a fresh one. Link 4 is
     ignored here — ESP-AT reports the same lifecycle lines for the
     UDP socket, and treating them as a REPL client would misroute it.
   - `<link>,CLOSED` — if it names the current REPL client, REPL
     routing (and the stdout capture hook) deactivates.
4. **AT reply matchers** (§5.1), fed the same non-payload bytes.

Frame payload routing, by link id:

| link | destination |
|---|---|
| 4 | protocol receive queue — **one datagram is one protocol line**, handed to this transport's own `ProtocolHandler` (protocol.md); a UDP datagram IS the message, no delimiter needed or stripped |
| current REPL client | pushed into the REPL stdin ring |
| anything else | dropped |

### 6.1 Peer-learning

The robot has no configuration for the host's address. It learns the
peer **from the `+IPD` header alone** on link 4: the extended header's
ip/port become (or refresh) the current peer, and even an
**empty datagram counts as heard-from** — the header, not the payload,
is the evidence. Measured on the bench: first datagram to learned-peer
`READY` emission in **108 ms** (bench §17).

- **Peer silence forget: 60 000 ms.** If nothing is heard from the
  peer for 60 s the peer is forgotten (checked lazily at use, not on a
  timer) and outbound sends drop until a new datagram teaches a new
  peer. A host that wants the channel held open must send *something*
  more often than that — a keepalive, or just its normal traffic.
- **New-peer edge.** The link exposes "a NEW (ip, port) started
  talking" as a consumable one-shot event; the application layer uses
  it to greet a freshly connected host (the reference emits its boot
  `READY` line to the new peer). Same peer re-heard is not an edge;
  a genuinely different (ip, port) is.

---

## 7. Outbound: one `AT+CIPSEND` per datagram, drop-don't-stall

**The cardinal rule: one `AT+CIPSEND` per datagram, never per
character.** Per-character sends flood the module's command parser and
were the original per-char-flood landmine this design inherited.
Verified live: exactly one `AT+CIPSEND=4,<len>,"<ip>",<port>` per
outbound protocol line in every captured trace (bench §24).

The send engine is a two-phase, non-blocking exchange, one send in
flight at a time from a FIFO queue:

1. `AT+CIPSEND=4,<len>,"<ip>",<port>` (UDP, explicit peer address per
   send) or `AT+CIPSEND=<link>,<len>` (TCP REPL), then await the `>`
   prompt (4000 ms).
2. On the prompt: write the payload bytes; await `SEND OK` (4000 ms).

On reject or timeout at either phase: **drop the datagram and move
on** — stale data is not worth retrying, and stalling the pump is
worse than losing a frame. This matches the fleet-wide transport
policy: dropping a telemetry frame is documented behavior on every
transport; blocking the control loop is not. Sends are also dropped
(silently, by design) whenever the link is not ready or no peer is
currently known.

**REPL stdout is drained through the same engine at lower priority**:
only when the send queue is empty and no send is in flight, up to 512
bytes are pulled from the REPL stdout ring and queued as one TCP send
to the current client. Protocol traffic therefore always preempts
REPL echo, never the reverse.

### 7.1 The queue must be bounded — measured failure when it is not

The reference implementation's send queue is unbounded, and this was
measured to be its one serious operational hazard: with telemetry
enabled, frames are produced (~every 24 ms) faster than the
prompt/payload/`SEND OK` exchange can drain them (tens of ms each),
the queue grows without limit, and the heap eventually exhausts —
observed as a fully wedged device needing a reset, with a repeating
`MemoryError` inside the send path (bench §20–21, §28). A telemetry
throttle for this plane (≥ 50 ms minimum interval between periodic
pushes; replies/acks always unthrottled) exists in the reference but
is not yet wired into its emission path — tracked there as an open
issue.

**A port must not reproduce this.** Treat it as a requirement, not
guidance: either throttle periodic pushes to this plane (≥ 50 ms
floor), or bound the send queue with drop-oldest, or both. Only
periodic telemetry may be throttled — command replies and acks go out
unthrottled, always.

---

## 8. The TCP REPL mirror — what it is and is not

**There is exactly one interactive REPL on the robot, and the TCP
plane is a mirror of it, not a second session** (bench §11). USB
serial and the TCP client see the identical shared stdin/stdout: a
foreground script started over USB makes the prompt unavailable on
both transports (that is "the REPL is busy", not a fault), and its
`print()` output appears interleaved on both. A port implementing the
robot side hooks the runtime's stdio (a capture hook on stdout writes,
a ring feeding stdin reads); it does not spawn an interpreter.

Mechanics a port and a *client* must both know:

- **Line ending: the REPL requires `\r` (or `\r\n`) to submit a
  line. A bare `\n` does nothing** — an automated client sending
  Unix-style newlines hangs silently at a prompt that never advances.
  This cost a bench session to find (bench §10); `nc` works because a
  terminal sends `\r\n`.
- **Stdout capture is active only while a TCP client is connected**
  (the `CONNECT`/`CLOSED` edges of §6 toggle it), so an idle mirror
  costs nothing and output is not buffered for a client that isn't
  there.
- **One client at a time, newest wins** (§6). There is no session
  multiplexing.
- Mirrored stdout reaches the client with the send engine's
  lower-priority drain (§7), so heavy protocol traffic can delay echo;
  it cannot deadlock it.
- **Proven characteristics** (bench §12–§13, §22, §32): a 5-minute
  idle hold survives; the background pump imposes no observable cost
  on foreground execution (675/675 foreground iterations at 500 ms
  cadence with the link continuously ready); the session stays
  interactive across a concurrent, complete UDP protocol conversation.

---

## 9. Application wiring

How the link composes with the rest of the firmware (reference:
`nezha-upy/src/core/boot.py`, `comms.py`):

- **Bring-up is gated on a secrets file** on the robot's filesystem:
  `wifi_secrets.json`, schema exactly `{"ssid": ..., "password": ...}`.
  Absent or malformed → the link is never constructed; the robot runs
  radio/USB-only. The file is never committed anywhere (public repos);
  it is placed on the device at bench time.
- **The link is one transport among several.** The comms layer builds
  a separate `ProtocolHandler` per transport over one shared adapter
  (protocol.md); this link's receive queue and sends are that
  handler's byte source and sink. Nothing about the protocol engine is
  WiFi-specific, and nothing in this link parses protocol bytes.
- **Single-context rule.** Every method of the link is called from one
  scheduled, main-context pump — never from an interrupt, timer
  callback, or GC hook. The link's internals are therefore free of
  locks by construction; a port that services it from multiple
  contexts has changed the design and owns the consequences.
- **Diagnostics are part of the design, not an afterthought.** A link
  that fails must say what failed in the module's own words. The
  reference exposes: `state()` (which bring-up state), and
  `debug_trace()` — the last AT command written plus a bounded ring
  (24 lines) of every raw status/reply line the module sent. Both
  exist because state numbers alone could not explain a live
  divergence ("ready but unreachable") on the bench; the AT trace
  could. Budget for the equivalent in any port.

---

## 10. Operational characteristics — the measured record

Everything here is observed, with the bench-log section that recorded
it. This is the section to re-read before declaring a port "working".

| characteristic | value | source |
|---|---|---|
| join time, fresh RST → reachable | 6–170 s, high variance | bench §7 |
| link stability once ready | no spontaneous drop observed, any session | bench §7, §12 |
| first datagram → READY at host | 108 ms | bench §17 |
| TCP idle hold | 301 s / 5-min hold, clean | bench §12 |
| foreground cost of background pump | zero missed iterations, 675 × 500 ms | bench §13 |
| dual-plane concurrency | full TCP hold across a complete UDP protocol conversation | bench §22, §32 |
| unthrottled telemetry | send-queue growth → heap exhaustion → wedged device | bench §20–21, §28 |
| module state across MCU reset | fully persists (AP join, server, sockets) | design + landmine ledger |
| DHCP vs fleet-convention address | diverged in practice; confirm live address per session | bench §8 |

Bench-methodology landmines that will bite anyone reproducing the
measurements (robot side is MicroPython-specific, but the shape
generalizes):

- Host tools that reset the MCU on connect (e.g. `mpremote exec` on a
  fresh connection) restart the WiFi bring-up — one continuous session
  per observation window, or every measurement includes a rejoin.
- The first hardware run of code that passed offline tests is still a
  first run: the reference's line-buffer reset used an operation its
  host-language subset didn't support on-device, and the resulting
  per-tick exception silently wedged the entire pump (bench §18). Run
  the port's real interpreter/toolchain against its protocol path
  before the bench, not just a host-language approximation.

---

## 11. What a port must decide deliberately

1. **Send-queue bounding / telemetry throttle** (§7.1) — mandatory;
   the reference's gap is a known defect, not a precedent.
2. **Static addressing** — the `AT+CIPSTA` hook if a deployment can't
   rely on DHCP reservations (§5.4).
3. **Security posture** — the UDP plane accepts commands from whoever
   speaks first on the LAN; there is no authentication at this layer.
   The protocol layer's own containment (e.g. `RUN`'s explicit
   registration allowlist, protocol.md §6.3) is the only gate. A port
   exposing more surface than the reference must revisit this.
4. **Multi-client REPL** — newest-wins is a deliberate simplification;
   anything richer (rejection, sessions) is new design.
5. **Jack probing** — the reference pins the module's UART pins at
   build time. The earlier C++ exploration swept multiple RJ11 jacks
   for an `AT`→`OK` responder; restore that if the fleet's wiring
   varies.
6. **Backoff/restart counting** — the reference restarts silently;
   the C++ exploration counted restarts as a health signal ("a link
   that is up but climbing is a link that keeps losing its peer").
   Cheap and worth carrying.

---

## 12. Reference implementations and evidence

| artifact | where (repo: League-Robotics/nezha-upy) |
|---|---|
| the state machine (this spec's source of truth) | `src/core/wifi_at.py` |
| native UART shim + stdio mirror rings | `native/modwifiuart.cpp`, `native/codal_app/wifi_uart_pipe.cpp`, `native/codal_app/wifi_stdio_hook.cpp` |
| application wiring | `src/core/boot.py` (secrets gate, transport registration), `src/core/comms.py` |
| offline behavioral suite (mock serial) | `tests/test_wifi_at.py` |
| host-side probes (bench clients) | `tools/wifi_tcp_probe.py`, `tools/wifi_udp_probe.py` |
| the measured record | `docs/bench-log-tovez-wifi-2026-08-21.md` §1–§34 |
| prior C++ passthrough exploration (rejected mode, useful AT findings) | radio-robot-elite worktree `micropython-exploration-repl-commands`: `src/firm/hardware/planetx/wifi_link.{h,cpp}`, `modrobot/wifi_stdio.cpp` |
