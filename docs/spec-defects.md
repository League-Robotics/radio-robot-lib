# Protocol v6 spec — defects blocking "archetype" status

**Why this document exists.** `src/protocol/` is meant to be the archetype
other implementations are ported from — MicroPython, JavaScript, whatever comes
next. Those implementers will read [protocol-v6-spec.md](protocol-v6-spec.md),
not the C++. **An ambiguity in the spec becomes a divergence between
implementations**, and the golden-vector fixture only catches it if a vector
happens to cover that exact case.

Each defect below was found by *implementing* the spec, not by reading it.
Every one is a place where two competent implementers would produce
incompatible robots.

Status: **needs a decision from the spec owner.** The C++ handler currently
resolves each one as noted; none of these are code bugs, they are spec bugs the
code had to route around.

---

## Resolution status (updated after commit `5a5b6da`)

The 2026-08-20 grammar switch — space separators, `#`-prefixed trailing ids —
landed with these findings folded in. Recorded here so the decision trail
survives.

| | status |
|---|---|
| **D1** optional-id contradiction | **RESOLVED, structurally.** Omitted id and `#0` are now *visibly different wire forms*: omitted → executes, bare `ok`/`err` once; `#0` → executes silently. The normalize-missing-to-`0` trap is gone by construction rather than by prose — a better fix than the one recommended below. |
| **D2** `GET` unknown name | **RESOLVED.** §7.1 now states it: silent, not counted malformed. |
| **D3 / D3a** column count | **RESOLVED.** §6.4's heading is 35, §6.1's FULL row agrees, the table is declared authoritative, and `l1…l4` is spelled out as `l1 l2 l3 l4`. |
| **D4** 3× reply repeat | **SPEC FIXED, DECISION STILL OPEN.** §8.1 now says the repeat is *emission policy* owned by whatever drives per-cycle output, explicitly **not** the line codec, and marks it "specified target behavior, not yet implemented anywhere." It no longer reads as shipped. **The (a)/(b)/(c) choice for this library's handler is still unmade** — see D4 below. |

**D4 is the only live question.** D1-D3 closed in `5a5b6da`; D5/D5a closed in
`11f1d2d`, and the implementation was *verified against the new text* rather
than assumed to match — see each defect's resolution note.

One consequence for the fixture: because the bare-`ok` form replaced `ok:0`,
every golden vector's id-less arm changes shape, not just its separator.

---

## D1 — `SET`/`WHEELS`'s optional id: §7.1 and §8.2 directly contradict

**The contradiction, on the same wire form.**

§7.1's worked example:

```
SET:wheel_control.pid_kp:0.03   ->  ok:0        (or err:0:3 out of range)
```

§8.2's rule:

> Id `0` means "no ack wanted", legal on any verb with an optional id.

§7.1 says an omitted id is acked as `ok:0`. §8.2 says id `0` is not acked at
all. Both describe the identical line.

**How the C++ resolves it today:** by treating *omitted* and *explicit `0`* as
different — omitted → `ok:0` (satisfying §7.1's literal text); a literal `0`
written on the wire → no reply (satisfying §8.2's literal text).

**Why that resolution is not good enough for an archetype.** It makes two wire
forms that look equivalent behave differently, and *no sentence in the spec
says so*. An implementer who normalises "missing field → 0" during parsing —
the obvious thing to do, and what most parsers naturally do — silently gets the
opposite behaviour. This is precisely the class of bug that shows up as "the
JavaScript robot doesn't ack" three months from now.

**Recommended fix: one rule, no split.** Make omitted and `0` identical, and
keep ack-suppression as the meaning:

> The trailing `id` is optional; when omitted it is `0`. **Id `0` means no ack
> is sent.** Any id in `1..999999` is acked.

Then **correct §7.1's example** to show no reply for the id-less form:

```
SET:wheel_control.pid_kp:0.03     ->  (no reply — id 0)
SET:wheel_control.pid_kp:0.03:9   ->  ok:9
```

Ack suppression is worth keeping: streaming `WHEELS` at 20 Hz over a lossy
radio link does not want 20 acks/s back. But it needs exactly one spelling.

*Alternative, if suppression is not wanted:* delete the "no ack wanted" rule
entirely, always ack, and leave §7.1's example as-is. Simpler still. Either is
fine — what is not fine is the current pair.

---

## D2 — `GET` with an unknown name is undefined

`GET` carries no id, so there is no channel to return `err:<id>:<code>` on.
§7.1 documents the error path for `SET` and says nothing about `GET`.

**How the C++ resolves it today:** fully silent — no reply, and not counted
malformed.

**Recommended fix:** state it. Silence is the right behaviour (there is nothing
to err against), but it must be written down, because the plausible
alternatives — replying `get:<name>:` with an empty value, or counting it
malformed — are both things an implementer would reasonably choose, and each
produces a different robot.

---

## D3 — §6.4's column count is wrong: heading says 30, the table lists 35

The heading reads **`### 6.4 FULL columns (30)`**. Counting the table:

| group | names | n |
|---|---|---|
| POSE's own (§6.3) | `seq now flags x y h ox oy oh` | 9 |
| mode | `mode` | 1 |
| left encoder | `elp elv ela ele` | 4 |
| right encoder | `erp erv era ere` | 4 |
| OTOS velocity | `ovx ovy` | 2 |
| OTOS rate / age | `ow` `oa` | 2 |
| body twist | `tvx tvy` `tw` | 3 |
| line sensor | `l1…l4` | 4 |
| colour | `cr cg cb cc` | 4 |
| cycle timing | `cyb cyp` | 2 |
| | **total** | **35** |

**35, not 30.** Since `thdr:` is self-describing the mismatch is not fatal on
the wire — but an implementer sizing a fixed column array from the heading
overflows it by five, and this is firmware with no dynamic allocation.

**Recommended fix:** correct the heading to 35, and state the count *once*
rather than in a heading that can drift from the table under it.

### D3a — `l1…l4` uses an ellipsis

That row spells four column names as `l1…l4`. Every other row lists names
literally. A spec that is the source of a machine-readable conformance fixture
should not require the reader to expand a range — write `l1 l2 l3 l4`.

---

## D4 — the 3× reply repeat is specified but unimplemented, and the spec does not say who owns it

§8.1:

> Each is sent **three times** on consecutive cycles. A reply is ~12 bytes, so
> this costs nothing, and it makes an outcome survive the measured ~5% radio
> frame loss without a ring, a depth, an eviction policy or a scan.

**This is not implemented, and cannot be** by the handler as designed. "On
consecutive cycles" requires a periodic entry point and a table of pending
replies — state the handler deliberately does not have, because the settled
`no done: for WHEELS` decision keeps it a pure function of the bytes fed to it.

**So v6's entire loss-tolerance story is currently absent.** That matters more
than it looks: it is the *only* mechanism the spec offers against the measured
~5% radio frame loss, having explicitly deleted the v5 ack ring in its favour.

**This needs a decision, and it is the most consequential one in this
document:**

- **(a) The transport/app layer owns it.** The handler stays pure; whatever
  drives the loop repeats outbound replies. Spec §8.1 moves out of the handler's
  contract and says so explicitly.
- **(b) The handler owns it**, gaining a `tick()` and a small fixed pending
  table. Costs the purity, but keeps loss tolerance inside the one component
  every implementation must port — which is a real argument, since (a) means
  every future MicroPython/JS integrator has to reinvent it correctly.
- **(c) Drop it.** Acks are idempotent and the host can re-poll. Cheapest, and
  honest — but then §8.1 must be deleted rather than left as aspirational text.

**Recommendation: (b), when `MOVE`/`done:` arrives.** Loss tolerance belongs in
the archetype, not in each integrator's glue, and `done:` needs the same
pending-state machinery anyway — so it is one mechanism, added once. Until then
§8.1 should be marked "not yet implemented" rather than reading as shipped
behaviour.

---

## What to do with this

D1–D3 are small, mechanical spec edits once the calls are made. D4 is a real
design decision. **All four should be settled before anyone ports this to a
second language**, because each one is cheap to fix now and expensive to
reconcile across three implementations later.

---

## D5 — `ESTOP` vs the malformed-line `#id` recovery rule: the precedence is unstated

**Found during the space/`#id` migration (commit `a380495`), after `5a5b6da`.**

Two spec rules collide and nothing says which wins.

§2's recovery rule: a malformed line whose last token is a well-formed nonzero
`#id` gets `err #<id> <code>` — **"including unknown verbs."** No carve-out is
given for verbs whose own grammar has no id concept at all.

§5.4 / §8.2 on `ESTOP`: it *"never carries an id and is never acked — it must
not queue behind anything, including an ack."*

So what should `ESTOP #5` produce? The general rule says `err #5 <code>`.
`ESTOP`'s own rule says nothing may ever be emitted for an `ESTOP`, ever.

**How the C++ resolves it:** `ESTOP` wins. Its rule is stated more
specifically and far more emphatically, and the whole point of that rule is
that a panic stop must never queue behind an outbound reply. Pinned by
`test_estop_with_trailing_id_still_never_acks`, which fails if implemented the
other way.

**Why this is an archetype hazard, not a curiosity.** A porter implementing
the two rules *independently* — each one literally, exactly as written — gets
the opposite answer, because the generic recovery path fires before anything
`ESTOP`-specific is consulted. Nothing in the spec text tells them otherwise.
That is a safety-relevant divergence: it means an emergency stop on one
implementation emits a reply that the same command on another implementation
does not.

**RESOLVED in `11f1d2d`.** §2's recovery rule now carries the carve-out inline
(*"The one exception is `ESTOP`, which never emits a reply under any
circumstance, including this one"*), and §8.2 states it from the other side.
Elite's ticket 008 pins the same precedence so the firmware cannot re-diverge.

**Verified, not assumed:** the spec says a malformed `ESTOP` is *"dropped and
counted, silently"* — our `handleEstop()` increments `malformedCount_` and
returns without replying, which is exactly that. Pinned by
`test_estop_with_trailing_id_still_never_acks`.

### D5a — the id grammar is stricter than the general integer rule, silently

`id ::= '#' [0-9]+` admits digits only — no sign, not even `+`. But §2.2's
general rule is that every wire value is *"a base-10 ASCII integer, optionally
signed."* A porter reusing their general integer parser for the id accepts
`#+5` as id 5.

**RESOLVED in `11f1d2d`.** §8.2 now states the id digits are bare and unsigned,
that `#+5`/`#-5`/`# 5` are all malformed, and that §2.2's "optionally signed"
applies to data fields only.

**Verified:** `parseIdDigits()` rejects any non-digit byte before `strtoul` ever
runs, and all three forms plus a bare `#` are covered by tests.
