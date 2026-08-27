---
status: pending
---

# v6 reliability layer: unsequence the query verbs, and close three query-side gaps

## Description

Three live debugging sessions on vevov (radio ch4) produced one root
design change plus three follow-on defects. The semantics were settled
with the firmware session (`pxt-nezha-diffdrive-70`) across four
exchanges on 2026-08-27; that side is landed, tested (761 tests), and
hardware-verified. **This issue is this repo's half**: `protocol.md` is
the canonical spec, and this repo also carries both implementations —
the C++ handler (`src/protocol/protocol_handler.cpp`) and the Python
host (`src/host/robot_v6/`). So this is a doc + two-implementation
change, not a spec edit.

## The symptom

Eric's raw-relay session, ch4 after `!GO`:

    HELP            -> (nothing)
    ID              -> (nothing)
    ID #1           -> ack 1 0 none / id diffdrive vevov 1.0.10 vevov
    ID #1 (resent)  -> ack 1 0 none          <- ack, NO id line
    ID #2           -> ack 2 0 none / id diffdrive vevov 1.0.10 vevov

Three unrelated defects presenting identically, plus a fourth found
later (below) and a fifth that turned out to be the radio link.

## The rule this turns on

> A verb is sequenced iff its correctness depends on its position in the
> stream — either because executing it twice changes the robot, or
> because answering it out of order yields a wrong answer.

**Sequenced:** `GET SET TLM STOP RUN WHEELS_X WHEELS_V MOVE_X MOVE_V
GO_TO_R GO_TO_W`
**Unsequenced:** `HELLO PING ESTOP HELP ID VER STATUS`

`GET` stays sequenced and is **not** an exception to the rule — it is
order-dependent. A `GET` racing a pending `SET` returns the pre-`SET`
value with nothing marking it stale, which is a silently wrong answer to
a config question. Document it as covered by the rule, never as a
stakeholder-directed carve-out; the weaker framing invites someone to
relitigate it.

`ID`/`VER` answer session constants. Confirmed structurally on the
firmware side: `onSet`'s signature is `Result onSet(const char*, float,
uint32_t)` and every settable field is a float in the kernel config
table, while Identity fields are `const char*` — there is no
representable path by which `SET` mutates one.

## Why STATUS must be unsequenced — a latent spec bug, not a preference

§8.7 assigns `STATUS` the job of letting a desynced host resync via
`next=<n>` "without forcing a full `HELLO` reset", and usecases.md
UC-009 repeats it. That job is impossible today: a host that has lost
sequence tracking cannot pick an id. Guess low and it gets a stale
re-ack with no status line (exactly the capture above); guess high and
it opens a gap, stalling the stream it was trying to diagnose. The one
verb whose purpose is recovering from desync is gated behind not being
desynced.

## Blocker resolved in ordering: STATUS needs `done=`/`reason=` FIRST

`_maybe_poll_status` (reliability.py:296) sends a **sequenced** `STATUS`
specifically to provoke an ack carrying `(lastDone, reason)` — since
§8.5 deleted the telemetry piggyback, that pair only ever rides a direct
reply. Unsequencing `STATUS` removes the ack and kills completion
delivery silently. Fix is `status` gaining `done=` and `reason=` keys,
which also closes the gap §8.7 already flags as "not a considered
omission". Additive, and §6's unknown-key tolerance makes it backward
compatible. **Land the keys before unsequencing STATUS.**

## Stakeholder decisions (2026-08-27)

1. **Gap probing → resend the oldest pending command.** `_maybe_resend_from`
   (reliability.py:259) is the host's only retransmit path and fires on
   `nack` alone; nacks only answer sequenced lines. Unsequencing STATUS
   removes the poke that provoked them, leaving a quiet host with a lost
   command self-healing only on its next send — indistinguishable from a
   dead robot. Chosen fix retransmits the oldest still-pending command
   instead, using §8.1's re-ack row for what it exists for: no ids
   burned, no `_pending` growth per poll, nothing extra for a stalled
   robot to discard. The firmware repo's `robotlink.py` (`send_until()`)
   independently arrived at this shape and is the host that does not
   break when STATUS leaves the sequenced plane.
2. **`GET` with an unknown name emits `err 1 #<id>` alongside its ack.**
   Currently silent: `execGet` (protocol_handler.cpp:682) sets
   `errCode = 0` deliberately, discarding the `false` that
   `Adapter::onGet` (adapter.h:226) already returns. Asymmetric with
   `SET`, which errs on the same typo on the same config plane; and on a
   lossy link "ack, no get line" is ambiguous between a wrong name and a
   `get` line eaten in flight. The ack still fires (merits rejection,
   §8.2 — not a decode failure, sequence advances). Bare `GET #id` is
   untouched.
3. **Full scope** — spec, C++ handler, and Python host together.
4. **Unsequenced verbs may still TRIGGER an `ack`/`nack`** (refinement,
   2026-08-27). Verbatim: *"I don't actually mind if ID, VER, and help
   also return an ACK/NAK. What I mind is that they require an ACK/NAK
   ... What I don't want is requiring IDs to have a sequence number, or
   for help to have a sequence number and just be able to issue those
   any time, but they can also trigger an ACK/NAK."*

   Two separable things were collapsed into one. The objection is to
   **sequence gating** — a query required to carry an id, dropped or
   stale-acked without one. It is *not* to **reply emission**. A `HELP`
   issuable any time with no id should still be able to come back with
   "your last command didn't land."

   **Consistent with §8.5, not a reversal of it.** That direction
   objected to *periodicity* ("littered with 5 acks or nacks a second"),
   not to the line existing. An `ack` replying to an inbound unsequenced
   line is still a response to a message, and an idle connection stays
   silent. §8.5's rule sentence widens from "only in direct response to
   an inbound **sequenced** line" to "...an **inbound line**". The
   anti-beacon property is preserved exactly. We read reply-only as
   sequenced-only; it never said that.

   **DECIDED (stakeholder, 2026-08-27): conditional.** Emit from an
   unsequenced verb only when a gap or decode-failure stall is
   outstanding: silent on a clean stream, speaks up when something is
   wrong. That is the "reminder" framing, and it adds no second line per
   query on a link already at ~66%. The alternative (always append
   `ack <expectedNext_ - 1> ...`) needs no new state but is a receipt,
   not a reminder.

   **Cost, stated plainly: conditional emission brings `gapOutstanding_`
   back.** Deleted 2026-08-26 with §8.5 because its only reader was the
   telemetry piggyback. It cannot be derived — `expectedNext_` alone
   cannot distinguish "clean, waiting for #5 not yet sent" from
   "stalled, discarded #6, still want #5". So this partially reverses a
   stakeholder-directed deletion, and handler reliability state goes
   from one integer to one integer plus one bool. Still no clock, no
   timer, `feed()` still the only origin of every emission — §8.1's
   load-bearing properties untouched. The stakeholder was asked
   explicitly whether to re-add the bool, given that removing it was
   part of his own §8.5 direction the day before, and approved it.

   **Applies to:** `PING`, `HELP`, `ID`, `VER`, `STATUS`.
   **Excluded:** `ESTOP` (§8.3 — bare word `estop`, no fields ever, must
   never queue behind an outbound reply; the safety rule wins) and
   `HELLO` (resets `expectedNext_ = 1`, so nothing can be outstanding
   after it). Known accepted redundancy: `STATUS` already reports
   `next=`/`done=`/`reason=`, so a trailing nack repeats it — kept
   anyway, because "every unsequenced verb except ESTOP and HELLO
   carries the reminder" is a rule that fits in one's head.

## Arity posture — mandatory, not a detail

The four newly-unsequenced verbs must take the **PING posture**
(maximally forgiving), not the HELLO posture (strict zero-arity). Per
§9.8 item 7, a malformed *unsequenced* verb gets no reply at all — there
is no ack to anchor an `err` against. Strict posture would make `ID #1`
wrong-arity and answer it with silence, trading "dropped as stale" for
"dropped as malformed": same symptom, new cause, and it breaks every
existing host that appends an id out of habit.

## Scope

### Spec

1. **`docs/design/protocol.md`**
   - §2.2 — mandatory-id verb list.
   - §6 — verb table: `sequenced?` column for `HELP`/`ID`/`VER`/`STATUS`;
     `ID` reply gains a fourth field (below); `STATUS` reply gains
     `done=`/`reason=`; `GET` row's unknown-name behavior.
   - §8.3 — exemption set grows to seven; state the harm rule once,
     here, as the thing that governs the split; record the forgiving
     arity posture for the four new members; add the three-way probe
     split (**PING** = alive, **STATUS** = alive and where the sequence
     stands, **HELLO** = start over — HELLO resets `expectedNext_ = 1`
     and is not a health check).
   - §8.5 — "polls with any sequenced verb (e.g. `STATUS`...)" is now
     wrong; rewrite for unsequenced STATUS carrying `done=`/`reason=`,
     and document resend-oldest as the gap probe.
     Widen the rule sentence from "inbound **sequenced** line" to
     "inbound line" (decision 4), and restore `gapOutstanding_` to
     §8.1's state list with a dated note that its 2026-08-26
     deletion is partially reversed, and why. The reversal is scoped:
     the bool returns as a *reply* predicate only — it never restores a
     periodic or telemetry-carried emission, which stays deleted.
     State the STRUCTURAL INVARIANT first, ahead of the rule sentence:
     *every emission originates in `feed()`; the handler holds no clock
     and no periodic entry point, so periodicity is structurally
     impossible, not merely prohibited.* That is checkable by reading
     the call graph rather than by trusting a rule, and it is what makes
     re-adding `gapOutstanding_` safe — a predicate on a reply cannot
     become a beacon when nothing but inbound bytes can reach an
     emitter.
   - §8.7 — resync now structurally works; `done=`/`reason=` closes the
     flagged gap.
   - §9.8 items 6 and 7 — re-resolve under the new posture.
   - §8.0 — the stated design premise "measured loss on the radio link
     this protocol targets is real (~5%)" is contradicted by every
     measurement taken 2026-08-27 (delivery 66.5% / 75.0% / 83.3% across
     three runs on ch4, i.e. 17-33% loss, against 99.5% wired). Add a
     dated note: the reliability layer's own rationale currently cites a
     number the link has never been shown to deliver. This does not
     invalidate the design — the scheme works at these rates — but the
     premise should not stand unqualified.
   - New dated subsection recording the rationale, including
     profile-vs-identity (below) so neither is re-deleted as redundant.
2. **`docs/design/specification.md`** §3 — sequenced/unsequenced lists.
3. **`docs/design/usecases.md`** — UC-009 (resync), UC-010 (extend
   beyond PING: the unsequenced set answers while the stream is stalled).
3b. **`docs/design/overview.md`, `docs/design/motion-api.md`** — added to
   scope 2026-08-27. The withdrawn "~5%" premise is restated in both, so
   leaving them would keep the spec asserting in three places what §8.0
   now retracts. `overview.md`'s bullet is corrected outright.
   `motion-api#6`'s is **flagged, not rewritten**: it is a *safety*
   invariant sourced to a named bench test
   (`src/tests/bench/radio_move_reliability.py`) measuring lost *moves*,
   a different quantity from the lost *lines* measured in §8.0, so it is
   not simply superseded. But a move cannot survive a lost command line,
   and the two cannot both describe the same link — so it is marked
   "assume the higher rate until re-run". **The discrepancy runs in the
   unsafe direction**: an invariant under-stating loss tells an operator
   to be mildly careful when it should tell them to verify every move.
   Re-running that bench is follow-up work, not part of this issue.

### `ID`'s fourth field — accept the firmware extension

`id <drivetrain> <profile> <version> <name>`, with `name` **mandatory**.
Fields 0-2 are byte-identical, so the extension is strictly additive.
It exists because `profile` is build provenance baked at deploy time,
not identity: a fleet-wide incident had every robot reporting the same
profile, so `name` (sourced from the chip) is the wire's only
authoritative board identity. Optional would defeat it — a host would
fall back to `profile` exactly when `name` is absent, reproducing the
incident more rarely and less visibly.

`Identity.name` is already plumbed here (adapter.h:98) and already feeds
HELLO's banner; `execId` (protocol_handler.cpp:613) simply does not use
it. Conformance is a one-line `snprintf` change.

### Code

4. **`src/protocol/protocol_handler.cpp`**
   - Intercept `HELP`/`ID`/`VER`/`STATUS` by verb identity before id
     resolution, PING posture, `fieldCount = 0` regardless of what
     arrived. Per decision 4 these emit the reminder reliability line
     when a stall is outstanding — the id is not required, but the
     reply may still carry one.
   - Restore `gapOutstanding_`: set on a gap or decode-failure stall,
     cleared when the missing id finally arrives in order, and on
     `HELLO` (which resets the state it would report on).
   - The reminder MUST be the full three-field `nack <n> <lastDone>
     <reason>` (§6.1), read fresh off the adapter at format time. A
     two-field nack raises `IndexError` in `Session._ack_nack_fields`
     (reliability.py:246), which indexes `fields[0..2]` unconditionally
     — i.e. the reminder would crash the host session it exists to
     help, and only while the stream is already stalled.
   - `execId` — fourth field.
   - `execStatus` — `done=`/`reason=`, read fresh off
     `adapter_.lastDone()`/`lastDoneReason()` at format time, never
     cached (§8.8). Re-check the reply buffer width.
   - `execGet` — `errCode = 1` when `onGet` returns false for a named
     GET; bare GET unchanged.
5. **`src/host/robot_v6/reliability.py`**
   - `UNSEQUENCED_VERBS` → the seven.
   - Split `_maybe_poll_status` into completion polling (unsequenced
     STATUS, reading `done=`/`reason=`) and gap probing (resend oldest
     pending).
   - `wait_for_ack`'s documented passivity and the square-tour test
     choreography both need revisiting against the new probe.
6. **`src/host/robot_v6/codec.py`** — the unsequenced-verb comment at
   :101 and the reply-classification comment at :45.
7. **Tests** — arity forms across the cartesian product for each newly
   unsequenced verb, asserting byte-identical replies. **Not** "no
   ack/nack in these replies" — per decision 4, assert no reliability
   line on a CLEAN stream and the reminder line PRESENT when a gap is
   outstanding;
   STATUS key presence; GET unknown-name err; the resend-oldest path.

## Out of scope

- **A short `STATUS` form.** Radio shows a real length effect at the
  extremes (PING 78% vs STATUS 54%, p=0.011, n=50 each) but the wired
  control is 50/50 at 109 bytes — the firmware is not length-limited.
  Shortening the diagnostic to survive a link that measured healthy the
  same morning treats the symptom and would bake a permanent spec
  concession out of a possibly-transient RF problem. Measurement is in
  hand if the link question resolves the other way.
- **Restoring the deleted beacon.** §8.5 is stakeholder direction. A
  beacon-restored *scratch* build is proposed only as a diagnostic for
  the link investigation, never shipped.
- **The ch4 link investigation** (66.5% radio vs 99.5% wired). Separate
  work, firmware session leads. Ordering agreed cheapest-first: pin the
  relay unit (currently handed out by the torture pool — `gozop` on one
  run, `guvov` on another), then the beacon-restored scratch build, then
  environmental. Note the morning-vs-afternoon comparison straddles our
  own firmware change and used an instrument (keepalive counting) that
  no longer exists, so the magnitude of the apparent degradation is not
  established. **Update, later 2026-08-27: there is no established
  regression at all.** Both mechanical candidates are ruled out (relay
  unit: guvov 74.2% / zetog 75.8%, n=120 each; beacon deletion: tested
  without a flash by using `TLM POSE` as continuous robot TX — 83.3%
  quiet vs 76.7% busy, p=0.361, point estimate against the hypothesis).
  And the morning figures never showed what they were read as showing:
  8/8 has a 95% CI of [63.1%, 100%], 6/6 of [54.1%, 100%], and the
  morning's own STATUS figure (5/6 = 83.3%) is numerically identical to
  the best measurement taken tonight. Combined with 17 points of drift
  inside two hours (66.5% / 75.0% / 83.3%), the morning and evening data
  are indistinguishable. The question is not "what broke ch4 today" but
  "why has ch4 apparently always been at ~75%". Nothing in our sprint
  history is a suspect. The drift itself is now the phenomenon to
  characterise — a long interleaved run would give its period, and a
  period is the strongest available clue to what is duty-cycling.
  Sequential A/B on this link is worthless; interleave or run
  simultaneously.
- **B2** (nack the stale id) — rejected: it tells a host to resend what
  it just sent, and makes `nack <n>` ambiguous between "I need n" and "I
  already have n". **B3** (cached reply payload) — rejected: reintroduces
  the per-id storage §8.1 exists without. **B4** (re-execute read-only
  sequenced verbs on retransmit) — held in reserve, pre-agreed, if GET's
  stale case bites.

## Verification

- Full test suite here.
- Cross-repo conformance: the firmware session conforms to the revised
  `protocol.md` and has offered hardware re-verification on vevov via a
  Pi-based flasher. Repeat-count anything marginal — ch4's loss rate
  makes single exchanges uninformative, and n=12 cannot distinguish 50%
  from 75%.
- Interactive protocol work should use gauti's USB path (wired to vevov
  on the playfield, 99.5%) rather than the relay.
