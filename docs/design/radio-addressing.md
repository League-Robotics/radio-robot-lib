# Radio addressing — channel and group from the board's name

> How a micro:bit's radio channel and group come from its five-letter name: decode the name to a number, then take it modulo 73 for channels 11–83 and modulo 241 for groups 15–255.

<!-- meta: {"order":35,"tags":["radio","addressing"],"updated":"2026-09-13"} -->

**Status:** adopted 2026-09-13 by microbit-radio-relay 0.20260913.2 — the relay
firmware's `!N <name>` / `!N?` and `mbrelay`'s name registry compute this map, both
pass the D2 digest below, and it is deployed on the torture relay host. Not yet
adopted by pxt-nezha-diffdrive, whose
[`docs/radio-addressing.md`](https://github.com/League-Robotics/pxt-nezha-diffdrive/blob/master/docs/radio-addressing.md)
still specifies the previous mapping (`channel = 25 + 2*(n % 25)`), nor by this
repo's fleet configs. The name decoding is the same as before; only the step from
number to channel and group changes.

## Why change

The old mapping has only **25 channels** (odd, 25–73). Its (channel, group)
*pairs* never repeat, but that isn't what counts. `channel` is the actual radio
frequency (`uBit.radio.setFrequencyBand`, 2400 + ch MHz). `group` is only a filter
applied after a packet is received. Two robots with the same channel and different
groups can still tell their traffic apart, but they **share the air and split the
bandwidth**. With 25 channels that happens early: vevov and togov both have
channel 37 today.

This mapping uses every legal channel from 11 up: **73 channels** instead of 25.

## The map

A micro:bit's name is its nRF device ID modulo 3125, written in base 5. Decode
the name back to that number `n`, then reduce `n`:

```
consonants (letters 0, 2, 4)   z v g p t  = 0 1 2 3 4
vowels     (letters 1, 3)      u o i e a  = 0 1 2 3 4

normalize:  trim ASCII whitespace; map A-Z to a-z
accept:     ^[zvgpt][uoiea][zvgpt][uoiea][zvgpt]$     (raise on anything else)

n = 0
for p in 0..4:
    n = n * 5 + index_in_alphabet(name[p])       # n in 0..3124; name[0] is MOST significant

channel = 11 + (n % 73)                          # 11 .. 83
group   = 15 + (n % 241)                         # 15 .. 255
```

| | range | count | why this range |
|---|---|---|---|
| channel | 11–83 | 73 | 0–10 reserved (legacy hand-allocated 3/4/5, MakeCode default 7). 83 is the top of the 2.4 GHz ISM band (2483 MHz). |
| group | 15–255 | 241 | 0–14 reserved (MakeCode default 0, relay `!C` group 10). 255 is `setGroup`'s maximum. |

Every intermediate value is non-negative and at most 3124, so the arithmetic is
identical in MakeCode static TypeScript, C++ and Python. There are no negative
numbers and no need for unsigned types. Anyone who knows a board's name can work
out its address offline, with no registry.

### Reverse (address → name)

Used by `!N?` readback and diagnostics.

```
reject unless 11 <= channel <= 83 and 15 <= group <= 255
c = channel - 11                                  # 0 .. 72
g = group - 15                                    # 0 .. 240
n = c + 73 * (((g - c + 241) * 208) % 241)        # 208 = 73^-1 mod 241
reject unless n < 3125                            # most pairs belong to no name
for p = 4 down to 0:  name[p] = alphabet(p)[n % 5];  n = n / 5
```

`g - c + 241` is always positive (169 to 481), and the largest product is
481 × 208 = 100,048, so int32 is enough. Only 3,125 of the 17,593 possible pairs
correspond to a name; any other pair has no name and must be rejected.

## Properties

Checked by computer across all 3125 names.

- **3125 names → 3125 distinct (channel, group) pairs.** This holds because 73 and
  241 are coprime and 3125 < 73 × 241 (Chinese remainder theorem). If a range is
  ever changed, keep the two counts coprime and their product above 3125.
- **Channels are close to evenly used:** 43 names on each of channels 11–69 and 42
  on each of 70–83.
- **Groups likewise:** 13 names on each of groups 15–247 and 12 on each of 248–255.
- **Never emits channels 0–10 or groups 0–14**, so it stays clear of the legacy
  fleet convention, MakeCode's unconfigured default and the relay's `!C` space.
- A relay has no address of its own. It tunes to the robot it's serving.

### Conformance digests

Same method as the old spec: for `n = 0..3124` in order, emit one line per name,
UTF-8, and take the sha256 of the concatenation.

| digest | line format | sha256 |
|---|---|---|
| **D2 (the gate)** | `<name>,<channel>,<group>,<decode(name)>,<reverse(channel,group)>\n` | `305d6ee08cfae978fe13e1179c6047a56e1b0b1abe23c2cb757f01461cf2d35f` |
| D1 (forward only) | `<name>,<channel>,<group>\n` | `c22691f1c47bed3ac5317119487a30ea8fd0224d61c50bba551b1e624b548a84` |

D2 is the one to pass. D1 never calls `decode` or `reverse`, so it only helps narrow
down a failure: if D1 passes and D2 fails, the bug is in `decode` or `reverse`.

**Diagnostics for the common bug** (reading the name's letters in the wrong order):

| symptom | digest |
|---|---|
| D1 equals this → your encoder is little-endian | `3ce51760e5f7f692c86a83af02a87a537e0de318b94fe0c73259612e68a8867b` |
| D2 equals this → only your decoder is little-endian | `df3d1db0a2a8298ac6de1c118570e990221f0861b7b028e6f3947acb748ef1ba` |

To test by hand, use `zuzuv` or `zotuz`. Names that read the same backwards
(`zuzuz`, `tatat`, `zavaz`) can't catch the wrong-order bug.

## Vectors

The fleet:

| name | role | n | channel | group | old ch/grp |
|---|---|---|---|---|---|
| zeguz | robot | 425 | 71 | 199 | 25/19 |
| zetuv | robot | 476 | 49 | 250 | 27/21 |
| vevov | robot | 1031 | 20 | 82 | 37/43 |
| tovez | robot | 2665 | 48 | 29 | 55/108 |
| togov | robot (label only, never probed) | 2681 | 64 | 45 | 37/109 |
| vevav | robot | 1046 | 35 | 97 | 67/43 |
| gopiv | bench rig | 1461 | 12 | 30 | 47/60 |
| getez | relay | 1740 | 72 | 68 | 55/71 |
| zavaz | relay | 545 | 45 | 78 | 65/23 |

No two robots share a channel under this map. Under the old map, vevov and togov
share 37.

Edge cases:

| name | n | channel | group |
|---|---|---|---|
| zuzuz | 0 | 11 | 15 |
| zuzuv | 1 | 12 | 16 |
| zugag | 72 | 83 | 87 |
| zugap | 73 | 11 | 88 |
| zotuz | 225 | 17 | 240 |
| zotez | 240 | 32 | 255 |
| zotev | 241 | 33 | 15 |
| tatat | 3124 | 69 | 247 |

Reject: `gauti`, `vevo`, `vevovv`, `aeiou`, `""`, `TOVEZZ`. Accept as equal to
`vevov`: `VEVOV`, `" vevov "`. A pair with no name, such as channel 11 / group 16,
must be rejected by `reverse`.

## Known limitations

**Channels still collide, just less often.** Spreading 3125 names over 73
channels doesn't prevent collisions. The chance that at least two of N robots
share a channel:

| robots | old (25 ch) | new (73 ch) |
|---|---|---|
| 6 | 48% | 19% |
| 8 | 71% | 33% |
| 10 | 88% | 48% |
| 15 | ~100% | 79% |
| 20 | ~100% | 94% |

A shared channel costs bandwidth but not addressability, because pairs are
always unique. The robot's config (`connection.radio_channel` /
`radio_group`) stays authoritative in `make_deploy`, so when two boards do
collide, one can be moved by hand. The deploy tool should warn when a derived
channel is already in use in the registry.

**Adjacent channels are 1 MHz apart.** The radio runs Nordic 1 Mbit mode
(`RADIO_MODE_MODE_Nrf_1Mbit`), which takes up about 1 MHz. The old mapping used
only odd channels (2 MHz spacing, as BLE does). With consecutive channels, robots
one channel apart will leak some energy into each other's receivers. tovez (48)
and zetuv (49) are one apart. This is much milder than sharing a channel, but
it's more than zero.

**Wi-Fi overlap.** Channels 11–83 fall on top of Wi-Fi channels 1, 6 and 11. The
old range (25–73) overlapped 6 and 11 too.

**Two boards can share a name.** The name keeps only 3125 of the 2³² possible
device IDs, so two boards occasionally get the same name and therefore the same
address. The config override covers this case too.

## What has to change

| repo | change |
|---|---|
| pxt-nezha-diffdrive | `docs/radio-addressing.md` and the vectors file; `make_deploy.derive_radio_from_name()` and the "is this a name-derived channel" check (currently odd 25–73) |
| microbit-radio-relay | **Done** (0.20260913.2): `!N <name>` / `!N?` from `source/relay/naming.h`, `mbrelay.naming` and the registry's defaults; both checked against D2 |
| radio-robot-lib | fleet configs in `config/robots/*.json`; the `radio_channel` description in `robot_config.schema.json` (currently "odd values 25..73") |
