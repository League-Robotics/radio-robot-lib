"""tests/protocol/test_protocol_adversarial.py -- hostile-input hardening
for Protocol::ProtocolHandler (src/protocol/protocol_handler.{h,cpp}).

This is the "rock solid, and an archetype for other implementations" pass:
src/protocol/ is going to be read and re-implemented in MicroPython and
JavaScript by people who are not in this room, so a latent parser bug
here is a bug that propagates into every future implementation. Three
things this file checks that test_protocol_harness.py's tidy,
one-line-at-a-time tests do not:

1. **The parser must never crash, overflow, or misdispatch on hostile
   bytes.** Every test in this file runs the REAL protocol_handler.cpp
   compiled WITH AddressSanitizer and UndefinedBehaviorSanitizer
   (`-fsanitize=address,undefined -fno-omit-frame-pointer`) linked into
   a small standalone executable (asan_fuzz_driver.cpp) -- a crash,
   heap/stack overflow, use-after-free, or UB trap aborts the process
   with a nonzero exit code and a sanitizer report on stderr, which is
   surfaced directly in the assertion failure. "Never crashes" is
   demonstrated here, not asserted.

2. **The recovery invariant.** After ANY garbage whatsoever, a
   subsequent well-formed line must still dispatch correctly -- a
   handler that wedges after one bad frame is useless on a lossy radio
   link. Every adversarial case below is followed by an explicit line
   terminator (flushing any pending partial/overflowing line) and then
   `HELLO\n`, and the test asserts the reply (the boot banner) is the
   last thing the sink produced. **Uses `HELLO`, not `PING`, since the
   2026-08-21 reliability layer (docs/design/protocol.md S8) made `PING`
   a SEQUENCED verb requiring a mandatory `#<id>` matching whatever
   `expectedNext_` happens to be at that point in the session --
   `HELLO` is the one verb whose recovery-check behavior is
   STATE-INDEPENDENT (it is unsequenced and unconditionally resets the
   sequence to a known value), so it is the only choice that lets this
   file's per-case sweep assert a single fixed expected output without
   tracking each case's own effect on `expectedNext_`.** (Sending the
   recovery command WITHOUT first flushing the garbage line would be
   testing a different, stricter property -- whether a well-formed
   command survives being concatenated directly onto an unterminated
   garbage prefix, which spec S2's line grammar never promises -- so
   this file always closes the garbage line first, the way a real next
   line from a real host would arrive.)

3. **Grammar migration, 2026-08-20 (spec commit 5a5b6da).** This file
   was rewritten from a colon-delimited, positional-id grammar to the
   space/`#id` grammar (protocol_handler.h's own file header has the
   full resolution history). Every adversarial case below was
   translated to the new separator, and several are NEW -- specific to
   hazards the space grammar introduces that the colon grammar never
   had (a self-marking `#id` token, whitespace bytes other than ' '
   remaining legal-but-hazardous field content, huge space runs, a
   blank/all-whitespace line as its own first-class case). See each
   case's own comment for which.

4. **Three genuine parser bugs found during the original (colon-era)
   hardening sweep, all still fixed in protocol_handler.cpp, carried
   forward through the grammar migration** (see that file's own
   comments at the fix site for the full story) -- this file's
   test_hex_float_no_longer_bypasses_no_exponents_rule and
   test_leading_whitespace_no_longer_silently_accepted are their
   regression tests:

   - `SET name 0x1.8p3` (C99 hex-float syntax) used to be silently
     ACCEPTED as 12.0, bypassing the "no exponents" rule (spec S2.2)
     entirely, because the exponent check only looked for 'e'/'E', not
     a hex float's 'x'/'p'. Neither Python's `float()` nor
     JavaScript's `Number()`/`parseFloat()` accepts this syntax, so
     this was a C++-only divergence a straight port would NOT
     reproduce -- an archetype-relevant finding on its own. Entirely a
     property of strtof() parsing an already-extracted field's own
     content, so unaffected by the colon-to-space migration.
   - A leading-whitespace numeric field used to be silently ACCEPTED,
     because strtol/strtoul/strtof all skip leading whitespace per the
     C standard -- contradicting this file's own "strict, whole field
     consumed" doc comment, which only actually held for TRAILING
     whitespace. Under the OLD colon grammar this was reachable via a
     literal leading SPACE right after a ':' separator (e.g.
     "WHEELS: 100:100:1000"). Under the NEW space grammar that exact
     shape is now STRUCTURALLY IMPOSSIBLE: the tokenizer collapses
     every run of ' ' into one separator, so a token can never begin
     with ' ' -- but the guard is NOT dead code, because spec S2's
     field grammar (`field ::= any bytes except ' ' and '\n'`) still
     allows '\t', '\v', '\f', and '\r' as ordinary, legal field bytes,
     and strtol/strtoul/strtof would silently skip any of THOSE too.
     This file's regression tests were rewritten to use a leading TAB
     (or another non-space whitespace byte) instead of a leading space,
     since that is the hazard that actually survives the migration --
     see protocol_handler.cpp's isWireSpace() comment for the same
     reachability analysis at the fix site.

   A third bug (formatConfigValue() casting a NaN adapter value to
   uint32_t -- real undefined behavior, confirmed live by UBSan) is
   regression-tested in nan_regression_driver.cpp /
   test_nan_inf_get_reply_formatting_is_safe below; it is reachable
   only through the Adapter seam (a NaN can never arrive over the wire
   -- parseFloatField already rejects it on input), not through feed()
   directly, so it gets its own tiny driver rather than living in the
   generic fuzz list. Pure value formatting, unaffected by the grammar
   migration.

Run with::

    uv run python -m pytest tests/protocol/test_protocol_adversarial.py -v -s
"""

import pathlib
import random
import struct
import subprocess

import pytest

# tests/protocol/test_protocol_adversarial.py -> protocol -> tests -> root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
_PACKAGE_DIR = _REPO_ROOT / "src" / "protocol"
_TEST_DIR = pathlib.Path(__file__).resolve().parent

_SANITIZE_FLAGS = [
    "-fsanitize=address,undefined",
    "-fno-omit-frame-pointer",
    "-g",
]

# UBSan reports a diagnostic on stderr and, by default, keeps going;
# ASan aborts on the first error unconditionally. halt_on_error=1 makes
# a UBSan finding fail the process the same way an ASan finding
# already does, so "returncode == 0" is a complete, uniform pass/fail
# signal for both sanitizers, not just one of them. detect_leaks=0:
# leak-checking is a different question from "does feed() corrupt
# memory or dispatch UB on hostile bytes", and MockAdapter/RecordingSink
# style test doubles are not written to be leak-clean under a
# process-exit-time leak scan.
_SANITIZER_ENV_EXTRA = {
    "UBSAN_OPTIONS": "halt_on_error=1",
    "ASAN_OPTIONS": "detect_leaks=0",
}


def _compile_asan_executable(tmp_path_factory, sources, out_name):
    build_dir = tmp_path_factory.mktemp("protocol_adversarial_build")
    exe_path = build_dir / out_name
    cmd = (
        ["/usr/bin/c++", "-std=c++20", "-Wall", "-Wextra"]
        + _SANITIZE_FLAGS
        + ["-I", str(_PACKAGE_DIR), "-I", str(_TEST_DIR)]
        + [str(s) for s in sources]
        + ["-o", str(exe_path)]
    )
    result = subprocess.run(cmd, capture_output=True, text=True)
    assert result.returncode == 0, (
        f"sanitizer build failed:\ncmd: {' '.join(cmd)}\n"
        f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}")
    return exe_path


@pytest.fixture(scope="session")
def fuzz_driver(tmp_path_factory):
    """The generic adversarial-input driver (asan_fuzz_driver.cpp): a
    fresh MockAdapter + RecordingSink-equivalent ProtocolHandler per
    process, fed a sequence of length-prefixed byte records from
    stdin, sink output on stdout."""
    return _compile_asan_executable(
        tmp_path_factory,
        [_PACKAGE_DIR / "protocol_handler.cpp",
         _TEST_DIR / "asan_fuzz_driver.cpp"],
        "asan_fuzz_driver")


@pytest.fixture(scope="session")
def nan_driver(tmp_path_factory):
    """The NaN/Inf/long-name GET reply-formatting regression driver
    (nan_regression_driver.cpp) -- see its own header comment."""
    return _compile_asan_executable(
        tmp_path_factory,
        [_PACKAGE_DIR / "protocol_handler.cpp",
         _TEST_DIR / "nan_regression_driver.cpp"],
        "nan_regression_driver")


@pytest.fixture(scope="session")
def run_debug_driver(tmp_path_factory):
    """sendDebug() sanitization + RUN's #0-suppresses-a-REGISTERED-ret
    driver (run_debug_driver.cpp) -- see its own header comment."""
    return _compile_asan_executable(
        tmp_path_factory,
        [_PACKAGE_DIR / "protocol_handler.cpp",
         _TEST_DIR / "run_debug_driver.cpp"],
        "run_debug_driver")


def _encode_records(chunks):
    """The fuzz driver's stdin protocol: repeated
    (uint32_t little-endian length, raw bytes) records."""
    out = bytearray()
    for chunk in chunks:
        out += struct.pack("<I", len(chunk))
        out += chunk
    return bytes(out)


def _run(exe_path, chunks, timeout=10):
    stdin_bytes = _encode_records(chunks)
    env = dict(__import__("os").environ)
    env.update(_SANITIZER_ENV_EXTRA)
    return subprocess.run(
        [str(exe_path)], input=stdin_bytes, capture_output=True,
        timeout=timeout, env=env)


def _assert_survived(result, context):
    assert result.returncode == 0, (
        f"{context}: process did not exit cleanly (sanitizer trap or "
        f"crash) -- returncode={result.returncode}\n"
        f"stdout: {result.stdout!r}\nstderr:\n"
        f"{result.stderr.decode('utf-8', 'replace')}")


# MockAdapter's default identityToReturn (mock_adapter.h) is every field
# defaulted to "" -- asan_fuzz_driver.cpp never calls phSetIdentity's
# equivalent, so a HELLO's banner on that driver is always this exact,
# deterministic line: "device NEZHA2 robot " + "" + " " + "" + "\n".
# Used as the recovery invariant's fixed expected tail (see this module's
# own docstring point 2 for why HELLO, not PING, is the recovery probe).
_HELLO_BANNER = b"device NEZHA2 robot  \n"


# ---------------------------------------------------------------------------
# Adversarial cases (spec grammar: line ::= verb (' ' field)* '\n', a run
# of spaces is ONE separator, id is a trailing '#'-prefixed field, verb
# case is direction, max line 240 bytes including the terminator). Each
# entry is (name, [chunk, ...]) -- one or more feed() calls' worth of
# raw bytes, deliberately NOT closed with a clean terminator in some
# cases (the "unterminated" ones), to exercise feed()'s cross-call
# buffering contract as well as single-call parsing.
# ---------------------------------------------------------------------------

ADVERSARIAL_CASES = [
    # ---- embedded NUL bytes mid-line ----
    ("embedded_nul_mid_verb", [b"PI\x00NG\n"]),
    ("embedded_nul_after_verb", [b"PING\x00extra\n"]),
    ("embedded_nul_in_set_name", [b"SET foo\x00bar 1.0\n"]),
    ("embedded_nul_in_set_value", [b"SET group.alpha 1\x002 #9\n"]),
    ("embedded_nul_in_wheels_field", [b"WHEELS_V 1\x0000 100 1000\n"]),
    ("embedded_nul_in_get_name", [b"GET foo\x00bar\n"]),
    ("embedded_nul_in_id", [b"STOP #1\x002\n"]),

    # ---- 8-bit / high-ASCII and UTF-8 sequences ----
    ("high_ascii_full_line", [bytes(range(0x80, 0x100)) + b"\n"]),
    ("high_ascii_verb", [bytes([0xC0, 0xC1, 0xFE, 0xFF]) + b"\n"]),
    ("utf8_verb", ["日本語".encode("utf-8") + b"\n"]),
    ("utf8_in_set_value",
     [b"SET " + "日本語".encode("utf-8") + b" 1.0\n"]),
    ("utf8_in_get_name",
     [b"GET " + "éèê\U0001F600".encode("utf-8") + b"\n"]),

    # ---- other control characters ----
    ("c0_control_chars_full_line",
     [bytes(b for b in range(1, 32) if b not in (0x0A,)) + b"\n"]),
    ("del_byte_full_line", [b"\x7f\n"]),
    ("del_byte_in_set_value", [b"SET group.alpha 1\x7f0\n"]),
    ("bell_and_escape_in_verb", [b"P\x07I\x1bNG\n"]),

    # ---- very long runs of '#' / hash-only lines / trailing hashes --
    # the new grammar's own special byte, replacing the old colon-flood
    # cases (a run of ':' meant nothing special under the space grammar,
    # so those cases are retired in favor of the byte that now IS
    # special: '#', the id marker) ----
    ("very_long_hash_run", [b"#" * 300 + b"\n"]),
    ("line_only_hashes_short", [b"###\n"]),
    ("line_only_hashes_long", [b"#" * 238 + b"\n"]),
    ("verb_directly_followed_by_hashes_no_space",
     [b"PING" + b"#" * 50 + b"\n"]),
    ("known_verb_directly_followed_by_hashes_no_space",
     [b"STOP" + b"#" * 100 + b"\n"]),
    ("known_verb_space_then_long_non_digit_hash_field",
     [b"STOP " + b"#" * 100 + b"\n"]),
    ("bare_hash_as_id_no_digits", [b"STOP #\n"]),
    ("hash_then_non_digit", [b"STOP #x\n"]),
    ("hash_with_leading_plus", [b"STOP #+5\n"]),
    ("hash_with_leading_minus", [b"STOP #-5\n"]),
    ("multiple_hash_tokens_last_one_wins",
     [b"STOP #5 #7\n"]),  # wrong arity (2 fields); #7 IS the recoverable
                           # last token even though #5 looks id-shaped too
    ("huge_digit_run_after_hash_overflows_uint32",
     [b"STOP #" + b"9" * 300 + b"\n"]),

    # ---- space-run stress: the new grammar's own separator, hammered --
    ("huge_space_run_between_fields",
     [b"WHEELS_V 100" + b" " * 200 + b"100 1000\n"]),
    ("many_spaces_then_nothing_is_blank",
     [b" " * 239 + b"\n"]),  # all-whitespace line, near the 240-byte cap
    ("verb_alone_no_trailing_content", [b"WHEELS_V\n"]),
    ("verb_then_trailing_spaces_only", [b"WHEELS_V" + b" " * 50 + b"\n"]),
    ("stop_alone_no_id", [b"STOP\n"]),
    ("stop_then_trailing_spaces_only", [b"STOP" + b" " * 50 + b"\n"]),

    # ---- empty lines / blank-line runs (spec S2: silently ignored, NOT
    # malformed -- this changed from the colon grammar, where an empty
    # line dispatched as an unknown zero-length verb) ----
    ("empty_line", [b"\n"]),
    ("three_empty_lines", [b"\n\n\n"]),
    ("many_empty_lines", [b"\n" * 20]),
    ("mixed_blank_and_whitespace_lines", [b"\n   \n\t\n \n"]),

    # ---- \r handling: lone \r, \r\n, \n\r ----
    ("crlf", [b"\r\n"]),
    ("lfcr", [b"\n\r"]),
    ("cr_mid_field_not_at_terminator", [b"WHEELS_V \r100 100 1000\n"]),
    ("multiple_lone_cr_mid_line", [b"PING\r\r\r\n"]),

    # ---- lines at 239 / 240 / 241 bytes, and further over ----
    ("line_content_238_total_239_under_cap", [b"Z" * 238 + b"\n"]),
    ("line_content_239_total_240_exact_cap", [b"Z" * 239 + b"\n"]),
    ("line_content_240_total_241_over_cap", [b"Z" * 240 + b"\n"]),
    ("line_content_1000_way_over_cap", [b"Z" * 1000 + b"\n"]),

    # ---- unterminated: partial lines, huge no-terminator blobs,
    # spread across MULTIPLE feed() calls ----
    ("unterminated_short_fragment", [b"WHEELS_V 100 100"]),
    ("unterminated_lone_cr", [b"\r"]),
    ("unterminated_4kb_single_call", [b"A" * 4096]),
    ("unterminated_plausible_prefix_then_huge_continuation",
     [b"WHEELS_V 100 100 1000", b"B" * 5000]),
    ("unterminated_split_across_many_small_calls",
     [b"W", b"H", b"E", b"E", b"L", b"S", b" ", b"1" * 300]),

    # ---- mixed-case / case-as-direction edge cases (spec S2.1) ----
    ("all_lowercase_verb_dropped", [b"ping\n"]),
    ("mixed_case_verb_unknown", [b"Wheels 100 100 1000\n"]),
    ("lowercase_verb_with_spaces_and_high_bytes",
     [b"dbg " + bytes(range(0x80, 0x90)) + b"\n"]),

    # ---- numeric-field adversarial spellings -- ids added (#1, a fresh
    # handle's own first in-order id) so these actually reach the
    # field-decode logic under ASan/UBSan, rather than bailing out
    # earlier on "no id at all" (docs/design/protocol.md S8) ----
    ("wheels_field_all_pluses", [b"WHEELS_V +100 +100 +1000 #1\n"]),
    ("wheels_field_leading_zeros", [b"WHEELS_V 000100 00100 0001000 #1\n"]),
    ("set_value_only_a_sign", [b"SET group.alpha - #1\n"]),
    ("set_value_only_a_dot", [b"SET group.alpha . #1\n"]),
    ("set_value_many_dots", [b"SET group.alpha 1.2.3.4 #1\n"]),
    ("wheels_duration_huge_digit_run",
     [b"WHEELS_V 100 100 " + b"9" * 40 + b" #1\n"]),

    # ---- RUN: open arity, adversarial argument counts/content (spec
    # grammar's RUN section -- the handler only parses, so these mostly
    # exercise the fieldCount/kMaxFieldTokens/kMaxRunArgs bound checks
    # handleRun() adds on top of the generic tokenizer, not anything
    # verb-specific to WHEELS/SET). Ids added (#1) so these reach
    # handleRun() itself under ASan/UBSan rather than bailing out on
    # "no id at all" first -- RUN's id is mandatory now (docs/design/
    # protocol.md S6.3/S8) ----
    ("run_zero_args", [b"RUN blink #1\n"]),
    ("run_many_args_within_kmaxrunargs",
     [b"RUN foo " + b" ".join(str(i).encode() for i in range(16)) +
      b" #1\n"]),
    ("run_args_over_kmaxrunargs",
     [b"RUN foo " + b" ".join(str(i).encode() for i in range(17)) +
      b" #1\n"]),
    ("run_args_over_kmaxfieldtokens",
     # MORE real tokens than kMaxFieldTokens (20) can hold pointers for
     # at all -- handleRun() must reject this BEFORE indexing fields[]
     # anywhere near that boundary, not just before kMaxRunArgs.
     [b"RUN foo " + b" ".join(str(i).encode() for i in range(40)) +
      b" #1\n"]),
    ("run_single_arg_near_line_length_cap",
     [b"RUN foo " + b"x" * 225 + b" #1\n"]),
    ("run_nonascii_arg",
     [b"RUN foo " + "héllo wôrld".encode("utf-8").replace(b" ", b"_") +
      b" #1\n"]),
    ("run_hash_prefixed_non_digit_last_arg", [b"RUN foo #abc\n"]),
    ("run_only_hash_id_no_function_name", [b"RUN #1\n"]),
    ("run_bare_no_function_name", [b"RUN\n"]),
    ("run_trailing_spaces_only", [b"RUN" + b" " * 50 + b"\n"]),

    # ---- non-space whitespace bytes as a field's LEADING byte -- the
    # hazard that survives the grammar migration (a literal leading ' '
    # is now structurally impossible; '\t'/'\v'/'\f'/'\r' remain legal,
    # ordinary field bytes per spec S2's field grammar, and are exactly
    # what isWireSpace() in protocol_handler.cpp still guards against) --
    ("tab_leading_wheels_field", [b"WHEELS_V \t100 100 1000\n"]),
    ("vtab_leading_set_value", [b"SET group.alpha \v1.0\n"]),
    ("formfeed_leading_wheels_duration", [b"WHEELS_V 100 100 \f1000\n"]),
    ("cr_leading_set_value_not_at_terminator",
     [b"SET group.alpha \r1.0\n"]),
]


def _adversarial_ids():
    return [name for name, _chunks in ADVERSARIAL_CASES]


@pytest.mark.parametrize("name,chunks", ADVERSARIAL_CASES, ids=_adversarial_ids())
def test_recovers_after_adversarial_input(fuzz_driver, name, chunks):
    """The recovery invariant (docs/design/protocol.md S2.1 / spec S2):
    however hostile `chunks` is, the parser must (a) never crash,
    overflow, or trap under ASan/UBSan, and (b) still dispatch a clean
    HELLO correctly once the garbage line is closed out. A handler that
    wedges after one bad frame is useless on a lossy radio link."""
    # A bare '\n' first closes out whatever partial/overflowing line the
    # adversarial chunks left pending, so HELLO arrives as its own clean
    # line -- see this module's docstring point 2 for why that is the
    # fair way to phrase "a subsequent well-formed line", not a way to
    # dodge the harder case, and why HELLO rather than PING.
    result = _run(fuzz_driver, list(chunks) + [b"\n", b"HELLO\n"])
    _assert_survived(result, f"case {name!r}")
    assert result.stdout.endswith(_HELLO_BANNER), (
        f"case {name!r}: HELLO after the garbage did not produce the "
        f"expected banner -- handler did not recover\n"
        f"stdout: {result.stdout!r}")


def test_recovers_after_every_adversarial_input_in_one_session(fuzz_driver):
    """Companion to the per-case sweep above: ALL adversarial cases,
    back-to-back, on ONE handler instance in ONE process (rather than a
    fresh handler per case) -- the failure mode a per-case sweep cannot
    see is state leaking or accumulating badly enough across many bad
    lines that some LATER, unrelated line misdispatches. A HELLO is
    interleaved after every case; every one must come back clean, and
    (being unsequenced and state-resetting) each one's expected output
    is the SAME fixed banner regardless of anything any earlier case did
    to expectedNext_/gapOutstanding_ -- the property a sequenced PING
    could not offer here (see this module's own docstring point 2)."""
    chunks = []
    for _name, case_chunks in ADVERSARIAL_CASES:
        chunks.extend(case_chunks)
        chunks.append(b"\n")
        chunks.append(b"HELLO\n")
    result = _run(fuzz_driver, chunks, timeout=30)
    _assert_survived(result, "combined adversarial session")
    banner_count = result.stdout.count(_HELLO_BANNER)
    expected = len(ADVERSARIAL_CASES)
    assert banner_count == expected, (
        f"expected {expected} banner replies (one recovery HELLO per "
        f"adversarial case), got {banner_count} -- a HELLO somewhere in "
        f"the session did not come back, so state from an earlier case "
        f"corrupted a later one\nfull stdout: {result.stdout!r}")


def test_random_byte_fuzz_survives_and_recovers(fuzz_driver):
    """Broad-spectrum fuzzing beyond the hand-picked cases above:
    uniformly random byte strings (any value 0-255, including '\\n',
    ' ', and '#' at random positions, so a single blob can contain
    several "lines" of pure noise, and can spuriously look like a
    well-formed id), FIXED seed so a failure reproduces exactly. Each
    trial is followed by the same flush + HELLO recovery check."""
    rng = random.Random(20260820)
    trial_count = 40
    for trial in range(trial_count):
        length = rng.randint(0, 500)
        blob = bytes(rng.randrange(256) for _ in range(length))
        result = _run(fuzz_driver, [blob, b"\n", b"HELLO\n"])
        _assert_survived(result, f"random fuzz trial {trial} (len={length})")
        assert result.stdout.endswith(_HELLO_BANNER), (
            f"random fuzz trial {trial} (len={length}): did not recover\n"
            f"blob: {blob!r}\nstdout: {result.stdout!r}")


def test_embedded_nul_immediately_after_verb_matches_bare_verb(fuzz_driver):
    """NOT a crash, and not something either hardening pass fixes -- a
    characterization test that PINS a real divergence for the
    "archetype" question, so it cannot silently change later without a
    test noticing. Re-verified (not just carried forward) after the
    2026-08-20 grammar migration: the ROOT CAUSE is the same, but the
    MECHANISM shifted, so this needed an actual re-check, not a blind
    port.

    Under the OLD colon grammar, the truncation happened via strchr()
    (find the verb-terminating ':') and strcmp() both stopping at the
    first NUL in a C string. Under the NEW space grammar there is no
    strchr() at all -- but tokenizeLine()'s own forward scan
    (`while (*p != '\\0' && *p != ' ') ++p;`, protocol_handler.cpp) is
    itself written against NUL-terminated C strings, per this file's
    own documented "no allocation, no std::string" constraint (spec
    S2.2), so it ALSO stops at the first embedded NUL and treats it
    exactly like the true end of the line -- `PING\x00extra #5\n`
    dispatches exactly like a BARE `PING\n` with nothing after it at
    all, silently discarding "extra #5" (and anything else up to the
    real '\n') with no malformed-count increment and no sign anything
    was dropped.

    **Re-verified a third time (2026-08-22) after PING joined the
    unsequenced exemption set (docs/design/protocol.md §8.3):** a bare
    `PING` with no id now answers `pong` unconditionally (it never
    needed one in the first place, as of this change) -- so the
    observable consequence of the C-string truncation flipped AGAIN,
    from "no reply at all" (2026-08-21's mandatory-id era) to "answers
    exactly like a bare PING would" (now). The root cause -- `strlen()`
    stopping at the embedded NUL four bytes in, so "extra #5" is never
    seen at all -- is unchanged across all three eras; only what a
    truncated, id-less PING DOES with that has moved.

    Spec S2's verb grammar (`verb ::= [A-Za-z][A-Za-z0-9_]*`) does not
    admit NUL in a verb at all, so the grammar-correct behavior would be
    to REJECT this line as unparseable -- this C-string-based
    implementation instead silently accepts it as the shorter verb.

    This is exactly the kind of thing a MicroPython or JavaScript port
    would NOT reproduce: `bytes`/`str` equality in Python, and string
    equality in JavaScript, compare the FULL length, embedded NUL bytes
    included -- `b"PING\x00extra #5" == b"PING"` is `False`. A port that
    otherwise faithfully mirrors this C++ handler's logic would treat
    this exact input as an unknown verb (or a malformed one) carrying a
    perfectly good id, not as a bare PING, and that divergence would
    only surface as a conformance-suite mismatch on inputs no one
    thought to write down -- which is why it is written down here."""
    result = _run(fuzz_driver, [b"PING\x00extra #5\n"])
    _assert_survived(result, "embedded NUL immediately after a verb name")
    assert result.stdout == b"pong 0\n", (
        f"expected this to (surprisingly) dispatch as a BARE PING and "
        f"therefore reply 'pong 0' (the fuzz driver's MockAdapter never "
        f"calls phSetNow, so now() defaults to 0), got: {result.stdout!r} "
        f"-- if this now differs, the C-string dispatch behavior changed "
        f"and this test needs updating")


# ---------------------------------------------------------------------------
# Regression tests for the bugs this sweep found (protocol_handler.cpp's
# own comments at each fix site have the full story).
# ---------------------------------------------------------------------------

def test_hex_float_no_longer_bypasses_no_exponents_rule(fuzz_driver):
    """`SET name 0x1.8p3` is C99 hex-float syntax -- strtof() parses it
    to 12.0 unconditionally, and the old exponent guard only checked
    for 'e'/'E', never 'x'/'X', so this slipped through as if it were
    an ordinary decimal literal. Must now be rejected as malformed
    (err code 2, ERR_BADARG), the same as any other unparseable value,
    and must NOT have reached the adapter's onSet(). This is a HANDLER-
    level decode failure (the value field never parses), so as of
    2026-08-22 (docs/design/protocol.md S8.9, "decode failure is a
    NAK") it NACKS and does NOT advance the sequence -- it does not ack
    then err, the pre-2026-08-22 shape."""
    result = _run(fuzz_driver, [b"SET group.alpha 0x1.8p3 #1\n"])
    _assert_survived(result, "hex float SET")
    assert result.stdout == b"nack 1 0 none\nerr 2 #1\n", (
        f"hex-float value was not rejected: {result.stdout!r}")


@pytest.mark.parametrize("spelling", [
    b"0x1p0", b"0X1P1", b"0x1.8p3", b"0xAp0", b"0x0p0",
])
def test_hex_float_rejected_for_several_spellings(fuzz_driver, spelling):
    result = _run(fuzz_driver, [b"SET group.alpha " + spelling + b" #1\n"])
    _assert_survived(result, f"hex float spelling {spelling!r}")
    assert result.stdout == b"nack 1 0 none\nerr 2 #1\n", (
        f"hex-float spelling {spelling!r} was not rejected: "
        f"{result.stdout!r}")


def test_leading_whitespace_no_longer_silently_accepted(fuzz_driver):
    """A leading TAB in a numeric field used to sail through
    strtol/strtoul/strtof, which all skip leading whitespace per the C
    standard -- silently accepting "WHEELS_V \\t100 100 1000" as
    left=100. Must now be rejected as malformed (ERR_BADARG), matching
    this file's own "strict, whole field consumed" contract -- and, as
    of 2026-08-22, this is a NACK (a handler-level decode failure), not
    an ack-then-err.

    This is deliberately a TAB, not a leading SPACE: under the space
    grammar a leading space can never reach a field decoder at all (the
    tokenizer consumes it as the separator itself), so a test using a
    literal space here would not exercise this guard -- see this
    module's own docstring point 4 and protocol_handler.cpp's
    isWireSpace() comment for the full reachability analysis."""
    result = _run(fuzz_driver, [b"WHEELS_V \t100 100 1000 #1\n"])
    _assert_survived(result, "leading-tab WHEELS_V field")
    assert result.stdout == b"nack 1 0 none\nerr 2 #1\n", (
        f"leading-whitespace numeric field was not rejected: "
        f"{result.stdout!r}")


@pytest.mark.parametrize("line", [
    b"WHEELS_V 100 \v100 1000 #1\n",
    b"SET group.alpha \f1.0 #1\n",
    b"WHEELS_V 100 100 \r1000 #1\n",
])
def test_leading_whitespace_rejected_across_verbs(fuzz_driver, line):
    result = _run(fuzz_driver, [line])
    _assert_survived(result, f"leading-whitespace line {line!r}")
    assert result.stdout == b"nack 1 0 none\nerr 2 #1\n", (
        f"leading-whitespace field in {line!r} was not rejected: "
        f"{result.stdout!r}")


def test_stop_id_with_non_digit_byte_is_malformed_no_reply(fuzz_driver):
    """Every sequenced verb's id is decoded by a DIFFERENT, stricter
    parser than WHEELS/SET's ordinary numeric fields (parseIdDigits() in
    protocol_handler.cpp): the id grammar is exactly `'#' [0-9]+`, so
    this file pre-scans every byte after the '#' and rejects on the
    FIRST non-digit -- it never calls strtoul() at all for a string that
    fails that scan, so the "leading whitespace" hazard the tests above
    target does not even apply to ids the same way (a stray tab right
    after '#' is rejected by the manual digit-only pre-check, not
    because strtoul happened to skip it). This test pins that a
    whitespace byte in the id position is still correctly rejected --
    STOP's ONLY field is now its mandatory id (docs/design/protocol.md
    S8), so a malformed id means the whole line cannot be sequence-
    classified at all: NO reply, same as any other sequenced verb whose
    trailing token fails to parse as `#[0-9]+`."""
    result = _run(fuzz_driver, [b"STOP #\t5\n"])
    _assert_survived(result, "whitespace byte inside STOP's id")
    assert result.stdout == b"", (
        f"STOP with a non-digit byte in its id must produce NO reply "
        f"(no id can be trusted to sequence-classify against), got: "
        f"{result.stdout!r}")


def test_nan_inf_get_reply_formatting_is_safe(nan_driver):
    """formatConfigValue() (protocol_handler.cpp) used to cast a NaN
    value straight to uint32_t -- real undefined behavior, confirmed by
    UBSan pre-fix (converting a NaN to an unsigned integer type has no
    defined result in C++). A NaN can never arrive OVER THE WIRE
    (parseFloatField rejects it on input, spec S2.2/S7.2's "no NaN, no
    inf"), so this can only be reached through the Adapter seam -- an
    adapter's own stored config value being NaN (e.g. a divide-by-zero
    upstream in a real firmware's config math) -- which is exactly what
    nan_regression_driver.cpp does: set MockAdapter's GET override to
    NaN/+Inf/-Inf directly, then GET it over the wire.

    Also re-checks the historical GET reply-buffer bug (a near-cap-length
    field name, per spec S2's 240-byte line cap) on the same buffer,
    non-regression only. Pure value-formatting logic, unaffected by the
    colon-to-space grammar migration -- only the wire syntax the driver
    feeds ("GET <name>" instead of "GET:<name>") changed. GET is
    sequenced now (docs/design/protocol.md S8), so each of the four
    calls carries its own mandatory in-order id and is acked before its
    own `get` line -- which is also why the long name shrank from 235 to
    232 bytes (nan_regression_driver.cpp's own comment): the mandatory
    " #4" suffix has to fit the same 240-byte line together with it."""
    result = subprocess.run(
        [str(nan_driver)], capture_output=True, timeout=10,
        env={**__import__("os").environ, **_SANITIZER_ENV_EXTRA})
    _assert_survived(result, "NaN/Inf/long-name GET regression driver")
    lines = result.stdout.splitlines()
    assert lines == [
        b"ack 1 0 none", b"get nan.field 0.000000",
        b"ack 2 0 none", b"get posinf.field 4294.967040",
        b"ack 3 0 none", b"get neginf.field -4294.967040",
        b"ack 4 0 none", b"get " + b"n" * 232 + b" 1.500000",
    ], f"unexpected output: {result.stdout!r}"


def test_run_debug_driver_survives_and_matches(run_debug_driver):
    """Runs run_debug_driver.cpp under ASan/UBSan and asserts its exact
    output sequence -- both sendDebug()'s own sanitization (embedded
    '\\n'/'\\r' stripped; null/empty/all-newline text all collapse to
    the bare "debug\\n" shape; a too-long text is truncated to the
    233-byte cap, not overflowed) and RUN's ack-then-ret path with a
    REGISTERED return value, under sanitizers. (The `#0`-suppression
    scenario this test used to also cover is gone along with `#0`
    itself -- docs/design/protocol.md §6.3/§2.2 -- see
    run_debug_driver.cpp's own header comment.)"""
    result = subprocess.run(
        [str(run_debug_driver)], capture_output=True, timeout=10,
        env={**__import__("os").environ, **_SANITIZER_ENV_EXTRA})
    _assert_survived(result, "run/debug driver")
    lines = result.stdout.splitlines()
    assert lines == [
        b"debug helloworld",       # embedded '\n'/'\r' stripped
        b"debug",                  # "" -> bare
        b"debug",                  # nullptr -> the SAME case as ""
        b"debug",                  # entirely '\n'/'\r' -> bare too
        b"debug " + b"z" * 233,    # truncated to the 233-byte text cap
        b"ack 1 0 none",           # RUN foo #1 -- in order, acked...
        b"ret 42 #1",              # ...then its own registered ret
    ], f"unexpected output: {result.stdout!r}"
