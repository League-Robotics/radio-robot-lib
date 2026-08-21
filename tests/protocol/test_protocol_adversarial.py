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
   `PING\n`, and the test asserts the reply `pong:0\n` is the last thing
   the sink produced. (Sending the recovery command WITHOUT first
   flushing the garbage line would be testing a different, stricter
   property -- whether a well-formed command survives being
   concatenated directly onto an unterminated garbage prefix, which
   spec S2's line grammar never promises -- so this file always closes
   the garbage line first, the way a real next line from a real host
   would arrive.)

3. **Two genuine parser bugs this sweep found, both fixed in
   protocol_handler.cpp** (see that file's own comments at the fix
   site for the full story) -- this file's
   test_hex_float_no_longer_bypasses_no_exponents_rule and
   test_leading_whitespace_no_longer_silently_accepted are their
   regression tests:

   - `SET:name:0x1.8p3` (C99 hex-float syntax) used to be silently
     ACCEPTED as 12.0, bypassing the "no exponents" rule (spec S2.2)
     entirely, because the exponent check only looked for 'e'/'E', not
     a hex float's 'x'/'p'. Neither Python's `float()` nor
     JavaScript's `Number()`/`parseFloat()` accepts this syntax, so
     this was a C++-only divergence a straight port would NOT
     reproduce -- an archetype-relevant finding on its own.
   - A leading-whitespace numeric field (`WHEELS: 100:100:1000`, note
     the space after the first ':') used to be silently ACCEPTED,
     because strtol/strtoul/strtof all skip leading whitespace per the
     C standard -- contradicting this file's own "strict, whole field
     consumed" doc comment, which only actually held for TRAILING
     whitespace.

   A third bug (formatConfigValue() casting a NaN adapter value to
   uint32_t -- real undefined behavior, confirmed live by UBSan) is
   regression-tested in nan_regression_driver.cpp /
   test_nan_inf_get_reply_formatting_is_safe below; it is reachable
   only through the Adapter seam (a NaN can never arrive over the wire
   -- parseFloatField already rejects it on input), not through feed()
   directly, so it gets its own tiny driver rather than living in the
   generic fuzz list.

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


# ---------------------------------------------------------------------------
# Adversarial cases (spec grammar: line ::= verb (':' field)* '\n', verb
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
    ("embedded_nul_in_set_name", [b"SET:foo\x00bar:1.0\n"]),
    ("embedded_nul_in_set_value", [b"SET:group.alpha:1\x002:9\n"]),
    ("embedded_nul_in_wheels_field", [b"WHEELS:1\x0000:100:1000\n"]),
    ("embedded_nul_in_get_name", [b"GET:foo\x00bar\n"]),

    # ---- 8-bit / high-ASCII and UTF-8 sequences ----
    ("high_ascii_full_line", [bytes(range(0x80, 0x100)) + b"\n"]),
    ("high_ascii_verb", [bytes([0xC0, 0xC1, 0xFE, 0xFF]) + b"\n"]),
    ("utf8_verb", ["日本語".encode("utf-8") + b"\n"]),
    ("utf8_in_set_value",
     [b"SET:" + "日本語".encode("utf-8") + b":1.0\n"]),
    ("utf8_in_get_name",
     [b"GET:" + "éèê\U0001F600".encode("utf-8") + b"\n"]),

    # ---- other control characters ----
    ("c0_control_chars_full_line",
     [bytes(b for b in range(1, 32) if b not in (0x0A,)) + b"\n"]),
    ("del_byte_full_line", [b"\x7f\n"]),
    ("del_byte_in_set_value", [b"SET:group.alpha:1\x7f0\n"]),
    ("bell_and_escape_in_verb", [b"P\x07I\x1bNG\n"]),

    # ---- very long runs of ':' / colon-only lines / trailing colons ----
    ("very_long_colon_run", [b":" * 300 + b"\n"]),
    ("line_only_colons_short", [b":::\n"]),
    ("line_only_colons_long", [b":" * 238 + b"\n"]),
    ("verb_with_trailing_colons", [b"PING" + b":" * 50 + b"\n"]),
    ("known_verb_many_trailing_colons", [b"STOP" + b":" * 100 + b"\n"]),

    # ---- empty fields everywhere ----
    ("empty_fields_set", [b"SET::::::\n"]),
    ("empty_fields_wheels", [b"WHEELS::::\n"]),
    ("empty_fields_get", [b"GET:\n"]),
    ("empty_fields_tlm", [b"TLM:\n"]),
    ("empty_fields_stop", [b"STOP:\n"]),

    # ---- empty lines / blank-line runs ----
    ("empty_line", [b"\n"]),
    ("three_empty_lines", [b"\n\n\n"]),
    ("many_empty_lines", [b"\n" * 20]),

    # ---- \r handling: lone \r, \r\n, \n\r ----
    ("crlf", [b"\r\n"]),
    ("lfcr", [b"\n\r"]),
    ("cr_mid_field_not_at_terminator", [b"WHEELS:\r100:100:1000\n"]),
    ("multiple_lone_cr_mid_line", [b"PING\r\r\r\n"]),

    # ---- lines at 239 / 240 / 241 bytes, and further over ----
    ("line_content_238_total_239_under_cap", [b"Z" * 238 + b"\n"]),
    ("line_content_239_total_240_exact_cap", [b"Z" * 239 + b"\n"]),
    ("line_content_240_total_241_over_cap", [b"Z" * 240 + b"\n"]),
    ("line_content_1000_way_over_cap", [b"Z" * 1000 + b"\n"]),

    # ---- unterminated: partial lines, huge no-terminator blobs,
    # spread across MULTIPLE feed() calls ----
    ("unterminated_short_fragment", [b"WHEELS:100:100"]),
    ("unterminated_lone_cr", [b"\r"]),
    ("unterminated_4kb_single_call", [b"A" * 4096]),
    ("unterminated_plausible_prefix_then_huge_continuation",
     [b"WHEELS:100:100:1000", b"B" * 5000]),
    ("unterminated_split_across_many_small_calls",
     [b"W", b"H", b"E", b"E", b"L", b"S", b":", b"1" * 300]),

    # ---- mixed-case / case-as-direction edge cases (spec S2.1) ----
    ("all_lowercase_verb_dropped", [b"ping\n"]),
    ("mixed_case_verb_unknown", [b"Wheels:100:100:1000\n"]),
    ("lowercase_verb_with_colons_and_high_bytes",
     [b"dbg:" + bytes(range(0x80, 0x90)) + b"\n"]),

    # ---- numeric-field adversarial spellings ----
    ("wheels_field_all_pluses", [b"WHEELS:+100:+100:+1000\n"]),
    ("wheels_field_leading_zeros", [b"WHEELS:000100:00100:0001000\n"]),
    ("set_value_only_a_sign", [b"SET:group.alpha:-\n"]),
    ("set_value_only_a_dot", [b"SET:group.alpha:.\n"]),
    ("set_value_many_dots", [b"SET:group.alpha:1.2.3.4\n"]),
    ("wheels_duration_huge_digit_run",
     [b"WHEELS:100:100:" + b"9" * 40 + b"\n"]),
]


def _adversarial_ids():
    return [name for name, _chunks in ADVERSARIAL_CASES]


@pytest.mark.parametrize("name,chunks", ADVERSARIAL_CASES, ids=_adversarial_ids())
def test_recovers_after_adversarial_input(fuzz_driver, name, chunks):
    """The recovery invariant (docs/design/protocol.md S2.1 / spec S2):
    however hostile `chunks` is, the parser must (a) never crash,
    overflow, or trap under ASan/UBSan, and (b) still dispatch a clean
    PING correctly once the garbage line is closed out. A handler that
    wedges after one bad frame is useless on a lossy radio link."""
    # A bare '\n' first closes out whatever partial/overflowing line the
    # adversarial chunks left pending, so PING arrives as its own clean
    # line -- see this module's docstring point 2 for why that is the
    # fair way to phrase "a subsequent well-formed line", not a way to
    # dodge the harder case.
    result = _run(fuzz_driver, list(chunks) + [b"\n", b"PING\n"])
    _assert_survived(result, f"case {name!r}")
    assert result.stdout.endswith(b"pong:0\n"), (
        f"case {name!r}: PING after the garbage did not produce the "
        f"expected reply -- handler did not recover\n"
        f"stdout: {result.stdout!r}")


def test_recovers_after_every_adversarial_input_in_one_session(fuzz_driver):
    """Companion to the per-case sweep above: ALL adversarial cases,
    back-to-back, on ONE handler instance in ONE process (rather than a
    fresh handler per case) -- the failure mode a per-case sweep cannot
    see is state leaking or accumulating badly enough across many bad
    lines that some LATER, unrelated line misdispatches. A PING is
    interleaved after every case; every one must come back clean."""
    chunks = []
    for _name, case_chunks in ADVERSARIAL_CASES:
        chunks.extend(case_chunks)
        chunks.append(b"\n")
        chunks.append(b"PING\n")
    result = _run(fuzz_driver, chunks, timeout=30)
    _assert_survived(result, "combined adversarial session")
    pong_count = result.stdout.count(b"pong:0\n")
    # +1, not exactly len(ADVERSARIAL_CASES): the "embedded_nul_after_verb"
    # case (b"PING\x00extra\n") is itself indistinguishable from a bare
    # "PING\n" to this parser -- see
    # test_embedded_nul_immediately_after_verb_matches_bare_verb below for
    # why -- so its OWN garbage payload already produces one pong, on top
    # of the recovery PING appended after every case. Any OTHER deficit
    # or surplus is a real finding, not this known, deterministic one.
    expected = len(ADVERSARIAL_CASES) + 1
    assert pong_count == expected, (
        f"expected {expected} pong replies ({len(ADVERSARIAL_CASES)} "
        f"recovery PINGs + 1 from embedded_nul_after_verb's own payload "
        f"matching PING), got {pong_count} -- a PING somewhere in the "
        f"session did not come back, so state from an earlier case "
        f"corrupted a later one\nfull stdout: {result.stdout!r}")


def test_random_byte_fuzz_survives_and_recovers(fuzz_driver):
    """Broad-spectrum fuzzing beyond the hand-picked cases above:
    uniformly random byte strings (any value 0-255, including '\\n' and
    ':' at random positions, so a single blob can contain several
    "lines" of pure noise), FIXED seed so a failure reproduces exactly.
    Each trial is followed by the same flush + PING recovery check."""
    rng = random.Random(20260820)
    trial_count = 40
    for trial in range(trial_count):
        length = rng.randint(0, 500)
        blob = bytes(rng.randrange(256) for _ in range(length))
        result = _run(fuzz_driver, [blob, b"\n", b"PING\n"])
        _assert_survived(result, f"random fuzz trial {trial} (len={length})")
        assert result.stdout.endswith(b"pong:0\n"), (
            f"random fuzz trial {trial} (len={length}): did not recover\n"
            f"blob: {blob!r}\nstdout: {result.stdout!r}")


def test_embedded_nul_immediately_after_verb_matches_bare_verb(fuzz_driver):
    """NOT a crash, and not something this pass fixes -- a
    characterization test that PINS a real divergence for the
    "archetype" question, so it cannot silently change later without a
    test noticing.

    Every wire-touching comparison in this handler (dispatch()'s
    strcmp() verb lookup, and every field decode) operates on
    NUL-terminated C strings, per protocol_handler.h's own documented
    "no allocation, no std::string" constraint. strcmp() stops
    comparing at the first NUL byte in EITHER operand -- so
    "PING\x00extra" and "PING" compare EQUAL, because both have a NUL
    at index 4. The result: `PING\x00extra\n` dispatches exactly like
    `PING\n`, silently discarding "extra" (and anything else up to the
    real '\n') with no malformed-count increment and no sign anything
    was dropped.

    Spec S2's verb grammar (`verb ::= [A-Za-z][A-Za-z0-9_]*`) does not
    admit NUL in a verb at all, so the grammar-correct behavior would be
    to REJECT this line as unparseable -- this C-string-based
    implementation instead silently accepts it as the shorter verb.

    This is exactly the kind of thing a MicroPython or JavaScript port
    would NOT reproduce: `bytes`/`str` equality in Python, and string
    equality in JavaScript, compare the FULL length, embedded NUL bytes
    included -- `b"PING\x00extra" == b"PING"` is False. A port that
    otherwise faithfully mirrors this C++ handler's logic would treat
    this exact input as an unknown verb (or a malformed one), not as
    PING, and that divergence would only surface as a conformance-suite
    mismatch on inputs no one thought to write down -- which is why it
    is written down here."""
    result = _run(fuzz_driver, [b"PING\x00extra\n"])
    _assert_survived(result, "embedded NUL immediately after a verb name")
    assert result.stdout == b"pong:0\n", (
        f"expected this to (surprisingly) dispatch as PING, got: "
        f"{result.stdout!r} -- if this now differs, the C-string "
        f"dispatch behavior changed and this module's docstring / "
        f"test_recovers_after_every_adversarial_input_in_one_session's "
        f"pong-count math both need updating")



# ---------------------------------------------------------------------------
# Regression tests for the bugs this sweep found (protocol_handler.cpp's
# own comments at each fix site have the full story).
# ---------------------------------------------------------------------------

def test_hex_float_no_longer_bypasses_no_exponents_rule(fuzz_driver):
    """`SET:name:0x1.8p3` is C99 hex-float syntax -- strtof() parses it
    to 12.0 unconditionally, and the old exponent guard only checked
    for 'e'/'E', never 'x'/'X', so this slipped through as if it were
    an ordinary decimal literal. Must now be rejected as malformed
    (err code 2, ERR_BADARG), the same as any other unparseable value,
    and must NOT have reached the adapter's onSet()."""
    result = _run(fuzz_driver, [b"SET:group.alpha:0x1.8p3\n"])
    _assert_survived(result, "hex float SET")
    assert result.stdout == b"err:0:2\n", (
        f"hex-float value was not rejected: {result.stdout!r}")


@pytest.mark.parametrize("spelling", [
    b"0x1p0", b"0X1P1", b"0x1.8p3", b"0xAp0", b"0x0p0",
])
def test_hex_float_rejected_for_several_spellings(fuzz_driver, spelling):
    result = _run(fuzz_driver, [b"SET:group.alpha:" + spelling + b"\n"])
    _assert_survived(result, f"hex float spelling {spelling!r}")
    assert result.stdout == b"err:0:2\n", (
        f"hex-float spelling {spelling!r} was not rejected: "
        f"{result.stdout!r}")


def test_leading_whitespace_no_longer_silently_accepted(fuzz_driver):
    """A leading space (or tab, or a stray '\\r' that has drifted
    mid-field rather than sitting immediately before the terminator) in
    a numeric field used to sail through strtol/strtoul/strtof, which
    all skip leading whitespace per the C standard -- silently
    accepting "WHEELS: 100:100:1000" as left=100. Must now be rejected
    as malformed (ERR_BADARG), matching this file's own "strict, whole
    field consumed" contract."""
    result = _run(fuzz_driver, [b"WHEELS: 100:100:1000\n"])
    _assert_survived(result, "leading-space WHEELS field")
    assert result.stdout == b"err:0:2\n", (
        f"leading-whitespace numeric field was not rejected: "
        f"{result.stdout!r}")


@pytest.mark.parametrize("line", [
    b"WHEELS:\t100:100:1000\n",
    b"SET:group.alpha: 1.0\n",
])
def test_leading_whitespace_rejected_across_verbs(fuzz_driver, line):
    result = _run(fuzz_driver, [line])
    _assert_survived(result, f"leading-whitespace line {line!r}")
    assert result.stdout == b"err:0:2\n", (
        f"leading-whitespace field in {line!r} was not rejected: "
        f"{result.stdout!r}")


def test_leading_whitespace_rejected_in_stop_id_no_reply(fuzz_driver):
    """STOP's id is REQUIRED, not optional (spec S3.1's `STOP | id`, no
    brackets) -- handleStop() decodes it directly with parseUint32() and
    has no id-present/absent resolution step to fall back to, so when
    the id itself fails to parse there is nothing trustworthy to echo
    an err reply against. This matches the EXISTING no-reply-on-
    unparseable-id behavior (e.g. "STOP:notanumber") -- a leading-space
    id ("STOP: 5") is rejected the same way, silently, not with
    "err:0:2" like SET/WHEELS's optional-id fields."""
    result = _run(fuzz_driver, [b"STOP: 5\n"])
    _assert_survived(result, "leading-whitespace STOP id")
    assert result.stdout == b"", (
        f"STOP with an unparseable id must produce NO reply (no id can "
        f"be trusted to echo), got: {result.stdout!r}")


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

    Also re-checks the historical GET reply-buffer bug (a 235-byte
    field name, the near-cap legal maximum per spec S2's 240-byte line
    cap) on the same buffer, non-regression only."""
    result = subprocess.run(
        [str(nan_driver)], capture_output=True, timeout=10,
        env={**__import__("os").environ, **_SANITIZER_ENV_EXTRA})
    _assert_survived(result, "NaN/Inf/long-name GET regression driver")
    lines = result.stdout.splitlines()
    assert lines == [
        b"get:nan.field:0.000000",
        b"get:posinf.field:4294.967040",
        b"get:neginf.field:-4294.967040",
        b"get:" + b"n" * 235 + b":1.500000",
    ], f"unexpected output: {result.stdout!r}"
