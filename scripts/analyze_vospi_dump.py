#!/usr/bin/env python3
"""
analyze_vospi_dump.py - decode a raw memory dump of lepton_buffers[] taken
from a PureThermal over SWD, and report what the sensor actually sent and how
its packet framing lines up with the MCU's 244-byte records.

Taking the dump (STM32CubeProgrammer, *Hot plug* mode so the MCU is not reset):
  1. arm-none-eabi-nm main.out | grep lepton_buffers   -> base address
  2. Halt the core, Memory & File editing: Address = base, Size = 0xF060
  3. Read, then Read drop-down -> Save As... -> lepton_buffers.bin
  4. python3 scripts/analyze_vospi_dump.py lepton_buffers.bin

Verdicts, per buffer:
  HEALTHY          video packets 0..59 in order, headers at byte 0.
  ALIGNED/PARTIAL  video headers at byte 0, but the read started or ended at
                   the wrong packet (packet-level timing, fixable by walking).
  FRAMING OFFSET   video headers sit k halfwords into every record: the MCU's
                   packet boundaries and the sensor's disagree.
  BIT SLIP         video headers only line up after re-cutting the bitstream
                   1..15 bits later: the sensor counted SCK edges differently
                   from the MCU.
  DISCARDS ONLY    every record holds a discard packet (ID xFxx): the sensor is
                   sending no video at all ("until synchronization is achieved
                   ... Lepton transmits discard packets", datasheet Rev 400).
                   The framing of those discards is reported too.
  NO VOSPI         nothing packet-like at any offset.

Heuristics (datasheet Rev 400 §4.2.3): an ID's first bit is always 0; discard
IDs are xFxx; TTT is only meaningful on packet 20. A run of sequential video
IDs counts as real only if it is >= 20 packets long, or >= 6 with TTT = 0 away
from packet 20 (or CRC agreement) and, in a discard-dominated buffer, at the
same framing as the discards. Real Lepton discard payloads carry a counter
whose top nibble steps every packet; re-cut at odd bit offsets it fakes short
sequential runs, which is why. A discard ID re-cut by a bit or two is often
still a valid discard ID, so discard framing is resolved by matching the
packet contents against the aligned buffers.

`--selftest` builds synthetic buffers for each case and checks the verdicts.
"""

import argparse
import collections
import random
import struct
import sys

RECORD_LINES = 63                 # IMAGE_NUM_LINES + TELEMETRY_MAX_LINES
IMAGE_LINES = 60
PKT_BYTES = {"rgb": 244, "y16": 164}
STRIDE = 15384                    # sizeof(lepton_buffer), -fno-short-enums
NUMBER_OFF = RECORD_LINES * 244   # 15372: uint8_t number
STATUS_OFF = NUMBER_OFF + 4       # 15376: lepton_status status (int)
SEGMENT_OFF = STATUS_OFF + 4      # 15380: uint8_t segment
STATUS_NAMES = {0: "OK", 1: "TRANSFERRING", 2: "RESYNC", 4: "CONTINUE"}
MIN_RUN = 6
SURE_RUN = 20
DISCARD_FRACTION = 0.8

# CRC-16, x^16 + x^12 + x^5 + 1, MSB first, seed 0 assumed (not stated in
# the datasheet). Discard packets carry no valid CRC.
_CRC_TABLE = []
for _i in range(256):
    _c = _i << 8
    for _ in range(8):
        _c = ((_c << 1) ^ 0x1021) if _c & 0x8000 else (_c << 1)
    _CRC_TABLE.append(_c & 0xFFFF)


def crc16(data, crc=0):
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]
    return crc


def packet_crc(pw):
    data = bytearray()
    for j, w in enumerate(pw):
        if j == 0:
            w &= 0x0FFF
        elif j == 1:
            w = 0
        data += bytes((w >> 8, w & 0xFF))
    return crc16(data)


def is_discard(h):
    return h is not None and (h & 0x8F00) == 0x0F00


def is_video(h):
    return h is not None and (h & 0x8000) == 0 and (h & 0x0F00) != 0x0F00 and (h & 0x0FFF) < RECORD_LINES


def pkt(h):
    return h & 0x0FFF


def ttt(h):
    return (h >> 12) & 7


def label(h):
    if h is None:
        return "----"
    if h == 0xFFFF:
        return "FFFF"
    if is_discard(h):
        return "d%03X" % (h & 0xFFF) if h >> 12 == 0 else "d%04X" % h
    if is_video(h):
        s = "p%d" % pkt(h)
        if pkt(h) == 20 and ttt(h):
            s += "/s%d" % ttt(h)
        return s
    return "%04X" % h


def shifted(words, s):
    """Re-cut the bitstream s bits later: word i = bits [16i+s, 16i+s+16)."""
    if s == 0:
        return words
    return [((a << s) | (b >> (16 - s))) & 0xFFFF for a, b in zip(words, words[1:])]


def ids_at(ws, P, k, n):
    return [ws[i * P + k] if i * P + k < len(ws) else None for i in range(n)]


def best_run(hdrs):
    best = (0, None, None)
    i = 0
    while i < len(hdrs):
        if not is_video(hdrs[i]):
            i += 1
            continue
        j = i
        while j + 1 < len(hdrs) and is_video(hdrs[j + 1]) and pkt(hdrs[j + 1]) == pkt(hdrs[j]) + 1:
            j += 1
        if j - i + 1 > best[0]:
            best = (j - i + 1, i, pkt(hdrs[i]))
        i = j + 1
    return best


def crc_passes(ws, P, k, line0, L):
    ok = 0
    for i in range(line0, line0 + L):
        pw = ws[i * P + k: i * P + k + P]
        if len(pw) == P and packet_crc(pw) == pw[1]:
            ok += 1
    return ok


def template(ws, P, k, n):
    """Most common value at each word position across the discard records."""
    cols = [collections.Counter() for _ in range(P)]
    for i in range(n):
        rec = ws[i * P + k: i * P + k + P]
        if len(rec) == P and is_discard(rec[0]):
            for j, v in enumerate(rec):
                cols[j][v] += 1
    return [c.most_common(1)[0][0] if c else None for c in cols]


def analyze_region(raw, fmt, nlines):
    P = PKT_BYTES[fmt] // 2
    nwords = min(len(raw) // 2, RECORD_LINES * P)
    words = list(struct.unpack("<%dH" % nwords, raw[: nwords * 2]))
    cuts = [shifted(words, s) for s in range(16)]

    runs = []             # (L, s, k, line0, p0, ttt_clean)
    disc = {}             # (s, k) -> discard count
    for s in range(16):
        ws = cuts[s]
        for k in range(P):
            ids = ids_at(ws, P, k, nlines)
            disc[(s, k)] = sum(1 for h in ids if is_discard(h))
            L, line0, p0 = best_run(ids)
            if L >= MIN_RUN:
                run = ids[line0: line0 + L]
                runs.append((L, s, k, line0, p0, all(ttt(h) == 0 for h in run if pkt(h) != 20)))
    dmax = max(disc.values())
    strong_discards = dmax >= DISCARD_FRACTION * nlines

    # A short run is believable only if it also looks like VoSPI (clean TTT or
    # CRC) and, when the buffer is dominated by discards, sits at the same
    # framing as those discards (a partial read: discards, then video).
    video = []            # (L, s, k, line0, p0, credible, crc)
    for L, s, k, line0, p0, ttt_clean in runs:
        crc = crc_passes(cuts[s], P, k, line0, L)
        looks = ttt_clean or crc * 2 >= L
        fits = (not strong_discards) or disc[(s, k)] + L >= DISCARD_FRACTION * nlines
        if L >= SURE_RUN or (looks and fits):
            video.append((L, s, k, line0, p0, True, crc))
    video.sort(key=lambda v: (-v[0], v[1], v[2]))

    dcands = sorted(sk for sk, c in disc.items() if c == dmax)
    r = dict(fmt=fmt, P=P, words=words, cuts=cuts, n=nlines,
             hdrs=ids_at(words, P, 0, nlines), video=video[0] if video else None,
             disc_count=dmax, disc_cands=dcands, disc_at0=disc[(0, 0)])
    r["disc_pick"] = (0, 0) if (0, 0) in dcands else dcands[0]

    v = r["video"]
    if v and (v[1], v[2]) == (0, 0):
        r["verdict"] = "HEALTHY" if (v[3] == 0 and v[4] == 0 and v[0] >= IMAGE_LINES) else "ALIGNED/PARTIAL"
    elif v:
        r["verdict"] = "FRAMING OFFSET" if v[1] == 0 else "BIT SLIP"
    elif dmax >= DISCARD_FRACTION * nlines:
        r["verdict"] = "DISCARDS ONLY"
    else:
        r["verdict"] = "NO VOSPI"
    return r


def framing_text(s, k, P):
    bits = k * 16 + s
    if bits == 0:
        return "aligned (byte 0 of each record)"
    txt = "%d bits into each %d-byte record (%d halfwords + %d bits)" % (bits, P * 2, k, s)
    if s:
        txt += " - not a whole 16-bit frame, so not something the MCU alone can produce"
    return txt


def describe(r):
    out = []
    P, n = r["P"], r["n"]
    v = r["video"]
    verdict = r["verdict"]
    if verdict in ("HEALTHY", "ALIGNED/PARTIAL", "FRAMING OFFSET", "BIT SLIP"):
        L, s, k, line0, p0, _, crc = v
        out.append("video packets %d..%d in sequence at lines %d..%d" % (p0, p0 + L - 1, line0, line0 + L - 1))
        out.append("framing: " + framing_text(s, k, P))
        out.append("CRC agrees on %d of %d (seed-0 assumption)" % (crc, L))
        if verdict == "ALIGNED/PARTIAL":
            out.append("-> packet-level timing (read started/ended at the wrong packet), not framing")
        elif verdict != "HEALTHY":
            out.append("-> MCU and sensor packet boundaries disagree; walking whole records cannot fix this")
        if r["disc_count"]:
            out.append("%d of %d records are discards at their best framing" % (r["disc_count"], n))
    elif verdict == "DISCARDS ONLY":
        s, k = r["disc_pick"]
        out.append("%d of %d records are discard packets (ID xFxx); no video packet anywhere" % (r["disc_count"], n))
        out.append("discard framing: " + framing_text(s, k, P))
        stray = [i for i, h in enumerate(ids_at(r["cuts"][s], P, k, n)) if not is_discard(h)]
        if stray:
            aligned = [i for i in stray if is_discard(r["hdrs"][i])]
            out.append("records not matching that framing: %s%s" % (
                stray[:12], (" (records %s are aligned discards: older data from a different read)" % aligned) if aligned else ""))
        rec = r["cuts"][s][k: k + 20]
        out.append("first discard, words 0..19: " + " ".join("%04X" % x for x in rec))
    else:
        ffff = sum(1 for w in r["words"][: n * P] if w == 0xFFFF) / float(n * P)
        zero = sum(1 for w in r["words"][: n * P] if w == 0) / float(n * P)
        out.append("no credible video run and no consistent discard framing")
        out.append("0xFFFF (idle MISO) %.0f%%, 0x0000 %.0f%%" % (100 * ffff, 100 * zero))
    return out


def firmware_view(r):
    h, n = r["hdrs"], r["n"]
    first = h[0] & 0xFF if h[0] is not None else None
    seg = ttt(h[20]) if len(h) > 20 and h[20] is not None else None
    last = h[n - 1] & 0xFF if h[n - 1] is not None else None
    txt = "first_packet=%s  current_segment=%s  last_end_line=%s (expects 0 / 1..4 / %d)" % (first, seg, last, n - 1)
    if is_discard(h[20]):
        txt += "\n    note: line 20 is a discard packet; the 'segment' read from its TTT bits is meaningless"
    return txt


def grid(hdrs, per_row=10):
    return ["    %2d: " % i + " ".join("%-6s" % label(h) for h in hdrs[i: i + per_row])
            for i in range(0, len(hdrs), per_row)]


def pick_consistent(results):
    """For DISCARDS ONLY buffers with several equally good framings (a one-bit
    re-cut of a discard ID is often still a valid discard ID), pick the one
    whose packet template best matches the other buffers'."""
    ref = None
    for r in results:
        if r["verdict"] == "DISCARDS ONLY" and (0, 0) in r["disc_cands"]:
            ref = template(r["cuts"][0], r["P"], 0, r["n"])
            break
    if ref is None:
        return
    for r in results:
        if r["verdict"] != "DISCARDS ONLY" or len(r["disc_cands"]) < 2:
            continue

        def score(sk):
            t = template(r["cuts"][sk[0]], r["P"], sk[1], r["n"])
            return sum(1 for a, b in zip(t, ref) if a is not None and a == b)

        r["disc_pick"] = max(r["disc_cands"], key=lambda sk: (score(sk), sk == (0, 0)))


def run_file(path, fmt, nlines, show_grid):
    data = open(path, "rb").read()
    if len(data) >= STRIDE:
        nbuf = len(data) // STRIDE
        regions = [(i, data[i * STRIDE: i * STRIDE + NUMBER_OFF], data[i * STRIDE: (i + 1) * STRIDE]) for i in range(nbuf)]
        print("%s: %d bytes -> %d lepton_buffer(s) of %d bytes" % (path, len(data), nbuf, STRIDE))
    else:
        regions = [(0, data, None)]
        print("%s: %d bytes -> treating as one lines[] region" % (path, len(data)))

    results = []
    for idx, region, whole in regions:
        if fmt == "auto":
            cands = [analyze_region(region, f, nlines) for f in ("rgb", "y16")]
            rank = lambda c: ((c["video"][0] if c["video"] else 0), c["disc_count"], c["fmt"] == "rgb")
            r = max(cands, key=rank)
            r["fmt_note"] = "%s (auto)" % r["fmt"]
        else:
            r = analyze_region(region, fmt, nlines)
            r["fmt_note"] = fmt
        r["idx"], r["whole"] = idx, whole
        results.append(r)
    pick_consistent(results)

    for r in results:
        print()
        meta, whole, idx = "", r["whole"], r["idx"]
        if whole is not None and len(whole) > SEGMENT_OFF:
            number = whole[NUMBER_OFF]
            status = struct.unpack_from("<i", whole, STATUS_OFF)[0]
            meta = "  number=%d status=%s segment=%d" % (number, STATUS_NAMES.get(status, str(status)), whole[SEGMENT_OFF])
            if number != idx:
                meta += "   <-- expected number=%d: wrong base address or stride?" % idx
        print("buffer %d  [%s]%s" % (idx, r["fmt_note"], meta))
        print("  firmware's view: " + firmware_view(r))
        if show_grid:
            print("  IDs at record offset 0 (what lepton_task checks; dXXX = discard):")
            for row in grid(r["hdrs"]):
                print(row)
        print("  VERDICT: %s" % r["verdict"])
        for line in describe(r):
            print("    " + line)

    print()
    print("summary:")
    for r in results:
        if r["verdict"] == "DISCARDS ONLY":
            s, k = r["disc_pick"]
            where = "discards, %d bits" % (k * 16 + s)
        elif r["video"]:
            where = "video, %d bits" % (r["video"][2] * 16 + r["video"][1])
        else:
            where = "-"
        print("  buf%d  %-16s %s" % (r["idx"], r["verdict"], where))
    offs = set(r["disc_pick"][1] * 16 + r["disc_pick"][0] for r in results if r["verdict"] == "DISCARDS ONLY")
    if len(offs) > 1:
        print("  framing differs between buffers (%s bits): the sensor's packet phase moved between reads"
              % ", ".join(str(o) for o in sorted(offs)))
    return [r["verdict"] for r in results]


# ----------------------------------------------------------------- self-test

def _synth(fmt, offset_bits, start_pkt=0, idle=False, discards_only=False, seed=1):
    rnd = random.Random(seed)
    P = PKT_BYTES[fmt] // 2
    counter = [0x3013]

    def packet(n, seg=1):
        idw = n | ((seg << 12) if n == 20 else 0)
        pw = [idw, 0] + [rnd.randrange(65536) for _ in range(P - 2)]
        pw[1] = packet_crc(pw)
        return pw

    def discard():
        # mimic a real Lepton discard: 1FFF FFFF, zeros, a few status words
        # and a counter whose top nibble steps every packet
        pw = [0x1FFF, 0xFFFF] + [0] * (P - 2)
        pw[14:20] = [0x0004, 0x58FD, 0x1100, counter[0], 0x3530, 0x0002]
        counter[0] = (counter[0] + 0x1012) & 0xFFFF
        return pw

    stream = []
    for _ in range(2):
        stream += discard()
    if discards_only:
        for _ in range(RECORD_LINES + 2):
            stream += discard()
    else:
        for n in range(start_pkt, IMAGE_LINES):
            stream += packet(n)
    for _ in range(8):
        stream += discard()
    if idle:
        stream = [0xFFFF] * len(stream)
    bits, nbits = 0, 0
    for w in stream:
        bits = (bits << 16) | w
        nbits += 16
    start = 2 * P * 16 - offset_bits
    words = []
    for i in range(RECORD_LINES * P):
        p = start + 16 * i
        words.append((bits >> (nbits - p - 16)) & 0xFFFF if p + 16 <= nbits else 0xFFFF)
    buf = bytearray(STRIDE)
    region = struct.pack("<%dH" % len(words), *words)
    buf[: len(region)] = region
    return bytes(buf)


def selftest():
    cases = [
        ("aligned segment", dict(offset_bits=0), "HEALTHY", 0),
        ("read began at packet 23", dict(offset_bits=0, start_pkt=23), "ALIGNED/PARTIAL", 0),
        ("37-halfword framing offset", dict(offset_bits=37 * 16), "FRAMING OFFSET", 37 * 16),
        ("1-halfword framing offset", dict(offset_bits=16), "FRAMING OFFSET", 16),
        ("5 halfwords + 3 bits", dict(offset_bits=5 * 16 + 3), "BIT SLIP", 5 * 16 + 3),
        ("discards only, aligned", dict(offset_bits=0, discards_only=True), "DISCARDS ONLY", 0),
        ("discards only, 94 hw + 1 bit", dict(offset_bits=94 * 16 + 1, discards_only=True), "DISCARDS ONLY", 94 * 16 + 1),
        ("idle bus", dict(offset_bits=0, idle=True), "NO VOSPI", None),
    ]
    results = [analyze_region(_synth("rgb", **kw)[:NUMBER_OFF], "rgb", IMAGE_LINES) for _, kw, _, _ in cases]
    pick_consistent(results)
    ok = True
    for (name, kw, want, want_bits), r in zip(cases, results):
        good = r["verdict"] == want
        if good and want == "DISCARDS ONLY":
            s, k = r["disc_pick"]
            good = k * 16 + s == want_bits
        elif good and want_bits is not None:
            L, s, k = r["video"][0], r["video"][1], r["video"][2]
            good = k * 16 + s == want_bits and r["video"][6] == L
        ok &= good
        print("%-32s -> %-16s %s" % (name, r["verdict"], "ok" if good else "FAIL (wanted %s)" % want))
    print("self-test %s" % ("passed" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("dump", nargs="?", help="raw .bin dump of lepton_buffers[] (or of one buffer)")
    ap.add_argument("--format", choices=["auto", "rgb", "y16"], default="auto",
                    help="packet size: rgb = 244 bytes (UYVY/RGB888 path), y16 = 164 bytes")
    ap.add_argument("--lines", type=int, default=IMAGE_LINES,
                    help="lines per read (IMAGE_NUM_LINES + g_telemetry_num_lines), default 60")
    ap.add_argument("--no-grid", action="store_true", help="omit the per-line ID table")
    ap.add_argument("--selftest", action="store_true", help="run synthetic cases and exit")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not a.dump:
        ap.error("need a dump file (or --selftest)")
    run_file(a.dump, a.format, a.lines, not a.no_grid)
    return 0


if __name__ == "__main__":
    sys.exit(main())
