#!/usr/bin/env python3
"""
mutants.py - check that the test suite would catch known regressions.

Each mutant reintroduces one bug that has already been fixed (or removes one
safeguard) in a scratch copy of Src/lepton_task.c, builds the tests against
that copy, and runs them. A mutant some test fails on is "caught". A mutant
every test passes on means the suite has a blind spot.

Src/ is never modified. Run via `make mutants` (it needs build/ populated).

If lepton_task.c changes so that a mutant's pattern no longer matches, the
mutant is reported as STALE: update or drop it.
"""
import glob
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "Src", "lepton_task.c")

MUTANTS = {
    # Clearing EXTI with the IRQ number (40) instead of the pin mask left the
    # Lepton's stale edge pending.
    "exti_irq_number_bug": [
        ("__HAL_GPIO_EXTI_CLEAR_IT(LEPTON_GPIO3_Pin)", "__HAL_GPIO_EXTI_CLEAR_IT(EXTI15_10_IRQn)")],
    # Validation used to look only at the last line.
    "only_last_line_checked": [
        ("if (first_packet != 0 ||\n", "if (0 ||\n")],
    # The resync walk used to stop at the first non-discard packet.
    "walk_stops_at_first_non_discard": [
        ("((last_header & 0x00ff) != 0x0000) ||", "0 ||")],
    # ... and took an undriven bus (0x0000) for packet 0.
    "walk_accepts_silence": [
        ("(last_crc == 0x0000)));", "0));")],
    "walk_unbounded": [
        ("if (++resync_tries > RESYNC_MAX_PACKETS)", "if (0)")],
    # SCK idle was once 185 ms, just under 5 frame periods.
    "sck_idle_too_short": [
        ("> 190);", "> 150);")],
    # VSYNC phase delay was only set at boot and lost on the OEM power cycle.
    "no_vsync_restore": [
        ("if (lepton_restore_vsync_config() != HAL_OK)", "if (0)")],
    "no_escape_hatch": [
        ("if (consecutive_desyncs >= LEPTON_MAX_CONSECUTIVE_DESYNCS)", "if (0)")],
    "hatch_threshold_changed": [
        ("#define LEPTON_MAX_CONSECUTIVE_DESYNCS (60)", "#define LEPTON_MAX_CONSECUTIVE_DESYNCS (50)")],
    "hatch_skips_format_config": [
        ("g_dbg.vsync_cfg_fails++;\n\n\t\t\t\tapply_format_config();", "g_dbg.vsync_cfg_fails++;\n")],
    "reset_boot_wait_too_short": [
        ("> LEPTON_HW_BOOT_MS);", "> 500);")],
    "escape_firings_not_reset_per_stream": [
        ("\t\t\tescape_firings = 0;\n\n\t\t\t// flush", "\n\t\t\t// flush")],
}


def main():
    cflags = sys.argv[1:]
    test_objs = [o for o in sorted(glob.glob(os.path.join(HERE, "build", "*.o")))
                 if not os.path.basename(o).startswith("fw_")]
    if not test_objs:
        sys.exit("build/ is empty - run `make` first")

    original = open(SRC).read()
    caught = missed = stale = 0

    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, "lepton_task.c")
        obj = os.path.join(tmp, "fw.o")
        exe = os.path.join(tmp, "run_tests")

        for name, edits in MUTANTS.items():
            text = original
            if not all(old in text for old, _ in edits):
                print(f"  STALE   {name}  (pattern no longer in lepton_task.c)")
                stale += 1
                continue
            for old, new in edits:
                text = text.replace(old, new, 1)
            open(src, "w").write(text)

            subprocess.run(["gcc", *cflags, "-std=c99", "-w", "-c", "-o", obj, src],
                           check=True, cwd=HERE)
            subprocess.run(["gcc", *cflags, "-o", exe, obj, *test_objs, "-lm"],
                           check=True, cwd=HERE)
            run = subprocess.run([exe], cwd=HERE, capture_output=True, text=True)
            failing = [line.split()[1] for line in run.stdout.splitlines()
                       if line.strip().startswith("FAIL")]

            if failing:
                caught += 1
                print(f"  caught  {name}  ({len(failing)} failing, e.g. {failing[0]})")
            else:
                missed += 1
                print(f"  MISSED  {name}")

    print(f"\n{caught} caught, {missed} missed, {stale} stale")
    return 1 if missed else 0


if __name__ == "__main__":
    sys.exit(main())
