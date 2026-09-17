#!/usr/bin/env bash
# read_counters.sh - read the g_dbg diagnostic counters from a running or
# wedged PureThermal over SWD, twice, and show what moved in between.
#
# What moves while the unit is WEDGED is the whole diagnostic:
#
#   resync_entries climbing            -> livelocked in the VoSPI resync path
#   vsync_irqs frozen AND resync frozen-> stalled at PT_WAIT_UNTIL(current_buffer)
#                                         because the Lepton stopped asserting VSYNC
#   vsync_irqs climbing, frames flat   -> transfers running but never validating
#   frames_completed climbing          -> acquisition is fine; fault is on the USB side
#
# Usage:  sudo ./scripts/read_counters.sh [seconds-between-reads]
#         run it from the directory holding main.out

set -uo pipefail
GAP=${1:-10}
ELF=${ELF:-main.out}

command -v gdb-multiarch >/dev/null || {
  echo "gdb-multiarch not installed. Run:"
  echo "    sudo apt install -y gdb-multiarch"
  exit 1; }
command -v st-util >/dev/null || {
  echo "st-util not installed. Run:"
  echo "    sudo apt install -y stlink-tools"
  exit 1; }
[ -f "$ELF" ] || { echo "no $ELF here - run this from the build directory"; exit 1; }

STARTED_SERVER=0
if ! ss -ltn 2>/dev/null | grep -q ':4242'; then
  st-util >/tmp/stutil.log 2>&1 &
  STARTED_SERVER=$!
  sleep 2
  ss -ltn 2>/dev/null | grep -q ':4242' || {
    echo "st-util failed to start. Last lines:"; tail -5 /tmp/stutil.log; exit 1; }
fi
cleanup() { [ "$STARTED_SERVER" != "0" ] && kill "$STARTED_SERVER" 2>/dev/null; }
trap cleanup EXIT

# One read. Halts the core, prints the struct, detaches so it keeps running.
read_once() {
  gdb-multiarch -batch -nx "$ELF" \
    -ex "set confirm off" \
    -ex "target extended-remote :4242" \
    -ex "print g_dbg" \
    -ex "detach" 2>/dev/null | grep -o '{.*}'
}

A=$(read_once)
[ -n "$A" ] || { echo "could not read g_dbg - is the ST-LINK attached?"; exit 1; }
echo "waiting ${GAP}s with the target running..."
sleep "$GAP"
B=$(read_once)

A="$A" B="$B" GAP="$GAP" python3 - <<'PY'
import os,re
def parse(s): return dict((k,int(v)) for k,v in re.findall(r'(\w+)\s*=\s*(\d+)', s))
a,b = parse(os.environ['A']), parse(os.environ['B'])
if not a: print("could not parse gdb output:", os.environ['A']); raise SystemExit(1)

m = a.get('magic')
print()
if m == 0xDBC0FFEE:
    print("magic 0x%08X  OK - reading the right address" % m)
else:
    print("magic 0x%08X  *** WRONG - expected 0xDBC0FFEE. Stale main.out, or" % (m or 0))
    print("                  the build on the board isn't this one. Stop here.")
    raise SystemExit(1)
print()
print("%-18s %12s %12s %10s" % ("counter","first","second","delta"))
print("-"*56)
for k in [k for k in a if k != 'magic']:
    d = b.get(k,0)-a[k]
    print("%-18s %12d %12d %10s" % (k, a[k], b.get(k,0), ("+%d"%d) if d else "."))
print()
moving = [k for k in a if k!='magic' and b.get(k,0)-a[k] > 0]
print("moving over %ss: %s" % (os.environ['GAP'], ", ".join(moving) if moving else "NOTHING - every counter frozen"))
PY
