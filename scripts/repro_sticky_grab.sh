#!/usr/bin/env bash
# repro_sticky_grab.sh v2 - A/B test for the PureThermal "sticky frame grab" bug.
#
#   Mode A  N independent single-frame grabs. Each does
#           open -> SET_INTERFACE(alt=N) -> DQBUF -> SET_INTERFACE(alt=0) -> close.
#   Mode B  N frames in ONE continuous stream. One setup, one teardown.
#
# v2 changes, all of which mattered:
#   - The device is re-enumerated between modes. In v1 mode B inherited a
#     device that mode A had already wedged, which made its result meaningless.
#   - Mode A stops after -k consecutive failures instead of grinding through
#     hundreds of 10s timeouts.
#   - -r repeats mode A several times, resetting in between, and reports the
#     first-failure iteration of each run. Scatter in that number is the test
#     that separates a race from resource exhaustion.
#   - Fixed a DMESG_MARK bug that produced "syntax error in expression" when
#     dmesg is restricted (kernel.dmesg_restrict=1, the Ubuntu default).
#
# Usage: ./repro_sticky_grab.sh [-d /dev/video0] [-n 200] [-t 10] [-m ab] [-r 1] [-k 5]

set -uo pipefail

trap 'echo; echo "aborted."; exit 130' INT TERM

DEV=/dev/video0; N=200; TIMEOUT=10; MODES=ab; REPEATS=1; ABORT_AFTER=5
DELAY=0        # -s: idle seconds between grabs (accepts fractions)
SOFT_RESET=0   # -A: sysfs re-enumerate instead of asking for a replug

while getopts "d:n:t:m:r:k:s:Ah" o; do case $o in
  d) DEV=$OPTARG ;; n) N=$OPTARG ;; t) TIMEOUT=$OPTARG ;; m) MODES=$OPTARG ;;
  r) REPEATS=$OPTARG ;; k) ABORT_AFTER=$OPTARG ;; A) SOFT_RESET=1 ;;
  s) DELAY=$OPTARG ;;
  h) sed -n '2,26p' "$0"; exit 0 ;; *) exit 2 ;;
esac; done

command -v v4l2-ctl >/dev/null || { echo "need v4l-utils: sudo apt install v4l-utils"; exit 1; }
[ -e "$DEV" ] || { echo "no such device: $DEV"; exit 1; }

# ---------------------------------------------------------------- device reset
usb_dev_path() {
  local link="/sys/class/video4linux/$(basename "$DEV")/device"
  [ -e "$link" ] || return 1
  local p; p=$(dirname "$(readlink -f "$link")")
  while [ "$p" != "/" ]; do
    [ -f "$p/authorized" ] && { echo "$p"; return 0; }
    p=$(dirname "$p")
  done
  return 1
}

wait_for_dev() {
  local i
  for ((i=0;i<30;i++)); do [ -e "$DEV" ] && { sleep 1; return 0; }; sleep 1; done
  echo "  !! $DEV never came back"; return 1
}

reset_device() {
  local p
  # A sysfs authorized 0/1 makes the HOST re-enumerate. The STM32 never
  # reboots, so every firmware-side static - protothread state, the lepton
  # buffer ring, VoSPI sync - survives it untouched. Measured: a wedged unit
  # stays wedged across re-enumeration and fails on the very next grab. Only
  # removing power actually resets the device, so that is the default here.
  if [ "$SOFT_RESET" = "1" ]; then
    if p=$(usb_dev_path 2>/dev/null); then
      if [ -w "$p/authorized" ]; then
        echo 0 > "$p/authorized"; sleep 1; echo 1 > "$p/authorized"
      elif sudo -n true 2>/dev/null; then
        sudo sh -c "echo 0 > $p/authorized"; sleep 1; sudo sh -c "echo 1 > $p/authorized"
      fi
      echo "  [re-enumerated $(basename "$p") - HOST ONLY, firmware state intact]"
      wait_for_dev; return
    fi
  fi
  echo "  >> Unplug the camera, wait 2s, plug it back in."
  read -rp "  >> Press Enter once it is back: " _
  wait_for_dev
}

# ---------------------------------------------------------------- mode A
mode_a() {
  local tag="$1" fails=0 first=0 slow=0 worst=0 consec=0 done_n=0 i ms rc g0
  local t0; t0=$(date +%s%N)
  for ((i=1;i<=N;i++)); do
    g0=$(date +%s%N)
    timeout "$TIMEOUT" v4l2-ctl -d "$DEV" --stream-mmap --stream-count=1 \
        --stream-to=/dev/null >/dev/null 2>&1
    rc=$?
    ms=$(( ($(date +%s%N) - g0) / 1000000 ))
    (( ms > worst )) && worst=$ms
    if [ $rc -ne 0 ]; then
      fails=$((fails+1)); consec=$((consec+1))
      if [ $first -eq 0 ]; then
        first=$i
        printf "  >>> FIRST FAILURE at grab %d  (rc=%d, %dms)\n" "$i" "$rc" "$ms"
      fi
      if [ $consec -ge $ABORT_AFTER ]; then
        printf "  >>> %d consecutive failures - wedged, stopping early\n" "$consec"
        done_n=$i; break
      fi
    else
      consec=0; done_n=$i
      if [ $ms -gt 1000 ]; then
        slow=$((slow+1)); printf "  grab %-5d slow: %dms\n" "$i" "$ms"
      fi
    fi
    # Idle gap between grabs. Each teardown calls lepton_low_power() and each
    # restart calls lepton_power_on(), which issues LEP_RunOemPowerOn and
    # returns immediately - no settle wait before VoSPI is clocked again. If
    # the wedge is really about sensor settle time, it should move with this.
    [ "$DELAY" != "0" ] && sleep "$DELAY"
  done
  printf "  %s: first_failure=%s  failures=%d  slow=%d  worst=%dms  elapsed=%ds\n" \
    "$tag" "$( [ $first -eq 0 ] && echo none || echo "$first" )" \
    "$fails" "$slow" "$worst" "$(( ($(date +%s%N) - t0)/1000000000 ))"
  FIRST_FAILS+=("$( [ $first -eq 0 ] && echo none || echo "$first" )")
}

# ---------------------------------------------------------------- report
echo "device : $DEV"
v4l2-ctl -d "$DEV" --get-fmt-video 2>/dev/null | sed -n 's/^\s*/  /p' | head -3
echo "config : n=$N timeout=${TIMEOUT}s abort_after=$ABORT_AFTER repeats=$REPEATS delay=${DELAY}s"
echo

DMESG_MARK=$(dmesg 2>/dev/null | wc -l); DMESG_MARK=${DMESG_MARK:-0}
[ "$DMESG_MARK" -eq 0 ] && echo "(dmesg unreadable - run with sudo for kernel messages)" && echo

FIRST_FAILS=()

if [[ $MODES == *a* ]]; then
  for ((r=1;r<=REPEATS;r++)); do
    echo "=== Mode A run $r/$REPEATS: up to $N single-frame grabs, teardown between each ==="
    reset_device
    mode_a "run$r"
    echo
  done
fi

if [[ $MODES == *b* ]]; then
  echo "=== Mode B: $N frames in one continuous stream (FRESH device) ==="
  reset_device
  b0=$(date +%s%N)
  timeout $((TIMEOUT * 10)) v4l2-ctl -d "$DEV" --stream-mmap \
      --stream-count="$N" --stream-to=/dev/null 2>&1 | tail -2
  rc=${PIPESTATUS[0]}
  ms=$(( ($(date +%s%N) - b0) / 1000000 )); [ "$ms" -lt 1 ] && ms=1
  case $rc in
    124) echo "  HUNG (killed after $((TIMEOUT*10))s)" ;;
      0) echo "  OK - $N frames in $((ms/1000))s (~$(( N * 1000 / ms )) fps)" ;;
      *) echo "  FAILED rc=$rc after ${ms}ms" ;;
  esac
  echo
fi

NEW=$(dmesg 2>/dev/null | tail -n +$((DMESG_MARK+1)) | grep -iE "uvc|usb" | tail -20)
[ -n "$NEW" ] && { echo "=== new kernel messages ==="; echo "$NEW"; echo; }

if [ ${#FIRST_FAILS[@]} -gt 1 ]; then
  echo "=== first-failure iteration across runs: ${FIRST_FAILS[*]} ==="
  cat <<'VAR'
  Clustered (all within ~20% of each other) -> something COUNTS UP and runs out:
    a leak, a FIFO filling, a buffer index. Deterministic, so bisectable by
    instrumenting the counter.
  Scattered (e.g. 40, 190, 95)              -> a RACE that latches once hit.
    The TxState theory predicts this shape: teardown has to land during an
    in-flight transfer, which is chance, but once latched it never recovers.
VAR
fi
