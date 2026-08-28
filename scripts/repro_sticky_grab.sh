#!/usr/bin/env bash
# repro_sticky_grab.sh - A/B test for the PureThermal "sticky frame grab" bug.
#
# The two modes differ in exactly one way: whether the UVC stream is torn down
# between frames.
#
#   Mode A  N independent single-frame grabs. Each one does
#           open -> SET_INTERFACE(alt=N) -> DQBUF -> SET_INTERFACE(alt=0) -> close.
#           This is what ffmpeg -frames:v 1, OpenCV, and most capture scripts do.
#
#   Mode B  One continuous stream of N frames. One setup, one teardown.
#
# If A degrades and B runs clean, the bug is in the stream *restart* path and
# not in steady-state streaming. That is the whole point of this script: it
# tells you which half of the firmware to stop looking at.
#
# Usage:  ./repro_sticky_grab.sh [-d /dev/video0] [-n 200] [-t 10] [-m ab]
#
# Requires v4l-utils (v4l2-ctl). Linux only - the teardown path is what matters
# and uvcvideo is the reference implementation of it.

set -uo pipefail

DEV=/dev/video0
N=200
TIMEOUT=10
MODES=ab

while getopts "d:n:t:m:h" opt; do
  case $opt in
    d) DEV=$OPTARG ;;
    n) N=$OPTARG ;;
    t) TIMEOUT=$OPTARG ;;
    m) MODES=$OPTARG ;;
    h) sed -n '2,25p' "$0"; exit 0 ;;
    *) exit 2 ;;
  esac
done

command -v v4l2-ctl >/dev/null || { echo "need v4l-utils: sudo apt install v4l-utils"; exit 1; }
[ -e "$DEV" ] || { echo "no such device: $DEV"; exit 1; }

echo "device : $DEV"
v4l2-ctl -d "$DEV" --info 2>/dev/null | sed -n 's/^\t*/  /p' | head -4
echo "frames : $N     per-grab timeout: ${TIMEOUT}s"
echo

DMESG_MARK=$(dmesg 2>/dev/null | wc -l || echo 0)

# ---------------------------------------------------------------- mode A
if [[ $MODES == *a* ]]; then
  echo "=== Mode A: $N independent single-frame grabs (teardown between each) ==="
  fails=0
  first_fail=0
  slow=0
  worst=0
  t_start=$(date +%s%N)

  for ((i=1; i<=N; i++)); do
    g0=$(date +%s%N)
    timeout "$TIMEOUT" v4l2-ctl -d "$DEV" \
        --stream-mmap --stream-count=1 --stream-to=/dev/null >/dev/null 2>&1
    rc=$?
    ms=$(( ($(date +%s%N) - g0) / 1000000 ))
    (( ms > worst )) && worst=$ms

    if [ $rc -ne 0 ]; then
      fails=$((fails+1))
      [ $first_fail -eq 0 ] && first_fail=$i
      if [ $rc -eq 124 ]; then
        printf "  grab %-5d HUNG (killed after %ss)\n" "$i" "$TIMEOUT"
      else
        printf "  grab %-5d FAILED (rc=%d, %dms)\n" "$i" "$rc" "$ms"
      fi
    elif [ $ms -gt 1000 ]; then
      slow=$((slow+1))
      printf "  grab %-5d slow: %dms\n" "$i" "$ms"
    fi
  done

  total=$(( ($(date +%s%N) - t_start) / 1000000 ))
  echo
  echo "  completed : $((N-fails))/$N"
  echo "  failures  : $fails${first_fail:+   (first at iteration $first_fail)}"
  echo "  slow (>1s): $slow"
  echo "  worst grab: ${worst}ms"
  echo "  wall clock: $((total/1000))s"
  echo
fi

# ---------------------------------------------------------------- mode B
if [[ $MODES == *b* ]]; then
  echo "=== Mode B: $N frames in one continuous stream (single teardown) ==="
  b0=$(date +%s%N)
  timeout $((TIMEOUT * 10)) v4l2-ctl -d "$DEV" \
      --stream-mmap --stream-count="$N" --stream-to=/dev/null 2>&1 | tail -3
  rc=${PIPESTATUS[0]}
  ms=$(( ($(date +%s%N) - b0) / 1000000 ))

  if [ $rc -eq 124 ]; then
    echo "  HUNG (killed after $((TIMEOUT*10))s)"
  elif [ $rc -ne 0 ]; then
    echo "  FAILED rc=$rc after ${ms}ms"
  else
    echo "  OK - $N frames in $((ms/1000))s ($(( N * 1000 / (ms>0?ms:1) )) fps)"
  fi
  echo
fi

# ---------------------------------------------------------------- kernel side
NEW=$(dmesg 2>/dev/null | tail -n +$((DMESG_MARK+1)) | grep -iE "uvc|usb" | tail -20)
if [ -n "$NEW" ]; then
  echo "=== new kernel messages ==="
  echo "$NEW"
  echo
fi

cat <<'INTERP'
=== how to read this ===
  A degrades, B clean    -> stream restart path. TxState latch and/or stale
                            uvc_xmit_seg. This is the expected signature.
  A and B both degrade   -> steady-state streaming or VoSPI sync, not restart.
  Both clean             -> raise -n, or the host stack is masking it. Try a
                            different host controller before trusting this.

For frame-level detail on a failing run:
  sudo sh -c 'echo 0xffff > /sys/module/uvcvideo/parameters/trace'
  sudo dmesg -w
  # then re-run mode A. Look for bad FID/EOF and dropped-payload complaints.
INTERP
