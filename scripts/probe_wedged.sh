#!/usr/bin/env bash
# probe_wedged.sh - run this WHILE the camera is in the wedged state.
#
# The question it answers: is the MCU dead, or is only the video pipeline dead?
# Those need completely different fixes, and right now we cannot tell them
# apart. Control transfers go through usbd_uvc_if.c on endpoint 0; video goes
# through usb_task.c and the Lepton VoSPI path on endpoint 0x81. If control
# still works while video does not, the MCU is alive and running its main loop,
# and the fault is downstream in acquisition or streaming.
#
# Usage: sudo ./scripts/probe_wedged.sh [/dev/video0]

DEV=${1:-/dev/video0}
[ "$(id -u)" -eq 0 ] || echo "(not root - dmesg and uvcvideo tracing will be skipped)"
echo

echo "=== 1. still enumerated? ==="
lsusb -d 1e4e: 2>/dev/null || echo "  device NOT on the bus"
[ -e "$DEV" ] && echo "  $DEV present" || echo "  $DEV MISSING"
echo

echo "=== 2. control path (endpoint 0) ==="
t0=$(date +%s%N)
timeout 10 v4l2-ctl -d "$DEV" --all >/tmp/pw_all.txt 2>&1
rc=$?; ms=$(( ($(date +%s%N) - t0)/1000000 ))
if [ $rc -eq 124 ]; then
  echo "  --all HUNG (>10s)  -> control path is dead too: MCU or USB stack wedged"
elif [ $rc -ne 0 ]; then
  echo "  --all failed rc=$rc after ${ms}ms"
else
  echo "  --all OK in ${ms}ms  -> MCU alive, main loop running, EP0 healthy"
  grep -E "Width/Height|Pixel Format|Frames per second" /tmp/pw_all.txt | sed 's/^/    /'
fi
echo

echo "=== 3. Lepton-backed controls (XU -> Lepton I2C) ==="
t0=$(date +%s%N)
timeout 10 v4l2-ctl -d "$DEV" -L >/tmp/pw_ctrls.txt 2>&1
rc=$?; ms=$(( ($(date +%s%N) - t0)/1000000 ))
if [ $rc -eq 124 ]; then
  echo "  -L HUNG  -> Lepton I2C likely not responding"
elif [ $rc -ne 0 ]; then
  echo "  -L failed rc=$rc after ${ms}ms"
else
  echo "  -L OK in ${ms}ms  -> Lepton I2C is answering, sensor is alive"
  head -6 /tmp/pw_ctrls.txt | sed 's/^/    /'
fi
echo

echo "=== 4. one traced grab attempt (is EP 0x81 silent, or sending garbage?) ==="
if [ "$(id -u)" -eq 0 ]; then
  echo 0xffff > /sys/module/uvcvideo/parameters/trace 2>/dev/null
  mark=$(dmesg | wc -l)
  timeout 10 v4l2-ctl -d "$DEV" --stream-mmap --stream-count=1 --stream-to=/dev/null >/dev/null 2>&1
  echo "  grab rc=$?"
  echo 0 > /sys/module/uvcvideo/parameters/trace 2>/dev/null
  echo "  --- uvcvideo trace ---"
  dmesg | tail -n +$((mark+1)) | tail -40 | sed 's/^/    /'
  [ "$(dmesg | tail -n +$((mark+1)) | wc -l)" -eq 0 ] && \
    echo "    (no kernel output at all - the host is not even seeing payloads)"
else
  echo "  skipped, needs root"
fi
echo

cat <<'INTERP'
=== how to read this ===
  2 OK, 3 OK, 4 silent      -> MCU and Lepton both alive; usb_task is not
                               transmitting. Either it is blocked waiting for a
                               lepton buffer that never arrives, or the IN
                               endpoint is stuck. This is the shape I expect.
  2 OK, 3 HUNG              -> Lepton I2C wedged. The fault is on the sensor
                               side, not the USB side, and my patch is
                               irrelevant to it.
  2 HUNG                    -> whole USB device stack is down; look for a hard
                               fault or a stuck interrupt, not a logic bug.
  4 shows payloads arriving -> the device IS talking and the host is rejecting
                               the frames. Read the FID/EOF complaints closely.
INTERP
