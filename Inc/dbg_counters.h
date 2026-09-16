/*
 * dbg_counters.h - diagnostic counters for the sticky-frame-grab investigation.
 *
 * Read these live over SWD while a unit is wedged. They live in one struct so
 * there is a single address to look up in main.map, and incrementing a word
 * costs nothing, so unlike printf they cannot perturb the race being hunted.
 *
 * Decision table, read while the LED is frozen:
 *
 *   resync_entries climbing            -> livelocked in the VoSPI resync path.
 *                                         The stale-EXTI theory is right.
 *   vsync_irqs frozen, resync frozen   -> stalled at PT_WAIT_UNTIL(current_buffer
 *                                         != NULL); the Lepton stopped asserting
 *                                         VSYNC entirely. Different bug.
 *   vsync_irqs climbing, frames flat   -> transfers are running but never
 *                                         validating. Look at desync_events.
 *   frames_completed climbing          -> acquisition is fine, the fault is on
 *                                         the USB side after all.
 */
#ifndef DBG_COUNTERS_H_
#define DBG_COUNTERS_H_

#include <stdint.h>

struct dbg_counters {
  uint32_t magic;            /* 0xDBC0FFEE - confirms you are at the right address */
  uint32_t vsync_irqs;       /* EXTI13 callbacks taken */
  uint32_t transfers_done;   /* complete_lepton_transfer() returned OK */
  uint32_t transfer_fails;   /* ... returned anything else */
  uint32_t desync_events;    /* last_end_line did not match the expected line */
  uint32_t resync_entries;   /* entered the 185ms resync sequence */
  uint32_t resync_packets;   /* discard packets consumed inside the resync loop */
  uint32_t frames_completed; /* good frames handed to usb_task */
  uint32_t frames_dropped;   /* good frames lost because the completed ring was full */
};

extern volatile struct dbg_counters g_dbg;

#endif /* DBG_COUNTERS_H_ */
