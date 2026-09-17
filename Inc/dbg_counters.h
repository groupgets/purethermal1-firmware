/*
 * dbg_counters.h - diagnostic counters and a stall breadcrumb for the
 * sticky-frame-grab investigation.
 *
 * Read over SWD with the unit wedged. One struct, one address to find in
 * main.map. Incrementing a word costs nothing, so unlike printf this cannot
 * perturb the race being hunted.
 *
 * Field offsets from the first nine counters are unchanged from the earlier
 * build; the new fields are appended, so previously noted addresses still hold.
 * The struct's base address can still move - re-check with:  grep g_dbg main.map
 */
#ifndef DBG_COUNTERS_H_
#define DBG_COUNTERS_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Where lepton_task last was. Anything from PHASE_LOW_POWER to PHASE_POWER_ON
 * is a blocking Lepton SDK call: those reach LEPTON_I2C_Protocol.c, whose
 * status polls are unbounded do{}while(!done) loops that exit only when the
 * sensor clears BUSY or the I2C transport errors. PHASE_IDLE_BLINK,
 * PHASE_WAIT_BUFFER and PHASE_TRANSFER all yield, so sitting in one of those
 * is normal - what is abnormal is main_loop_ticks freezing. */
enum dbg_phase {
  PHASE_BOOT           = 0,
  PHASE_IDLE_BLINK     = 1,   /* slow-blink loop, stream closed  (yields) */
  PHASE_LOW_POWER      = 2,   /* lepton_low_power()              BLOCKING */
  PHASE_DISABLE_RGB888 = 3,   /* disable_rgb888()                BLOCKING */
  PHASE_TELEMETRY      = 4,   /* enable/disable_telemetry()      BLOCKING */
  PHASE_AGC            = 5,   /* enable/disable_lepton_agc()     BLOCKING */
  PHASE_RAW14          = 6,   /* enable_raw14()                  BLOCKING */
  PHASE_ENABLE_RGB888  = 7,   /* enable_rgb888()                 BLOCKING */
  PHASE_POWER_ON       = 8,   /* lepton_power_on()               BLOCKING */
  PHASE_WAIT_BUFFER    = 9,   /* waiting for a VSYNC buffer      (yields) */
  PHASE_TRANSFER       = 10,  /* VoSPI transfer + completion     (yields) */
  PHASE_RESYNC         = 11,  /* VoSPI resync sequence           (yields) */
  PHASE_PUBLISH        = 12,  /* byte-swap and push to the ring  (yields) */
};

struct dbg_counters {
  /* --- original nine, offsets unchanged --- */
  uint32_t magic;            /* +0x00  0xDBC0FFEE - you are at the right address */
  uint32_t vsync_irqs;       /* +0x04  EXTI13 callbacks taken */
  uint32_t transfers_done;   /* +0x08  complete_lepton_transfer() returned OK */
  uint32_t transfer_fails;   /* +0x0C  ... returned anything else */
  uint32_t desync_events;    /* +0x10  last_end_line mismatched */
  uint32_t resync_entries;   /* +0x14  entered the 185ms resync sequence */
  uint32_t resync_packets;   /* +0x18  discard packets consumed in resync */
  uint32_t frames_completed; /* +0x1C  good frames handed to usb_task */
  uint32_t frames_dropped;   /* +0x20  good frames lost, ring was full */

  /* --- breadcrumb --- */
  uint32_t phase;            /* +0x24  enum dbg_phase, where lepton_task is */
  uint32_t phase_entry_ms;   /* +0x28  HAL_GetTick() when that phase started */

  /* --- stall detector --- */
  uint32_t main_loop_ticks;  /* +0x2C  bumped every pass of main()'s while(1) */
  uint32_t stall_detected;   /* +0x30  1 once the main loop stopped advancing */
  uint32_t stall_phase;      /* +0x34  the phase it was in when that happened */
  uint32_t stall_ms;         /* +0x38  ms spent in that phase at detection */
};

extern volatile struct dbg_counters g_dbg;

#define DBG_PHASE(p) do { \
    g_dbg.phase = (uint32_t)(p); \
    g_dbg.phase_entry_ms = HAL_GetTick(); \
  } while (0)

/* Called from SysTick_Handler. See stall_watchdog.c. */
void stall_watchdog_tick(void);

#endif /* DBG_COUNTERS_H_ */
