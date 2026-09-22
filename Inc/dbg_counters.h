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
  PHASE_VSYNC_CFG      = 13,  /* restoring VSYNC config          BLOCKING */
  PHASE_RECOVER        = 14,  /* escape hatch re-initialising    BLOCKING */
};

#define DBG_PHASE_COUNT (16u)   /* array size; 13 phases in use */

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

  /* --- per-phase residence, latched ---------------------------------------
   * The failure lives inside a ~10s window and then clears itself, so reading
   * `phase` after the fact only ever shows idle. These record the WORST time
   * ever spent in each phase, so the evidence survives until you read it and
   * you do not have to catch the moment.
   *
   * PHASE_IDLE_BLINK will always show a huge value - it legitimately sits
   * there between runs. Ignore index 1. Any of indices 2..12 showing several
   * thousand ms is the hang. */
  uint32_t phase_max_ms[DBG_PHASE_COUNT];  /* +0x3C */
  uint32_t phase_count[DBG_PHASE_COUNT];   /* +0x7C */

  /* --- segment alignment, added with the packet-0 resync fix --- */
  uint32_t first_line_bad;   /* +0xBC  frames whose FIRST packet was not #0 */
  uint32_t resync_giveups;   /* +0xC0  resyncs that hit the packet cap */
  uint32_t vsync_cfg_fails;  /* +0xC4  lepton_restore_vsync_config() errors */
  uint32_t hard_recoveries;  /* +0xC8  escape hatch fired this many times */
  uint32_t worst_desync_run; /* +0xCC  longest run of consecutive rejects seen */

  /* --- /CS control --- */
  uint32_t cs_stuck_low;     /* +0xD0  PB12 driven high but read back low: /CS
                                       is held down by something outside the
                                       MCU, so a VoSPI resync cannot work */
};

extern volatile struct dbg_counters g_dbg;

/* Records how long the phase being left was occupied, then enters the new one. */
#define DBG_PHASE(p) do { \
    uint32_t _dbg_now = HAL_GetTick(); \
    uint32_t _dbg_el  = _dbg_now - g_dbg.phase_entry_ms; \
    uint32_t _dbg_old = g_dbg.phase; \
    if (_dbg_old < DBG_PHASE_COUNT && _dbg_el > g_dbg.phase_max_ms[_dbg_old]) \
      g_dbg.phase_max_ms[_dbg_old] = _dbg_el; \
    if ((uint32_t)(p) < DBG_PHASE_COUNT) g_dbg.phase_count[(uint32_t)(p)]++; \
    g_dbg.phase = (uint32_t)(p); \
    g_dbg.phase_entry_ms = _dbg_now; \
  } while (0)

/* Called from SysTick_Handler. See stall_watchdog.c. */
void stall_watchdog_tick(void);

#endif /* DBG_COUNTERS_H_ */
