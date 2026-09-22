/*
 * dbg_counters.h - diagnostic counters for the Lepton acquisition path.
 *
 * One struct at one address, readable over SWD while the unit is running.
 * STM32CubeProgrammer in hot-plug mode reads it without halting the core;
 * a gdb attach halts, which stops USB being serviced and changes the
 * behaviour you are trying to observe.
 *
 * Find the base address with:  grep g_dbg main.map
 * or by scanning SRAM for the magic below.
 *
 * Incrementing a word costs nothing, so unlike printf these cannot perturb
 * the timing of what they measure.
 */
#ifndef DBG_COUNTERS_H_
#define DBG_COUNTERS_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

struct dbg_counters {
  uint32_t magic;            /* +0x00  0xDBC0FFEE - you are at the right address */

  /* --- acquisition --- */
  uint32_t vsync_irqs;       /* +0x04  EXTI13 callbacks taken */
  uint32_t transfers_done;   /* +0x08  complete_lepton_transfer() returned OK */
  uint32_t transfer_fails;   /* +0x0C  ... returned anything else */
  uint32_t frames_completed; /* +0x10  good frames handed to usb_task */
  uint32_t frames_dropped;   /* +0x14  good frames lost, ring was full */

  /* --- segment alignment --- */
  uint32_t desync_events;    /* +0x18  a frame failed validation */
  uint32_t first_line_bad;   /* +0x1C  ... because its FIRST packet was not #0 */
  uint32_t worst_desync_run; /* +0x20  longest run of consecutive rejects seen.
                                       Normal operation reaches a handful; this
                                       is what LEPTON_MAX_CONSECUTIVE_DESYNCS
                                       should be tuned against. */
  uint32_t resync_entries;   /* +0x24  entered the resync sequence */
  uint32_t resync_packets;   /* +0x28  packets consumed walking to packet 0 */
  uint32_t resync_giveups;   /* +0x2C  resyncs that hit RESYNC_MAX_PACKETS */
  uint32_t vsync_cfg_fails;  /* +0x30  lepton_restore_vsync_config() errors */

  /* --- wedge recovery --- */
  uint32_t hard_recoveries;  /* +0x34  escape hatch fired: the sensor stopped
                                       producing video and was hardware reset */
  uint32_t wedge_recoveries; /* +0x38  frames validated after at least one
                                       firing in the same stream, i.e. wedges
                                       the hatch actually cleared. Should track
                                       hard_recoveries; if it lags, recovery is
                                       failing. */
  uint32_t last_recovery_firings;
                             /* +0x3C  resets the most recent recovery needed.
                                       Above 1 means one was not enough. */
  uint32_t cs_stuck_low;     /* +0x40  PB12 driven high but read back low: /CS
                                       is held down by something outside the
                                       MCU, so a VoSPI resync cannot work */
};

extern volatile struct dbg_counters g_dbg;

#endif /* DBG_COUNTERS_H_ */
