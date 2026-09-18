/*
 * stall_watchdog.c - detect the main loop going dead, from the SysTick ISR.
 *
 * Why this exists, and why it is shaped this way:
 *
 * The Lepton SDK's command layer polls the sensor's STATUS register in
 * unbounded do{}while(!done) loops (LEPTON_I2C_Protocol.c, six of them). They
 * exit only when the sensor clears its BUSY bit, or when the I2C transport
 * itself errors. A sensor that answers every transaction cleanly but never
 * clears BUSY spins there forever.
 *
 * That matters more than it sounds. Those calls are made from lepton_task(),
 * which runs under PT_SCHEDULE() in main()'s while(1). A blocking call inside a
 * protothread does not yield - it stops the whole cooperative scheduler. Every
 * other task, usb_task included, stops with it. Interrupts keep running, so
 * SysTick still ticks and the USB device still enumerates and answers control
 * transfers on EP0, which is exactly why a wedged unit looks alive from the
 * host while producing no video at all.
 *
 * The vendor SDK is deliberately left untouched, so we cannot put a timeout
 * inside the loop. Instead we watch from the outside: main() bumps
 * g_dbg.main_loop_ticks every pass, and this runs in the SysTick interrupt,
 * which keeps firing no matter what the main loop is doing. If those ticks stop
 * advancing, the scheduler is blocked, and g_dbg.phase says in which call.
 *
 * Recording only by default. Resetting here would clear the wedge and destroy
 * the evidence, which is the opposite of what we want while diagnosing - the
 * unit stays wedged and fully inspectable over SWD. Build with
 * -DLEPTON_STALL_RESET to turn this into a recovery mechanism instead, once
 * the diagnosis is settled.
 */

#include "stm32f4xx_hal.h"
#include "dbg_counters.h"

/* Generous enough that no legitimate blocking call trips it. The longest
 * normal one is the 185ms VoSPI resync wait, and that yields anyway. */
#ifndef LEPTON_STALL_TIMEOUT_MS
#define LEPTON_STALL_TIMEOUT_MS (3000u)
#endif

void stall_watchdog_tick(void)
{
  static uint32_t last_ticks = 0;
  static uint32_t stalled_ms = 0;

  if (g_dbg.main_loop_ticks != last_ticks)
  {
    last_ticks = g_dbg.main_loop_ticks;
    stalled_ms = 0;
    return;
  }

  /* Ticks frozen. Note that the phase alone cannot tell us this - phases that
   * yield (IDLE_BLINK, WAIT_BUFFER) can sit for a long time quite legitimately
   * while the main loop keeps spinning. It is the ticks that distinguish
   * "waiting" from "blocked". */
  if (++stalled_ms < LEPTON_STALL_TIMEOUT_MS)
    return;

  if (!g_dbg.stall_detected)
  {
    g_dbg.stall_detected = 1u;
    g_dbg.stall_phase    = g_dbg.phase;
    g_dbg.stall_ms       = HAL_GetTick() - g_dbg.phase_entry_ms;
  }

#ifdef LEPTON_STALL_RESET
  NVIC_SystemReset();
#endif
}
