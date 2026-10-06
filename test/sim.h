/*
 * sim.h - a simulated PureThermal board for host-side tests.
 *
 * Src/lepton_task.c is compiled unmodified and linked against this. The
 * simulator supplies everything lepton_task.c calls out to (HAL, SPI driver,
 * CCI/I2C helpers, USB globals) and models a Lepton's VoSPI output closely
 * enough to reproduce the desync and wedge behaviour seen on hardware:
 *
 *  - Time advances in SIM_STEP_US steps, one lepton_task() call per step.
 *  - Every segment_period_us the sensor starts a new segment (packet 0) and
 *    raises VSYNC. Reads past the last packet of a segment get discard packets
 *    until the next one starts.
 *  - VSYNC edges latch while the EXTI IRQ is disabled and fire when it is
 *    re-enabled, as the NVIC does.
 *  - Faults can be injected: skipped packets (a read that starts mid-segment),
 *    a gap partway through a segment, a silent bus (MISO reads 0x0000), a
 *    wedge (discard/silence forever until hardware reset), and DMA that never
 *    completes.
 *
 * A consumer standing in for usb_task dequeues finished frames every step and
 * checks each one is a complete, in-order segment, so a test can assert that
 * nothing corrupt was ever handed up to USB.
 */
#ifndef SIM_H
#define SIM_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "lepton.h"

#define SIM_STEP_US         (100u)
#define SIM_L2_SEGMENT_US   (37037u)   /* 27 Hz */
#define SIM_L3_SEGMENT_US   (9434u)    /* 106 Hz */

/* Lepton 3 segment numbers as the sensor reports them in packet 20. Segment 0
 * marks a repeated (invalid) frame. */
#define SIM_MAX_SEQUENCE    (16)

typedef enum {
  EV_LOW_POWER = 1,
  EV_POWER_ON,
  EV_RESTORE_VSYNC,
  EV_REINIT_AFTER_RESET,
  EV_CS_RELEASE,
  EV_CS_RESTORE,
  EV_RESET_ASSERT,
  EV_PWDN_RELEASE,
  EV_RESET_RELEASE,
  EV_IRQ_ENABLE,
  EV_IRQ_DISABLE,
  EV_EXTI_CLEAR,
  EV_ENABLE_RAW14,
  EV_ENABLE_RGB888,
  EV_DISABLE_RGB888,
  EV_ENABLE_AGC,
  EV_DISABLE_AGC,
  EV_ENABLE_TELEMETRY,
  EV_DISABLE_TELEMETRY,
} sim_event_t;

#define SIM_MAX_EVENTS (4096)

struct sim_event {
  sim_event_t ev;
  uint32_t    ms;
};

struct sim {
  /* --- configuration, set before sim_start_stream() --- */
  int      lepton3;
  int      y16;
  int      telemetry;              /* request the telemetric frame index (Y16) */
  uint32_t segment_period_us;
  uint8_t  sequence[SIM_MAX_SEQUENCE];
  int      sequence_len;
  int      consume;                /* usb_task stand-in dequeues frames */

  /* --- clock and interrupt --- */
  uint64_t now_us;
  uint64_t next_segment_us;
  int      irq_enabled;
  int      irq_pending;
  uint32_t vsync_edges;
  uint32_t exti_clear_mask;        /* last mask passed to __HAL_GPIO_EXTI_CLEAR_IT */

  /* --- sensor --- */
  int      cursor;                 /* next packet number in the current segment */
  uint32_t segment_index;          /* segments started since boot */
  uint8_t  segment_id;             /* number reported in packet 20 (Lepton 3) */
  int      in_reset;               /* RESET_L held low */

  /* --- fault injection --- */
  int      skip_next;              /* next read starts this many packets late */
  int      gap_at_packet;          /* when this packet is due ... */
  int      gap_len;                /* ... jump ahead this many (once) */
  int      silent_packets;         /* next N reads return all zeros */
  int      garble_next_packet0;    /* next packet 0 arrives with a bad header */
  int      wedged;                 /* discard/silence until hardware reset */
  int      wedge_resets_needed;    /* resets before a wedge clears; <0 never */
  uint32_t wedge_counter;
  int      dma_hang;               /* transfers never complete */

  /* --- stubbed CCI results --- */
  HAL_StatusTypeDef restore_vsync_result;
  HAL_StatusTypeDef reinit_result;

  /* --- what the firmware did --- */
  uint32_t transfers;
  uint32_t packets_read;
  struct sim_event events[SIM_MAX_EVENTS];
  int      n_events;

  /* --- what reached USB --- */
  uint32_t frames_received;
  uint32_t frames_corrupt;
  uint32_t frames_per_segment_id[8];
  uint32_t last_frame_ms;
};

extern struct sim sim;

/* Reset everything to a Lepton 2.5, Y16, no telemetry, consumer on. */
void sim_init(void);

/* Configure for a Lepton 3.x with the usual 1,2,3,4 segment sequence. */
void sim_use_lepton3(void);

/* Host opens the stream / closes it. */
void sim_start_stream(void);
void sim_stop_stream(void);

/* Advance simulated time, running lepton_task() once per step. */
void sim_run_ms(uint32_t ms);

/* Run until cond() is true or ms elapse; returns 1 if cond() became true. */
int sim_run_until(int (*cond)(void), uint32_t ms);

int  sim_count(sim_event_t ev);
int  sim_index_of(sim_event_t ev, int start);   /* -1 if absent */
uint32_t sim_ms(void);

/* Packets per segment for the current configuration. */
int sim_segment_len(void);

/* Reset lepton_task's protothread. Each test runs in its own process, so the
 * firmware's static state always starts fresh. */
void sim_reset_task(void);

#endif
