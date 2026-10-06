/*
 * test_lepton_task.c - behaviour of the VoSPI acquisition loop in
 * Src/lepton_task.c, driven through the simulated board in sim.c.
 *
 * These pin down what the firmware does today, including every desync and
 * wedge fix that has been made, so a change that quietly undoes one of them
 * fails here instead of on a unit in the field.
 *
 * The invariant checked almost everywhere: whatever the sensor does, no
 * corrupt frame is ever handed to USB (sim.frames_corrupt == 0).
 */
#include <stddef.h>

#include "test.h"
#include "sim.h"
#include "dbg_counters.h"
#include "tasks.h"

/* Private to lepton_task.c; mirrored here. If either changes on purpose,
 * update these and the tests that depend on them will say what moved. */
#define LEPTON_MAX_CONSECUTIVE_DESYNCS (60)
#define RESYNC_MAX_PACKETS (2000)

/* Bring up a stream and let it settle. */
static void stream(void)
{
  sim_start_stream();
  sim_run_ms(1000);
}

static uint32_t target;
static int desyncs_reached(void) { return g_dbg.desync_events >= target; }
static int hard_recoveries_reached(void) { return g_dbg.hard_recoveries >= target; }

/* Run until g_dbg.desync_events reaches n. */
static void run_until_desyncs(uint32_t n, uint32_t ms)
{
  target = n;
  CHECK(sim_run_until(desyncs_reached, ms));
}

/* ======================================================================== */
/* Normal streaming                                                         */
/* ======================================================================== */

TEST(l2_y16_streams_intact_frames)
{
  sim_init();
  stream();
  sim_run_ms(2000);

  CHECK_GE(sim.frames_received, 75);          /* 3 s at 27 Hz, minus start-up */
  CHECK_EQ(sim.frames_corrupt, 0);
  CHECK_EQ(g_dbg.frames_completed, sim.frames_received);
  CHECK_EQ(g_dbg.desync_events, 0);
  CHECK_EQ(g_dbg.transfer_fails, 0);
  CHECK_EQ(g_dbg.hard_recoveries, 0);
}

TEST(l2_rgb_streams_intact_byte_swapped_frames)
{
  sim_init();
  sim.y16 = 0;
  stream();
  sim_run_ms(2000);

  CHECK_EQ(g_format_y16, 0);
  CHECK_GE(sim.frames_received, 75);
  CHECK_EQ(sim.frames_corrupt, 0);            /* includes the byte-swap check */
  CHECK_EQ(g_dbg.desync_events, 0);
}

TEST(l2_y16_telemetry_frames_carry_telemetry_lines)
{
  sim_init();
  sim.telemetry = 1;
  stream();
  sim_run_ms(1000);

  CHECK_EQ(g_telemetry_num_lines, 3);
  CHECK_EQ(sim_count(EV_ENABLE_TELEMETRY), 1);
  CHECK_GE(sim.frames_received, 40);
  CHECK_EQ(sim.frames_corrupt, 0);
  CHECK_EQ(g_dbg.desync_events, 0);
}

TEST(l3_y16_streams_all_four_segments)
{
  sim_init();
  sim_use_lepton3();
  stream();
  sim_run_ms(2000);

  CHECK_EQ(sim.frames_corrupt, 0);
  CHECK_EQ(g_dbg.desync_events, 0);
  CHECK_GE(sim.frames_per_segment_id[1], 60);
  CHECK_GE(sim.frames_per_segment_id[2], 60);
  CHECK_GE(sim.frames_per_segment_id[3], 60);
  CHECK_GE(sim.frames_per_segment_id[4], 60);
}

TEST(l3_segment_zero_is_never_forwarded)
{
  int i;
  sim_init();
  sim_use_lepton3();
  /* Real Lepton 3 output: one valid frame, then repeats flagged segment 0. */
  for (i = 0; i < 12; i++)
    sim.sequence[i] = (i < 4) ? (uint8_t)(i + 1) : 0;
  sim.sequence_len = 12;
  stream();
  sim_run_ms(2000);

  CHECK_EQ(sim.frames_per_segment_id[0], 0);
  CHECK_GE(sim.frames_per_segment_id[1], 15);
  CHECK_EQ(sim.frames_corrupt, 0);
  CHECK_EQ(g_dbg.desync_events, 0);
}

TEST(vospi_packet_sizes_match_dma_length)
{
  /* lepton.c sizes the DMA in halfwords as FRAME_HEADER_LENGTH plus the
   * payload; lepton_task.c indexes the result as an array of these structs.
   * The two only agree if the structs are exactly packet-sized. */
  CHECK_EQ(sizeof(vospi_packet_y16), 2 * (FRAME_HEADER_LENGTH + FRAME_LINE_LENGTH * 2 / 2));
  CHECK_EQ(sizeof(vospi_packet_y16), 164);
  CHECK_EQ(sizeof(vospi_packet_rgb), 2 * (FRAME_HEADER_LENGTH + FRAME_LINE_LENGTH * 3 / 2));
  CHECK_EQ(sizeof(vospi_packet_rgb), 244);
}

TEST(dbg_counters_layout_matches_documented_offsets)
{
  /* Read over SWD by raw address; tooling depends on these offsets. */
  CHECK_EQ(g_dbg.magic, 0xDBC0FFEE);
  CHECK_EQ(offsetof(struct dbg_counters, vsync_irqs), 0x04);
  CHECK_EQ(offsetof(struct dbg_counters, frames_completed), 0x10);
  CHECK_EQ(offsetof(struct dbg_counters, desync_events), 0x18);
  CHECK_EQ(offsetof(struct dbg_counters, worst_desync_run), 0x20);
  CHECK_EQ(offsetof(struct dbg_counters, resync_entries), 0x24);
  CHECK_EQ(offsetof(struct dbg_counters, vsync_cfg_fails), 0x30);
  CHECK_EQ(offsetof(struct dbg_counters, hard_recoveries), 0x34);
  CHECK_EQ(offsetof(struct dbg_counters, last_recovery_firings), 0x3C);
  CHECK_EQ(offsetof(struct dbg_counters, cs_stuck_low), 0x40);
}

/* ======================================================================== */
/* Stream lifecycle                                                         */
/* ======================================================================== */

TEST(stream_start_powers_on_then_restores_vsync_config)
{
  int clear, on, vsync;
  sim_init();
  sim_run_ms(300);                            /* idle: blinking, no stream */
  CHECK_EQ(sim_count(EV_POWER_ON), 0);
  stream();

  clear = sim_index_of(EV_EXTI_CLEAR, 0);
  on    = sim_index_of(EV_POWER_ON, 0);
  vsync = sim_index_of(EV_RESTORE_VSYNC, 0);
  CHECK_GE(clear, 0);
  CHECK_GT(on, clear);
  CHECK_GT(vsync, on);                        /* power cycle can clear it */

  CHECK_EQ(sim_count(EV_DISABLE_AGC), 1);
  CHECK_EQ(sim_count(EV_ENABLE_RAW14), 1);
  CHECK_EQ(sim_count(EV_DISABLE_TELEMETRY), 1);
  CHECK_EQ(sim_count(EV_ENABLE_RGB888), 0);
}

TEST(exti_clear_uses_pin_mask_not_irq_number)
{
  sim_init();
  stream();
  /* Regression: EXTI15_10_IRQn (40) was once passed here, which cleared the
   * wrong lines and left the Lepton's stale edge pending. */
  CHECK_EQ(sim.exti_clear_mask, LEPTON_GPIO3_Pin);
}

TEST(stale_vsync_from_idle_is_not_serviced_on_start)
{
  sim_init();
  sim_run_ms(500);                            /* VSYNC edges latch while idle */
  CHECK_EQ(sim.irq_pending, 1);
  stream();
  sim_run_ms(500);

  CHECK_EQ(g_dbg.desync_events, 0);
  CHECK_EQ(sim.frames_corrupt, 0);
}

TEST(rgb_stream_start_enables_agc_and_rgb888)
{
  sim_init();
  sim.y16 = 0;
  stream();

  CHECK_EQ(sim_count(EV_ENABLE_AGC), 1);
  CHECK_EQ(sim_count(EV_ENABLE_RGB888), 1);
  CHECK_EQ(sim_count(EV_ENABLE_RAW14), 0);
}

TEST(vsync_config_failure_is_counted_and_stream_continues)
{
  sim_init();
  sim.restore_vsync_result = HAL_ERROR;
  stream();
  sim_run_ms(1000);

  CHECK_EQ(g_dbg.vsync_cfg_fails, 1);
  CHECK_GE(sim.frames_received, 40);
}

TEST(rgb_stream_stop_powers_down_and_disables_rgb888)
{
  sim_init();
  sim.y16 = 0;
  stream();
  sim_stop_stream();
  sim_run_ms(1000);

  CHECK_EQ(sim_count(EV_LOW_POWER), 2);       /* boot idle + this stop */
  CHECK_EQ(sim_count(EV_DISABLE_RGB888), 1);
}

TEST(y16_stream_stop_leaves_rgb888_alone)
{
  sim_init();
  stream();
  sim_stop_stream();
  sim_run_ms(1000);

  CHECK_EQ(sim_count(EV_DISABLE_RGB888), 0);
}

TEST(no_frames_while_stream_stopped)
{
  uint32_t before;
  sim_init();
  stream();
  sim_stop_stream();
  sim_run_ms(100);
  before = sim.frames_received;
  sim_run_ms(2000);

  CHECK_EQ(sim.frames_received, before);
}

TEST(repeated_stream_restarts_stay_in_sync)
{
  int i;
  sim_init();
  for (i = 0; i < 20; i++)
  {
    uint32_t before = sim.frames_received;
    sim.y16 = i & 1;
    stream();
    CHECK_GE(sim.frames_received - before, 20);
    sim_stop_stream();
    sim_run_ms(300);
  }

  CHECK_EQ(sim_count(EV_POWER_ON), 20);
  CHECK_EQ(sim.frames_corrupt, 0);
  CHECK_EQ(g_dbg.desync_events, 0);
}

/* ======================================================================== */
/* Desync detection and VoSPI resync                                        */
/* ======================================================================== */

TEST(misaligned_first_read_is_dropped_without_resync)
{
  sim_init();
  sim_start_stream();
  sim.skip_next = 10;                         /* very first read is mid-segment */
  sim_run_ms(1000);

  CHECK_EQ(g_dbg.desync_events, 1);
  CHECK_EQ(g_dbg.first_line_bad, 1);
  CHECK_EQ(g_dbg.resync_entries, 0);          /* < 3 frames in: just drop it */
  CHECK_GE(sim.frames_received, 20);
  CHECK_EQ(sim.frames_corrupt, 0);
}

TEST(misaligned_read_mid_stream_resyncs_and_recovers)
{
  uint32_t before;
  sim_init();
  stream();
  sim.skip_next = 10;
  run_until_desyncs(1, 500);
  before = sim.frames_received;
  sim_run_ms(1000);

  CHECK_EQ(g_dbg.desync_events, 1);
  CHECK_EQ(g_dbg.first_line_bad, 1);
  CHECK_EQ(g_dbg.resync_entries, 1);
  CHECK_EQ(g_dbg.resync_giveups, 0);
  CHECK_GE(sim.frames_received - before, 20);
  CHECK_EQ(sim.frames_corrupt, 0);
}

TEST(bad_first_packet_is_caught_by_first_line_check)
{
  sim_init();
  stream();
  /* Rest of the segment is fine, so the last-line check alone passes it. */
  sim.garble_next_packet0 = 1;
  sim_run_ms(1000);

  CHECK_EQ(g_dbg.desync_events, 1);
  CHECK_EQ(g_dbg.first_line_bad, 1);
  CHECK_EQ(sim.frames_corrupt, 0);
}

TEST(gap_inside_segment_is_caught_by_last_line_check)
{
  sim_init();
  stream();
  sim.gap_at_packet = 30;                     /* first packet fine, tail is not */
  sim.gap_len = 5;
  sim_run_ms(1000);

  CHECK_EQ(g_dbg.desync_events, 1);
  CHECK_EQ(g_dbg.first_line_bad, 0);
  CHECK_EQ(sim.frames_corrupt, 0);
}

TEST(resync_walks_to_packet_zero_not_first_non_discard)
{
  sim_init();
  stream();
  sim.skip_next = 10;
  run_until_desyncs(1, 500);
  /* The walk's first packet will be #30. Stopping there, as the walk once
   * did, reads off the end of the segment into the inter-segment gap. */
  sim.skip_next = 30;
  sim_run_ms(1000);

  CHECK_EQ(g_dbg.resync_entries, 1);
  CHECK_GT(g_dbg.resync_packets, 30);
  CHECK_LT(g_dbg.resync_packets, 30 + SIM_L2_SEGMENT_US / SIM_STEP_US + 2);
  CHECK_EQ(g_dbg.desync_events, 1);
  CHECK_EQ(sim.frames_corrupt, 0);
}

TEST(resync_rejects_silent_bus_as_packet_zero)
{
  sim_init();
  stream();
  sim.skip_next = 10;
  run_until_desyncs(1, 500);
  /* MISO undriven reads 0x0000, whose header looks like packet 0. Only the
   * zero CRC word gives it away. */
  sim.silent_packets = 50;
  sim_run_ms(1000);

  CHECK_EQ(g_dbg.resync_entries, 1);
  CHECK_GE(g_dbg.resync_packets, 51);
  CHECK_EQ(g_dbg.resync_giveups, 0);
  CHECK_EQ(sim.frames_corrupt, 0);
}

TEST(resync_idles_sck_for_more_than_five_frame_periods)
{
  uint32_t t_desync, packets_at_desync;
  sim_init();
  stream();
  sim.skip_next = 10;
  run_until_desyncs(1, 500);
  t_desync = sim_ms();
  packets_at_desync = sim.packets_read;

  /* Nothing may be clocked out of the sensor for > 189 ms (5 periods at
   * 26.4 Hz) before the walk starts. */
  sim_run_ms(185);
  CHECK_EQ(sim.packets_read, packets_at_desync);
  sim_run_ms(20);
  CHECK_GT(sim.packets_read, packets_at_desync);
  (void)t_desync;
}

TEST(resync_walk_is_bounded)
{
  sim_init();
  stream();
  sim.skip_next = 10;
  run_until_desyncs(1, 500);
  sim.wedged = 1;                             /* never presents a packet 0 */
  sim.wedge_resets_needed = -1;
  sim_run_ms(3000);

  CHECK_GE(g_dbg.resync_giveups, 1);
  CHECK_LE(g_dbg.resync_packets, g_dbg.resync_entries * (RESYNC_MAX_PACKETS + 1));
  CHECK_EQ(sim.frames_corrupt, 0);
}

TEST(short_desync_burst_does_not_fire_escape_hatch)
{
  uint32_t before;
  sim_init();
  stream();
  sim.wedged = 1;                             /* a burst well short of the limit */
  run_until_desyncs(30, 20000);
  sim.wedged = 0;
  before = sim.frames_received;
  sim_run_ms(2000);

  CHECK_EQ(g_dbg.hard_recoveries, 0);
  CHECK_GE(g_dbg.worst_desync_run, 30);
  CHECK_LT(g_dbg.worst_desync_run, LEPTON_MAX_CONSECUTIVE_DESYNCS);
  CHECK_GE(sim.frames_received - before, 20);
  CHECK_EQ(sim.frames_corrupt, 0);
}

/* ======================================================================== */
/* Wedge escape hatch                                                       */
/* ======================================================================== */

TEST(wedge_is_cleared_by_hardware_reset)
{
  uint32_t frames_at_wedge;
  sim_init();
  stream();
  frames_at_wedge = sim.frames_received;
  sim.wedged = 1;
  sim.wedge_resets_needed = 1;
  sim_run_ms(30000);

  CHECK_EQ(g_dbg.hard_recoveries, 1);
  CHECK_EQ(g_dbg.wedge_recoveries, 1);
  CHECK_EQ(g_dbg.last_recovery_firings, 1);
  CHECK_EQ(g_dbg.worst_desync_run, LEPTON_MAX_CONSECUTIVE_DESYNCS);
  CHECK_GT(sim.frames_received, frames_at_wedge + 100);
  CHECK_GT(sim.last_frame_ms, sim_ms() - 100);  /* still streaming at the end */
  CHECK_EQ(sim.frames_corrupt, 0);
}

TEST(escape_hatch_follows_datasheet_reset_sequence)
{
  int rel, ast, pwd, rst, ini, res;
  sim_init();
  stream();
  sim.wedged = 1;
  target = 1;
  CHECK(sim_run_until(hard_recoveries_reached, 30000));
  sim_run_ms(3000);

  rel = sim_index_of(EV_CS_RELEASE, 0);
  ast = sim_index_of(EV_RESET_ASSERT, rel);
  pwd = sim_index_of(EV_PWDN_RELEASE, ast);
  rst = sim_index_of(EV_RESET_RELEASE, pwd);
  ini = sim_index_of(EV_REINIT_AFTER_RESET, rst);
  res = sim_index_of(EV_CS_RESTORE, ini);
  CHECK_GE(rel, 0);
  CHECK_GT(ast, rel);
  CHECK_GT(pwd, ast);
  CHECK_GT(rst, pwd);
  CHECK_GT(ini, rst);
  CHECK_GT(res, ini);

  CHECK_GE(sim.events[pwd].ms - sim.events[ast].ms, LEPTON_HW_RESET_STEP_MS);
  CHECK_GE(sim.events[rst].ms - sim.events[pwd].ms, LEPTON_HW_RESET_STEP_MS);
  CHECK_GE(sim.events[ini].ms - sim.events[rst].ms, LEPTON_HW_BOOT_MS);
  CHECK_GE(sim.events[res].ms - sim.events[ini].ms, LEPTON_VOSPI_RESYNC_MS);

  /* Reset loses all CCI configuration: Y16 setup must be re-applied. */
  CHECK_EQ(sim_count(EV_ENABLE_RAW14), 2);
  CHECK_EQ(sim_count(EV_DISABLE_AGC), 2);
}

TEST(escape_hatch_reapplies_rgb_config)
{
  sim_init();
  sim.y16 = 0;
  stream();
  sim.wedged = 1;
  sim_run_ms(30000);

  CHECK_EQ(g_dbg.hard_recoveries, 1);
  CHECK_EQ(sim_count(EV_ENABLE_RGB888), 2);
  CHECK_EQ(sim_count(EV_ENABLE_AGC), 2);
  CHECK_EQ(sim.frames_corrupt, 0);
}

TEST(wedge_needing_two_resets_records_both)
{
  sim_init();
  stream();
  sim.wedged = 1;
  sim.wedge_resets_needed = 2;
  sim_run_ms(60000);

  CHECK_EQ(g_dbg.hard_recoveries, 2);
  CHECK_EQ(g_dbg.wedge_recoveries, 1);
  CHECK_EQ(g_dbg.last_recovery_firings, 2);
  CHECK_EQ(sim.frames_corrupt, 0);
}

TEST(permanent_wedge_keeps_retrying_without_hanging)
{
  uint32_t frames_at_wedge;
  sim_init();
  stream();
  sim.wedged = 1;
  sim.wedge_resets_needed = -1;
  sim_run_ms(5);                              /* let a frame read before the wedge land */
  frames_at_wedge = sim.frames_received;
  sim_run_ms(120000);

  CHECK_GE(g_dbg.hard_recoveries, 3);
  CHECK_EQ(g_dbg.wedge_recoveries, 0);
  CHECK_EQ(sim.frames_received, frames_at_wedge);
  CHECK_EQ(sim.frames_corrupt, 0);
}

TEST(reinit_failure_after_reset_is_counted)
{
  sim_init();
  stream();
  sim.reinit_result = HAL_ERROR;
  sim.wedged = 1;
  sim_run_ms(30000);

  CHECK_EQ(g_dbg.hard_recoveries, 1);
  CHECK_EQ(g_dbg.vsync_cfg_fails, 1);
}

TEST(new_stream_does_not_credit_previous_streams_wedge)
{
  sim_init();
  stream();
  sim.wedged = 1;
  sim.wedge_resets_needed = -1;
  target = 1;
  CHECK(sim_run_until(hard_recoveries_reached, 30000));
  sim_run_ms(3000);

  sim_stop_stream();
  sim_run_ms(500);
  sim.wedged = 0;
  stream();
  sim_run_ms(1000);

  CHECK_GE(sim.frames_received, 20);
  CHECK_EQ(g_dbg.wedge_recoveries, 0);
}

/* ======================================================================== */
/* Transfer failures and back-pressure                                      */
/* ======================================================================== */

TEST(dma_timeout_counts_failure_and_recovers)
{
  uint32_t before;
  sim_init();
  stream();
  sim.dma_hang = 1;
  sim_run_ms(1000);
  CHECK_GE(g_dbg.transfer_fails, 3);
  sim.dma_hang = 0;
  before = sim.frames_received;
  sim_run_ms(1000);

  CHECK_GE(sim.frames_received - before, 20);
  CHECK_EQ(sim.frames_corrupt, 0);
}

TEST(full_ring_drops_frames_instead_of_overwriting)
{
  sim_init();
  sim.consume = 0;                            /* USB never takes a frame */
  stream();
  sim_run_ms(1000);

  CHECK_EQ(g_dbg.frames_completed, 4);        /* RING_SIZE */
  CHECK_GT(g_dbg.frames_dropped, 0);
}

/* ======================================================================== */
/* rgb2yuv                                                                  */
/* ======================================================================== */

TEST(rgb2yuv_reference_values)
{
  uint8_t y, u, v;
  rgb_t black = { 0, 0, 0 }, white = { 255, 255, 255 }, red = { 255, 0, 0 };

  rgb2yuv(black, &y, &u, &v);
  CHECK_EQ(y, 16);  CHECK_EQ(u, 128); CHECK_EQ(v, 128);

  rgb2yuv(white, &y, &u, &v);
  CHECK_EQ(y, 235); CHECK_EQ(u, 128); CHECK_EQ(v, 128);

  rgb2yuv(red, &y, &u, &v);
  CHECK_EQ(y, 81);  CHECK_EQ(u, 90);  CHECK_EQ(v, 240);

  rgb2yuv(red, &y, NULL, NULL);               /* u and v are optional */
  CHECK_EQ(y, 81);
}
