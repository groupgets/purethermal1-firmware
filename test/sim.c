/*
 * sim.c - simulated hardware for lepton_task.c. See sim.h.
 */
#include <string.h>
#include <stdio.h>

#include "sim.h"
#include "pt.h"
#include "tasks.h"
#include "lepton_i2c.h"
#include "usbd_uvc.h"

struct sim sim;
static struct pt task_pt;

/* ------------------------------------------------------------------------ */
/* Globals the firmware expects other translation units to define.          */
/* ------------------------------------------------------------------------ */

volatile uint8_t g_uvc_stream_status;
volatile uint16_t g_uvc_stream_packet_size;
volatile uint32_t g_uvc_stream_restarts;
volatile uint8_t g_lepton_type_3;
volatile uint8_t g_telemetry_num_lines;
volatile uint8_t g_format_y16;
struct uvc_streaming_control videoCommitControl;

GPIO_TypeDef fake_gpioa, fake_gpiob, fake_gpioc;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

static void log_event(sim_event_t ev)
{
  if (sim.n_events < SIM_MAX_EVENTS)
  {
    sim.events[sim.n_events].ev = ev;
    sim.events[sim.n_events].ms = sim_ms();
    sim.n_events++;
  }
}

/* ------------------------------------------------------------------------ */
/* HAL                                                                      */
/* ------------------------------------------------------------------------ */

uint32_t HAL_GetTick(void) { return (uint32_t)(sim.now_us / 1000u); }
void HAL_Delay(uint32_t ms) { sim.now_us += (uint64_t)ms * 1000u; }
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin) { (void)port; (void)pin; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState s) { (void)port; (void)pin; (void)s; }

void HAL_NVIC_EnableIRQ(IRQn_Type irq)
{
  (void)irq;
  sim.irq_enabled = 1;
  /* A latched edge is taken as soon as the line is unmasked. */
  if (sim.irq_pending)
  {
    sim.irq_pending = 0;
    HAL_GPIO_EXTI_Callback(LEPTON_GPIO3_Pin);
  }
}

void HAL_NVIC_DisableIRQ(IRQn_Type irq) { (void)irq; sim.irq_enabled = 0; }
void HAL_NVIC_ClearPendingIRQ(IRQn_Type irq) { (void)irq; }

void fake_exti_clear_it(uint32_t pin_mask)
{
  sim.exti_clear_mask = pin_mask;
  log_event(EV_EXTI_CLEAR);
  if (pin_mask & LEPTON_GPIO3_Pin)
    sim.irq_pending = 0;
}

/* ------------------------------------------------------------------------ */
/* Sensor model                                                             */
/* ------------------------------------------------------------------------ */

int sim_segment_len(void)
{
  return IMAGE_NUM_LINES + g_telemetry_num_lines;
}

static void put_packet(void *dst, uint16_t hdr, uint16_t crc, int payload_base, int pkt)
{
  int j;
  if (g_format_y16)
  {
    vospi_packet_y16 *p = dst;
    p->header[0] = hdr;
    p->header[1] = crc;
    for (j = 0; j < FRAME_LINE_LENGTH; j++)
      p->data.image_data[j] = (uint16_t)(payload_base * 7 + pkt * FRAME_LINE_LENGTH + j);
  }
  else
  {
    vospi_packet_rgb *p = dst;
    uint8_t *bytes = (uint8_t *)p->data.image_data;
    p->header[0] = hdr;
    p->header[1] = crc;
    for (j = 0; j < (int)sizeof(p->data.image_data); j++)
      bytes[j] = (uint8_t)(payload_base + pkt * 3 + j);
  }
}

static void put_fill(void *dst, uint8_t value)
{
  memset(dst, value, g_format_y16 ? sizeof(vospi_packet_y16) : sizeof(vospi_packet_rgb));
}

static void put_discard(void *dst)
{
  put_fill(dst, 0xff);
  ((uint16_t *)dst)[0] = 0x0fff;
  ((uint16_t *)dst)[1] = 0xbeef;
}

/* Clock one packet out of the sensor. */
static void read_one(void *dst)
{
  if (sim.in_reset)
  {
    put_fill(dst, 0);
    return;
  }

  if (sim.wedged)
  {
    /* What a wedged unit was measured to send: runs of discard packets
     * separated by stretches where nothing drives MISO at all. */
    if ((sim.wedge_counter++ % 300) < 220)
      put_discard(dst);
    else
      put_fill(dst, 0);
    return;
  }

  if (sim.silent_packets > 0)
  {
    sim.silent_packets--;
    put_fill(dst, 0);
    return;
  }

  if (sim.skip_next)
  {
    sim.cursor += sim.skip_next;
    sim.skip_next = 0;
  }
  if (sim.gap_at_packet >= 0 && sim.cursor == sim.gap_at_packet)
  {
    sim.cursor += sim.gap_len;
    sim.gap_at_packet = -1;
  }

  if (sim.cursor < sim_segment_len())
  {
    int pkt = sim.cursor++;
    uint16_t hdr = (uint16_t)pkt;
    if (sim.lepton3 && pkt == 20)
      hdr |= (uint16_t)((sim.segment_id & 0x7) << 12);
    if (pkt == 0 && sim.garble_next_packet0)
    {
      sim.garble_next_packet0 = 0;
      hdr = 0x0012;                         /* bit errors on MISO */
    }
    put_packet(dst, hdr, (uint16_t)(0xa000 | pkt), (int)sim.segment_index, pkt);
  }
  else
  {
    put_discard(dst);
  }
}

static void start_segment(void)
{
  if (sim.in_reset)
    return;

  if (!sim.wedged)
  {
    sim.cursor = 0;
    sim.segment_index++;
    sim.segment_id = sim.sequence[sim.segment_index % sim.sequence_len];
  }

  /* VSYNC keeps firing on a wedged sensor: only its VoSPI output is dead. */
  sim.vsync_edges++;
  if (sim.irq_enabled)
    HAL_GPIO_EXTI_Callback(LEPTON_GPIO3_Pin);
  else
    sim.irq_pending = 1;
}

/* ------------------------------------------------------------------------ */
/* SPI driver (replaces Src/lepton.c)                                       */
/* ------------------------------------------------------------------------ */

void lepton_transfer(lepton_buffer *buf, int nlines)
{
  int i;
  sim.transfers++;
  sim.packets_read += nlines;
  for (i = 0; i < nlines; i++)
  {
    if (g_format_y16)
      read_one(&buf->lines.y16[i]);
    else
      read_one(&buf->lines.rgb[i]);
  }
  buf->status = sim.dma_hang ? LEPTON_STATUS_TRANSFERRING : LEPTON_STATUS_OK;
}

lepton_status complete_lepton_transfer(lepton_buffer *buffer)
{
  return buffer->status;
}

void lepton_cs_release(void)      { log_event(EV_CS_RELEASE); }
void lepton_cs_restore(void)      { log_event(EV_CS_RESTORE); }
void lepton_hw_reset_assert(void) { sim.in_reset = 1; log_event(EV_RESET_ASSERT); }
void lepton_hw_pwdn_release(void) { log_event(EV_PWDN_RELEASE); }

void lepton_hw_reset_release(void)
{
  log_event(EV_RESET_RELEASE);
  sim.in_reset = 0;
  if (sim.wedged && sim.wedge_resets_needed > 0 && --sim.wedge_resets_needed == 0)
    sim.wedged = 0;
  /* Freshly booted: nothing to send until the next segment starts. */
  sim.cursor = sim_segment_len();
}

/* ------------------------------------------------------------------------ */
/* CCI / I2C (replaces Src/lepton_i2c.c)                                    */
/* ------------------------------------------------------------------------ */

HAL_StatusTypeDef lepton_low_power(void)  { log_event(EV_LOW_POWER); return HAL_OK; }
HAL_StatusTypeDef lepton_power_on(void)   { log_event(EV_POWER_ON); return HAL_OK; }
HAL_StatusTypeDef enable_raw14(void)      { log_event(EV_ENABLE_RAW14); return HAL_OK; }
HAL_StatusTypeDef disable_rgb888(void)    { log_event(EV_DISABLE_RGB888); return HAL_OK; }
HAL_StatusTypeDef enable_lepton_agc(void) { log_event(EV_ENABLE_AGC); return HAL_OK; }
HAL_StatusTypeDef disable_lepton_agc(void){ log_event(EV_DISABLE_AGC); return HAL_OK; }

HAL_StatusTypeDef enable_rgb888(LEP_PCOLOR_LUT_E lut)
{
  (void)lut;
  log_event(EV_ENABLE_RGB888);
  return HAL_OK;
}

HAL_StatusTypeDef enable_telemetry(void)
{
  log_event(EV_ENABLE_TELEMETRY);
  g_telemetry_num_lines = g_lepton_type_3 ? 1 : 3;
  return HAL_OK;
}

HAL_StatusTypeDef disable_telemetry(void)
{
  log_event(EV_DISABLE_TELEMETRY);
  g_telemetry_num_lines = 0;
  return HAL_OK;
}

HAL_StatusTypeDef lepton_restore_vsync_config(void)
{
  log_event(EV_RESTORE_VSYNC);
  return sim.restore_vsync_result;
}

HAL_StatusTypeDef lepton_reinit_after_reset(void)
{
  log_event(EV_REINIT_AFTER_RESET);
  return sim.reinit_result;
}

/* ------------------------------------------------------------------------ */
/* usb_task stand-in: take finished frames and check them                   */
/* ------------------------------------------------------------------------ */

static int frame_is_intact(lepton_buffer *b)
{
  int n = sim_segment_len();
  int p, j;
  for (p = 0; p < n; p++)
  {
    uint16_t hdr = g_format_y16 ? b->lines.y16[p].header[0] : b->lines.rgb[p].header[0];
    if ((hdr & 0x0f00) == 0x0f00 || (hdr & 0x00ff) != p)
      return 0;

    if (g_format_y16)
    {
      uint16_t base = b->lines.y16[0].data.image_data[0];
      for (j = 0; j < FRAME_LINE_LENGTH; j++)
        if (b->lines.y16[p].data.image_data[j] != (uint16_t)(base + p * FRAME_LINE_LENGTH + j))
          return 0;
    }
    else
    {
      /* lepton_task byte-swaps RGB lines in place before handing them on, so
       * byte 2m+1 holds what the sensor sent as byte 2m. */
      uint8_t *d = (uint8_t *)b->lines.rgb[p].data.image_data;
      uint8_t base = ((uint8_t *)b->lines.rgb[0].data.image_data)[1];
      for (j = 0; j < (int)sizeof(b->lines.rgb[p].data.image_data); j++)
      {
        uint8_t sent = d[j ^ 1];
        if (sent != (uint8_t)(base + p * 3 + j))
          return 0;
      }
    }
  }
  return 1;
}

static void consume_frames(void)
{
  lepton_buffer *b;
  while ((b = dequeue_lepton_buffer()) != NULL)
  {
    sim.frames_received++;
    sim.last_frame_ms = sim_ms();
    if (!frame_is_intact(b))
      sim.frames_corrupt++;
    sim.frames_per_segment_id[b->segment & 7]++;
  }
}

/* ------------------------------------------------------------------------ */
/* Scheduler                                                                */
/* ------------------------------------------------------------------------ */

static void step(void)
{
  lepton_task(&task_pt);
  if (sim.consume)
    consume_frames();

  sim.now_us += SIM_STEP_US;
  if (sim.now_us >= sim.next_segment_us)
  {
    sim.next_segment_us += sim.segment_period_us;
    start_segment();
  }
}

void sim_run_ms(uint32_t ms)
{
  uint64_t end = sim.now_us + (uint64_t)ms * 1000u;
  while (sim.now_us < end)
    step();
}

int sim_run_until(int (*cond)(void), uint32_t ms)
{
  uint64_t end = sim.now_us + (uint64_t)ms * 1000u;
  while (sim.now_us < end)
  {
    if (cond())
      return 1;
    step();
  }
  return cond();
}

uint32_t sim_ms(void) { return HAL_GetTick(); }

int sim_count(sim_event_t ev)
{
  int i, n = 0;
  for (i = 0; i < sim.n_events; i++)
    if (sim.events[i].ev == ev)
      n++;
  return n;
}

int sim_index_of(sim_event_t ev, int start)
{
  int i;
  for (i = start < 0 ? 0 : start; i < sim.n_events; i++)
    if (sim.events[i].ev == ev)
      return i;
  return -1;
}

void sim_reset_task(void)
{
  PT_INIT(&task_pt);
}

void sim_init(void)
{
  memset(&sim, 0, sizeof(sim));
  sim.y16 = 1;
  sim.segment_period_us = SIM_L2_SEGMENT_US;
  sim.sequence[0] = 0;
  sim.sequence_len = 1;
  sim.consume = 1;
  sim.gap_at_packet = -1;
  sim.wedge_resets_needed = 1;
  sim.restore_vsync_result = HAL_OK;
  sim.reinit_result = HAL_OK;
  sim.next_segment_us = sim.segment_period_us;

  g_uvc_stream_status = 0;
  g_lepton_type_3 = 0;
  g_telemetry_num_lines = 0;
  g_format_y16 = 0;
  memset(&videoCommitControl, 0, sizeof(videoCommitControl));

  sim_reset_task();

  /* On the board, lepton_task always gets going before a host can open a
   * stream, so its first pass is the idle branch. */
  step();
}

void sim_use_lepton3(void)
{
  sim.lepton3 = 1;
  g_lepton_type_3 = 1;
  sim.segment_period_us = SIM_L3_SEGMENT_US;
  sim.next_segment_us = sim.now_us + sim.segment_period_us;
  sim.sequence[0] = 1;
  sim.sequence[1] = 2;
  sim.sequence[2] = 3;
  sim.sequence[3] = 4;
  sim.sequence_len = 4;
}

void sim_start_stream(void)
{
  videoCommitControl.bFormatIndex = sim.y16 ? VS_FMT_INDEX(Y16) : VS_FMT_INDEX(YUYV);
  videoCommitControl.bFrameIndex = sim.telemetry ? VS_FRAME_INDEX(TELEMETRIC) : VS_FRAME_INDEX(DEFAULT);
  g_uvc_stream_status = 2;
}

void sim_stop_stream(void)
{
  g_uvc_stream_status = 0;
}
