
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "usb_device.h"


#include "pt.h"
#include "lepton.h"
#include "lepton_i2c.h"
#include "tmp007_i2c.h"
#include "usbd_uvc.h"
#include "usbd_uvc_if.h"
#include "circ_buf.h"
#include "dbg_counters.h"

#include "tasks.h"
#include "project_config.h"

extern volatile uint8_t g_lepton_type_3;
extern struct uvc_streaming_control videoCommitControl;

volatile struct dbg_counters g_dbg = { .magic = 0xDBC0FFEE };

lepton_buffer *completed_buffer;
uint32_t completed_frame_count;

uint8_t lepton_i2c_buffer[36];

// Upper bound on packets consumed by one resync attempt. Normal resyncs
// settle in well under a hundred; this only stops a pathological sensor from
// pinning the task here.
#define RESYNC_MAX_PACKETS (2000)

// Escape hatch. Losing VoSPI alignment is recoverable; staying lost is not,
// and until now nothing bounded how long the firmware would keep trying. One
// rejected frame per stream restart is normal - the sensor has just been
// power-cycled and the first read lands wherever it lands - so this threshold
// sits well above that and well below forever. g_dbg.worst_desync_run records
// how close normal operation actually gets, so it can be tuned from data.
#define LEPTON_MAX_CONSECUTIVE_DESYNCS (60)

#define RING_SIZE (4)
lepton_buffer lepton_buffers[RING_SIZE];

lepton_buffer* completed_frames_buf[RING_SIZE] = { 0 };
DECLARE_CIRC_BUF_HANDLE(completed_frames_buf);

struct rgb_to_yuv_state {
  struct pt pt;
  lepton_buffer *restrict rgb;
};

#if defined(USART_DEBUG) || defined(GDB_SEMIHOSTING)
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

uint32_t get_lepton_buffer(lepton_buffer **buffer)
{
  if (buffer != NULL)
    *buffer = completed_buffer;
	return completed_frame_count;
}

lepton_buffer* dequeue_lepton_buffer()
{
  if (empty(CIRC_BUF_HANDLE(completed_frames_buf)))
    return NULL;
  else
    return shift(CIRC_BUF_HANDLE(completed_frames_buf));
}

void init_lepton_task()
{
  int i;
  for (i = 0; i < RING_SIZE; i++)
  {
    lepton_buffers[i].number = i;
    lepton_buffers[i].status = LEPTON_STATUS_OK;
    DEBUG_PRINTF("Initialized lepton buffer %d @ %p\r\n", i, &lepton_buffers[i]);
  }
}

static float k_to_c(uint16_t unitsKelvin)
{
	return ( ( (float)( unitsKelvin / 100 ) + ( (float)( unitsKelvin % 100 ) * 0.01f ) ) - 273.15f );
}

static void print_telemetry_temps(telemetry_data_l2* telemetry)
{
	//
	uint16_t fpa_temperature_k = telemetry->fpa_temp_100k[0];
	uint16_t aux_temperature_k = telemetry->housing_temp_100k[0];

	float fpa_c = k_to_c(fpa_temperature_k);
	float aux_c = k_to_c(aux_temperature_k);

	DEBUG_PRINTF("fpa %d.%d°c, aux/housing: %d.%d°c\r\n",
		(int)(fpa_c), (int)((fpa_c-(int)fpa_c)*100),
		(int)(aux_c), (int)((aux_c-(int)aux_c)*100));
}

static lepton_buffer *current_buffer = NULL;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	static int current_buffer_index = 0;
	g_dbg.vsync_irqs++;
	lepton_buffer *buffer = &lepton_buffers[current_buffer_index];
	current_buffer = buffer;
	current_buffer_index = ((current_buffer_index + 1) % RING_SIZE);
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
}

// Apply the Lepton configuration for the stream format currently negotiated.
// Called both when a stream starts and when the escape hatch re-initialises
// the sensor, so the two paths cannot drift apart.
static void apply_format_config(void)
{
	if (g_format_y16)
	{
		if (videoCommitControl.bFrameIndex == VS_FRAME_INDEX_TELEMETRIC)
			enable_telemetry();
		else
			disable_telemetry();
		disable_lepton_agc();
		enable_raw14();
	}
	else
	{
		disable_telemetry();
		enable_lepton_agc();
		enable_rgb888((LEP_PCOLOR_LUT_E)-1); // -1 keeps the current palette
	}
}

PT_THREAD( lepton_task(struct pt *pt))
{
	PT_BEGIN(pt);

	static uint32_t curtick = 0;
	static uint32_t last_tick = 0;
	static uint32_t last_logged_count = 0;
	static uint32_t current_frame_count = 0;
	static int transferring_timer = 0;
	static uint8_t current_segment = 0;
	static uint8_t first_packet = 0;
	static uint8_t last_end_line = 0;
	static int resync_tries = 0;
	static uint32_t consecutive_desyncs = 0;
	static uint32_t escape_firings = 0;     // since the last validated frame
	static uint8_t has_started_a_stream = 0;
	curtick = last_tick = HAL_GetTick();

#ifdef THERMAL_DATA_UART
	enable_telemetry();
	enable_raw14();
#endif

	while (1)
	{
#ifndef THERMAL_DATA_UART
		if (g_uvc_stream_status == 0)
		{
			lepton_low_power();
			if (has_started_a_stream)
			{
				if (g_format_y16)
				{
					// no cleanup after y16
				}
				else
				{
					disable_rgb888();
				}
			}

			// Start slow blink (1 Hz)
			while (g_uvc_stream_status == 0)
			{
				HAL_GPIO_TogglePin(SYSTEM_LED_GPIO_Port, SYSTEM_LED_Pin);

				transferring_timer = HAL_GetTick();
				PT_YIELD_UNTIL(pt, g_uvc_stream_status != 0 || (HAL_GetTick() - transferring_timer) > 500);
			}

			g_format_y16 = (videoCommitControl.bFormatIndex == VS_FMT_INDEX(Y16));

			apply_format_config();
			has_started_a_stream = 1;

			// A recovery in a new stream can't be credited to the last one.
			escape_firings = 0;

			// flush out any old data
			while (dequeue_lepton_buffer() != NULL) {}

			// Make sure we're not about to service an old irq when the interrupts are re-enabled
			// EXTI->PR takes a PIN MASK, not an IRQ number. Passing EXTI15_10_IRQn
			// (40 = 0x28) cleared lines 3 and 5 and left the Lepton's line 13 pending,
			// so the stale edge fired the instant the IRQ was re-enabled.
			__HAL_GPIO_EXTI_CLEAR_IT(LEPTON_GPIO3_Pin);
			HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);

			lepton_power_on();

			// The OEM power cycle above can clear the sensor's VSYNC phase
			// delay, which is what keeps the VSYNC pulse aligned with packet 0.
			// It was only ever set at boot, so once it reverted nothing put it
			// back and every subsequent read started mid-segment.
			if (lepton_restore_vsync_config() != HAL_OK)
				g_dbg.vsync_cfg_fails++;
		}
#endif

		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

		PT_WAIT_UNTIL(pt, current_buffer != NULL);

		lepton_transfer(current_buffer, IMAGE_NUM_LINES + g_telemetry_num_lines);

		transferring_timer = HAL_GetTick();
		PT_YIELD_UNTIL(pt, current_buffer->status != LEPTON_STATUS_TRANSFERRING || ((HAL_GetTick() - transferring_timer) > 200));

		if (complete_lepton_transfer(current_buffer) != LEPTON_STATUS_OK)
		{
			g_dbg.transfer_fails++;
			DEBUG_PRINTF("Lepton transfer failed: %d\r\n", current_buffer->status);
			current_buffer = NULL;
			continue;
		}

		g_dbg.transfers_done++;
		current_frame_count++;

		if (g_format_y16)
		{
			current_segment = ((current_buffer->lines.y16[IMAGE_OFFSET_LINES + 20].header[0] & 0x7000) >> 12);
			first_packet = (current_buffer->lines.y16[IMAGE_OFFSET_LINES].header[0] & 0x00ff);
			last_end_line = (current_buffer->lines.y16[IMAGE_OFFSET_LINES + IMAGE_NUM_LINES + g_telemetry_num_lines - 1].header[0] & 0x00ff);
		}
		else
		{
			current_segment = ((current_buffer->lines.rgb[IMAGE_OFFSET_LINES + 20].header[0] & 0x7000) >> 12);
			first_packet = (current_buffer->lines.rgb[IMAGE_OFFSET_LINES].header[0] & 0x00ff);
			last_end_line = (current_buffer->lines.rgb[IMAGE_OFFSET_LINES + IMAGE_NUM_LINES + g_telemetry_num_lines - 1].header[0] & 0x00ff);
		}

		current_buffer->segment = current_segment;

		// Check both ends of the frame. A read that began mid-segment yields
		// plausible-looking early lines and an 0xFF tail, so testing only the
		// last line let a misaligned frame get 59 lines further than it should
		// before anything noticed.
		if (first_packet != 0 ||
		    last_end_line != (IMAGE_NUM_LINES + g_telemetry_num_lines - 1))
		{
			if (first_packet != 0)
				g_dbg.first_line_bad++;
			g_dbg.desync_events++;

			if (++consecutive_desyncs > g_dbg.worst_desync_run)
				g_dbg.worst_desync_run = consecutive_desyncs;

			// flush out any old data since it's no good
			while (dequeue_lepton_buffer() != NULL) {}

			// Nothing has validated for a long time, so resyncing again is not
			// going to help - every previous attempt has already failed. Stop
			// re-reading the stream and re-initialise the sensor instead.
			//
			// This is deliberately indifferent to root cause. Whatever leaves
			// the pipeline unable to produce a frame, a wedge that needs the
			// user to physically unplug the camera is a far worse failure than
			// a two-second interruption.
			//
			// A wedged unit's bitstream is runs of discard packets separated by
			// stretches where the sensor drives nothing at all: its VoSPI
			// transmitter has stopped and does not restart on its own. VSYNC
			// keeps firing throughout, so the sensor is alive and still framing
			// - only its serial output is dead.
			//
			// Every cheaper remedy was fired at a held wedge and measured: /CS
			// high for 250 ms with the pin readback proving it went high, a
			// long SCK idle, a CCI power cycle (what this hatch used to do, 95+
			// firings without a single recovery), an SPI2/DMA reset. None
			// recovered. Pulsing RESET_L / PWR_DWN_L - the same sequence
			// lepton_init() runs at boot - recovers every time, which is also
			// why an MCU reset has always appeared to fix this.
			if (consecutive_desyncs >= LEPTON_MAX_CONSECUTIVE_DESYNCS)
			{
				g_dbg.hard_recoveries++;
				escape_firings++;
				DEBUG_PRINTF("Unrecoverable desync, sensor hardware reset #%lu\r\n", escape_firings);

				HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

				// /CS stays high across the reset, the boot wait and the CCI
				// reconfiguration, so the sensor comes up with /CS deasserted
				// and SCK idle, exactly as it does at MCU power-up.
				lepton_cs_release();
				lepton_hw_reset_assert();
				transferring_timer = HAL_GetTick();
				PT_WAIT_UNTIL(pt, (HAL_GetTick() - transferring_timer) > LEPTON_HW_RESET_STEP_MS);

				lepton_hw_pwdn_release();
				transferring_timer = HAL_GetTick();
				PT_WAIT_UNTIL(pt, (HAL_GetTick() - transferring_timer) > LEPTON_HW_RESET_STEP_MS);

				lepton_hw_reset_release();
				transferring_timer = HAL_GetTick();
				PT_WAIT_UNTIL(pt, (HAL_GetTick() - transferring_timer) > LEPTON_HW_BOOT_MS);

				if (lepton_reinit_after_reset() != HAL_OK)
					g_dbg.vsync_cfg_fails++;

				apply_format_config();

				// Hold /CS high a little longer with SCK still idle, then hand
				// PB12 back to the SPI: the tail of the datasheet's (re)sync
				// procedure, which the sensor has just booted into.
				transferring_timer = HAL_GetTick();
				PT_WAIT_UNTIL(pt, (HAL_GetTick() - transferring_timer) > LEPTON_VOSPI_RESYNC_MS);
				lepton_cs_restore();

				while (dequeue_lepton_buffer() != NULL) {}

				consecutive_desyncs = 0;
				current_frame_count = 3;   // fall through into the resync below
			}

			if (current_frame_count > 2)
			{
				g_dbg.resync_entries++;
				uint16_t last_header;
				uint16_t last_crc = 0;

				DEBUG_PRINTF("Synchronization lost, status: %d, last end line %d\r\n",
					current_buffer->status, last_end_line);

				// Idle SCK for more than 5 frame periods. 185 ms was slightly
				// under that: 5 periods is ~189 ms at 26.4 Hz.
				transferring_timer = HAL_GetTick();
				PT_WAIT_UNTIL(pt, (HAL_GetTick() - transferring_timer) > 190);

				// Discard packets until the START of a segment.
				//
				// This previously stopped at the first non-discard packet, which
				// is not the same thing. VoSPI only guarantees alignment from
				// packet 0; stopping at, say, packet 30 meant the bulk read that
				// follows ran off the end of the segment into the inter-segment
				// gap, where MISO idles high and the buffer tail fills with 0xFF.
				// That is exactly the last_end_line == 255 seen on a wedged unit,
				// with a valid segment number still readable at line 20. The
				// frame then failed validation, triggered another resync, and the
				// cycle repeated forever - 1170 rejected frames out of 1170, and
				// no recovery short of a power cycle.
				//
				// The header alone is not enough to say "packet 0", because a
				// sensor that has stopped driving MISO reads back as 0x0000,
				// which passes the packet-0 test. Measured on a wedged unit:
				// the walk consumed ~220 discard packets, hit a silent stretch,
				// declared sync on 0x0000, read 59 more packets of nothing and
				// failed validation - 180 times in one 543-read window, never
				// once seeing a real packet. Requiring a non-zero CRC word
				// rejects silence without rejecting anything the sensor sends.
				resync_tries = 0;
				do {
					g_dbg.resync_packets++;
					lepton_transfer(current_buffer, 1);

					transferring_timer = HAL_GetTick();
					PT_YIELD_UNTIL(pt, current_buffer->status != LEPTON_STATUS_TRANSFERRING || ((HAL_GetTick() - transferring_timer) > 200));

					last_header = (g_format_y16 ?
							current_buffer->lines.y16[0].header[0] :
							current_buffer->lines.rgb[0].header[0]);
					last_crc    = (g_format_y16 ?
							current_buffer->lines.y16[0].header[1] :
							current_buffer->lines.rgb[0].header[1]);

					// Bounded: requiring packet 0 means we could otherwise spin
					// here indefinitely if the sensor never presents one.
					if (++resync_tries > RESYNC_MAX_PACKETS)
					{
						g_dbg.resync_giveups++;
						break;
					}

				} while (current_buffer->status == LEPTON_STATUS_OK &&
				         (((last_header & 0x0f00) == 0x0f00) ||   /* discard packet */
				          ((last_header & 0x00ff) != 0x0000) ||   /* not packet 0 */
				          (last_crc == 0x0000)));                 /* nothing on the wire */

				// we picked up the start of a new packet, so read the rest of it in
				lepton_transfer(current_buffer, IMAGE_NUM_LINES + g_telemetry_num_lines - 1);

				transferring_timer = HAL_GetTick();
				PT_YIELD_UNTIL(pt, current_buffer->status != LEPTON_STATUS_TRANSFERRING || ((HAL_GetTick() - transferring_timer) > 200));

				// Make sure we're not about to service an old irq when the interrupts are re-enabled
				// EXTI->PR takes a PIN MASK, not an IRQ number. Passing EXTI15_10_IRQn
				// (40 = 0x28) cleared lines 3 and 5 and left the Lepton's line 13 pending,
				// so the stale edge fired the instant the IRQ was re-enabled.
				__HAL_GPIO_EXTI_CLEAR_IT(LEPTON_GPIO3_Pin);
				HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);

				current_frame_count = 0;
			}

			current_buffer = NULL;

			continue;
		}

		// This frame validated, so whatever went wrong has cleared.
		consecutive_desyncs = 0;
		if (escape_firings)
		{
			g_dbg.wedge_recoveries++;
			g_dbg.last_recovery_firings = escape_firings;
			escape_firings = 0;
		}

		if (((curtick = HAL_GetTick()) - last_tick) > 3000)
		{
#ifdef PRINT_FPS
			DEBUG_PRINTF("fps: %lu, last end line: %d, frame #%lu, buffer %p\r\n",
				(current_frame_count - last_logged_count) / 3,
				last_end_line,
				current_frame_count, current_buffer
			);
#endif


			if (g_telemetry_num_lines > 0 && g_lepton_type_3 == 0)
			{
				if (g_format_y16)
					print_telemetry_temps(&current_buffer->lines.y16[TELEMETRY_OFFSET_LINES].data.telemetry_data);
				else
					print_telemetry_temps(&current_buffer->lines.rgb[TELEMETRY_OFFSET_LINES].data.telemetry_data);
			}

#if defined(TMP007)
			read_tmp007_regs();
#endif

			last_tick = curtick;
			last_logged_count = current_frame_count;
		}

		// Need to update completed buffer for clients?
		if (g_lepton_type_3 == 0 || (current_segment > 0 && current_segment <= 4))
		{
			static int row;

			completed_buffer = current_buffer;
			completed_frame_count = current_frame_count;

			HAL_GPIO_TogglePin(SYSTEM_LED_GPIO_Port, SYSTEM_LED_Pin);

			if (!g_format_y16)
			{
				for (row = 0; row < (IMAGE_NUM_LINES + g_telemetry_num_lines); row++)
				{
					uint16_t* lineptr = (uint16_t*)completed_buffer->lines.rgb[IMAGE_OFFSET_LINES + row].data.image_data;
					while (lineptr < (uint16_t*)&completed_buffer->lines.rgb[IMAGE_OFFSET_LINES + row].data.image_data[FRAME_LINE_LENGTH])
					{
					  uint8_t* bytes = (uint8_t*)lineptr;
					  *lineptr++ = bytes[0] << 8 | bytes[1];
					}
					PT_YIELD(pt);
				}
			}

			if (!full(CIRC_BUF_HANDLE(completed_frames_buf)))
			{
				push(CIRC_BUF_HANDLE(completed_frames_buf), completed_buffer);
				g_dbg.frames_completed++;
			}
			else
			{
				g_dbg.frames_dropped++;
			}
		}

		current_buffer = NULL;
	}
	PT_END(pt);
}

static inline uint8_t clamp (float x)
{
  if (x < 0)         return 0;
  else if (x > 255)  return 255;
  else               return (uint8_t)x;
}

void rgb2yuv(const rgb_t val, uint8_t *y, uint8_t *u, uint8_t *v)
{
	float r = val.r, g = val.g, b = val.b;

	float y1 = 0.299f * r + 0.587f * g + 0.114f * b;

	*y =   clamp (0.859f *      y1  +  16.0f);
	if (u)
		*u = clamp (0.496f * (b - y1) + 128.0f);
	if (v)
		*v = clamp (0.627f * (r - y1) + 128.0f);
}
