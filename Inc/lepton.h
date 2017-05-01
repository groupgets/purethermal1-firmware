#ifndef LEPTON_H_
#define LEPTON_H_

#include "project_config.h"

#define FRAME_HEADER_LENGTH (2)
#define FRAME_LINE_LENGTH (80)
#ifdef Y16
#define FRAME_TOTAL_LENGTH (FRAME_HEADER_LENGTH + FRAME_LINE_LENGTH)
#else
#define FRAME_TOTAL_LENGTH (FRAME_HEADER_LENGTH + FRAME_LINE_LENGTH + 40)
#endif
#define FRAME_TOTAL_SIZE_Y16 (FRAME_TOTAL_LENGTH * sizeof(uint16_t) * 1)
#define FRAME_TOTAL_SIZE_RGB (FRAME_TOTAL_LENGTH * sizeof(uint8_t) * 3)
#define IMAGE_NUM_LINES (60)
#ifdef Y16
  #ifndef LEPTON2 /* This is for Lepton 3 */
#define TELEMETRY_NUM_LINES (1)
  #else /* This is for Lepton 2 */
#define TELEMETRY_NUM_LINES (3)
  #endif
#else
#define TELEMETRY_NUM_LINES (0)
#endif
#define IMAGE_OFFSET_LINES (0)
#define TELEMETRY_OFFSET_LINES (IMAGE_NUM_LINES)

typedef enum {
  LEPTON_STATUS_OK = 0,
  LEPTON_STATUS_TRANSFERRING = 1,
  LEPTON_STATUS_RESYNC = 2,
  LEPTON_STATUS_CONTINUE = 4,
} lepton_status;

typedef enum {
  LEPTON_XFER_STATE_START = 0,
  LEPTON_XFER_STATE_SYNC = 1,
  LEPTON_XFER_STATE_DATA = 2,
} lepton_xfer_state;

typedef struct __attribute__((packed)) _telemetry_data_l2 {

  uint16_t telemetry_revision[1];     // w  0
  uint16_t time_ms[2];                // w  1  2
  uint16_t status_bits[2];            // w  3  4
  uint16_t serial_number[8];          // w  5  6  7  8  9 10 11 12
  uint16_t software_rev[4];           // w 13 14 15 16
  uint16_t reserved_1[3];             // w 17 18 19
  uint16_t frame_counter[2];          // w 20 21
  uint16_t frame_mean[1];             // w 22
  uint16_t fpa_temp_counts[1];        // w 23
  uint16_t fpa_temp_100k[1];          // w 24
  uint16_t housing_temp_counts[1];    // w 25
  uint16_t housing_temp_100k[1];      // w 26
  uint16_t reserved_2[2];             // w 27 28
  uint16_t fpa_temp_ffc_100k[1];      // w 29
  uint16_t time_ffc_ms[2];            // w 30 31
  uint16_t housing_temp_ffc_100k[1];  // w 32
  uint16_t reserved_3[1];             // w 33
  uint16_t agc_roi[4];                // w 34 35 36 37
  uint16_t agc_clip_limit_h[1];       // w 38
  uint16_t agc_clip_limit_l[1];       // w 39
  uint16_t reserved_4[34];            // w 40-73
  uint16_t log2_ffc_frames[1];        // w 74
  uint16_t reserved_5[5];             // w 75 76 77 78 79

} telemetry_data_l2;

typedef struct __attribute__((packed)) _telemetry_data_l3 {

  uint16_t telemetry_revision[1];     // w  0
  uint16_t time_ms[2];                // w  1  2
  uint16_t status_bits[2];            // w  3  4
  uint16_t reserved_1[8];             // w  5  6  7  8  9 10 11 12
  uint16_t software_rev[4];           // w 13 14 15 16
  uint16_t reserved_2[3];             // w 17 18 19
  uint16_t frame_counter[2];          // w 20 21
  uint16_t frame_mean[1];             // w 22
  uint16_t fpa_temp_counts[1];        // w 23
  uint16_t fpa_temp_100k[1];          // w 24
  uint16_t reserved_3[4];             // w 25 26 27 28
  uint16_t fpa_temp_ffc_100k[1];      // w 29
  uint16_t time_ffc_ms[2];            // w 30 31
  uint16_t reserved_4[2];             // w 32 33
  uint16_t agc_roi[4];                // w 34 35 36 37
  uint16_t agc_clip_limit_h[1];       // w 38
  uint16_t agc_clip_limit_l[1];       // w 39
  uint16_t reserved_5[32];            // w 40-71
  uint16_t video_output_format[2];    // w 72 73
  uint16_t reserved_6[7];             // w 74 75 76 77 78 79

} telemetry_data_l3;

typedef struct __attribute__((packed)) _rgb {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} rgb_t;

#ifdef Y16
typedef struct __attribute__((packed)) _vospi_packet {
  uint16_t header[2];
  union {
    uint16_t image_data[FRAME_LINE_LENGTH];
    telemetry_data_l2 telemetry_data;
  } data;
} vospi_packet;
#else
typedef struct __attribute__((packed)) _vospi_packet {
  uint16_t header[2];
  union {
    rgb_t image_data[FRAME_LINE_LENGTH];
    telemetry_data_l2 telemetry_data;
  } data;
} vospi_packet;
#endif

typedef struct _lepton_buffer {
  vospi_packet lines[IMAGE_NUM_LINES + TELEMETRY_NUM_LINES];
  uint8_t number;
  lepton_status status;
} lepton_buffer;

#define NUM_SEGMENTS (4)
typedef lepton_buffer frame_buffer[NUM_SEGMENTS];


typedef struct __attribute__((packed)) _yuv422 {
  uint8_t uv;
  uint8_t y;
} yuv422_t;

typedef yuv422_t yuv422_row_t[FRAME_LINE_LENGTH];

typedef struct __attribute__((packed)) _yuv422_buffer {
  yuv422_row_t data[IMAGE_NUM_LINES];
} yuv422_buffer_t;

void set_current_lepton_buffer(lepton_buffer *buff);
lepton_status complete_lepton_transfer(lepton_buffer *);
lepton_buffer* lepton_transfer(void);

void print_image_binary_background(void);
void lepton_init(void );

#endif

