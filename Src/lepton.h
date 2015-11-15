#ifndef LEPTON_H_
#define LEPTON_H_

#define VOSPI_FRAME_SIZE (164)
#define LEPTON_IMAGE_SIZE ((60*80)*2)

typedef enum {
  LEPTON_STATUS_OK = 0,
  LEPTON_STATUS_TRANSFERRING = 1,
  LEPTON_STATUS_RESYNC = 2,
} lepton_status;

typedef struct __attribute__((packed)) _telemetry_data_l2 {

  uint16_t pad_1[2];                  // w xx xx

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

  uint16_t pad_2[2];                  // w xx xx
  uint16_t reserved_6[80];            // w 80-159

  uint16_t pad_3[2];                  // w xx xx
  uint16_t reserved_7[80];            // w 160-239

} telemetry_data_l2;

typedef struct __attribute__((packed)) _telemetry_data_l3 {

  uint16_t pad_1[2];                  // w -2 -1

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

  uint16_t pad_2[2];                  // w xx xx
  uint16_t reserved_7[80];            // w 80-159

  uint16_t pad_3[2];                  // w xx xx
  uint16_t reserved_8[80];            // w 160-239

  uint16_t pad_4[2];                  // w xx xx
  uint16_t reserved_9[80];            // w 240-319

} telemetry_data_l3;

typedef struct _lepton_buffer {
  uint8_t number;
  lepton_status status;
  uint16_t data[VOSPI_FRAME_SIZE/2 * 60];
  telemetry_data_l2 telemetry;
} lepton_buffer;

lepton_status complete_lepton_transfer(lepton_buffer *);
lepton_buffer* lepton_transfer(void);

void print_image_binary_background(void);
void lepton_init(void );

#endif

