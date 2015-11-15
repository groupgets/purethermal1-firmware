#ifndef LEPTON_H_
#define LEPTON_H_

#define VOSPI_FRAME_SIZE (164)

#define LEPTON_ROWS	(60)
#define LEPTON_COLS	(80)
#define LEPTON_IMAGE_SIZE ((LEPTON_ROWS*LEPTON_COLS))
#define LEPTON_IMAGE_BYTE_SIZE ((LEPTON_IMAGE_SIZE)*2)
#define TELEMETRY_SIZE	(80)

typedef enum {
  LEPTON_STATUS_OK = 0,
  LEPTON_STATUS_TRANSFERRING = 1,
  LEPTON_STATUS_RESYNC = 2,
} lepton_status;

typedef struct _lepton_buffer {
  uint8_t number;
  lepton_status status;
  uint16_t data[VOSPI_FRAME_SIZE/2 * 60 + TELEMETRY_SIZE];
} lepton_buffer;

lepton_status complete_lepton_transfer(lepton_buffer *);
lepton_buffer* lepton_transfer(void);

void print_image_binary_background(void);
void lepton_init(void );

#endif

