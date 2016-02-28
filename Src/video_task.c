#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal.h"

#include "pt.h"
#include "lepton.h"
#include "tasks.h"
#include "project_config.h"

//*****************************************************************************
//
// PT1 video output notes
//
// If you have a PT1 board from groupget round 2 or earlier, your board needs
// a resistor modification to do this. Round 2 boards also have the video
// connector soldered on upside down. Round 1 boards are green, round 2 boards
// are blue. Round 3 fixes these issues, and if you were on this group buy or
// if you purchased from the store starting in Mar 2016, you're good. These
// boards are also blue.
//
// See the development wiki for information about how to test and modify your
// board to make video output function:
// https://github.com/groupgets/purethermal1-firmware/wiki/NTSC-Video-Out
//
//*****************************************************************************

extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_tim1_up;
extern DMA_HandleTypeDef hdma_tim1_ch1;

static uint32_t line = 0;
static uint32_t frame = 0;
struct pt_sem sem_vid_next_line;

__ALIGN_BEGIN uint16_t vsync[455] __ALIGN_END = { 0 };
__ALIGN_BEGIN uint16_t burstline[2][455] __ALIGN_END = { { 0 } };

__ALIGN_BEGIN uint16_t vid_data[2][455] __ALIGN_END = { { 0 } };
__ALIGN_BEGIN uint32_t phase_data[2][455] __ALIGN_END = { { 0 } };

static uint16_t *next_vid_line, *next_phase_line;

#define VIDEO_FRAME_NUM_LINES (526)
#define VIDEO_FIELD_NUM_LINES (263)

#define VIDEO_FIRST_VISIBLE_LINE (23)
#define VIDEO_VISIBLE_LINES (240)
#define VIDEO_VISIBLE_COLS (320)

#define VIDEO_IMAGE_VERTICAL_UPSAMPLE (VIDEO_VISIBLE_LINES / IMAGE_NUM_LINES)
#define VIDEO_IMAGE_HORIZONTAL_UPSAMPLE (VIDEO_VISIBLE_COLS / FRAME_LINE_LENGTH)

#define SYNC (4)
#define COLOR_AMPL (2)
#define GPIO_VALUE_SIZE (16)
static uint16_t GPIO_VALUES[GPIO_VALUE_SIZE] = { 0 };

static inline void build_line(uint16_t* lptr, int even)
{
  int i = 0;

  for (; i<34; i++)
  {
    lptr[i] = GPIO_VALUES[0];
  }

  // IRE = 0
  for (; i<(34+5); i++)
  {
    lptr[i] = GPIO_VALUES[SYNC];
  }

  if (even)
  {
    // color burst
    for (; i<(34+5+17); i++)
    {
      if ((i % 2) == 0)
        lptr[i] = GPIO_VALUES[SYNC - COLOR_AMPL];
      else
        lptr[i] = GPIO_VALUES[SYNC + COLOR_AMPL];
    }
  }
  else
  {
    for (; i<(34+5+17); i++)
    {
      if ((i % 2) == 0)
        lptr[i] = GPIO_VALUES[SYNC + COLOR_AMPL];
      else
        lptr[i] = GPIO_VALUES[SYNC - COLOR_AMPL];
    }
  }

  for (; i<455; i++)
  {
    lptr[i] = GPIO_VALUES[SYNC];
  }

}

static inline void build_vsync(uint16_t* lptr)
{
  int i = 0;

  for (; i<34; i++)
  {
    lptr[i] = GPIO_VALUES[SYNC];
  }

  // IRE = 0
  for (; i<(455); i++)
  {
    lptr[i] = GPIO_VALUES[0];
  }
}

PT_THREAD( video_task(struct pt *pt))
{
	PT_BEGIN(pt);

  static uint16_t i;
  static uint16_t* lptr;
  static uint16_t field_line;

  while (1)
  {
    PT_SEM_WAIT(pt, &sem_vid_next_line);

    if (line == 0)
      frame++;

    field_line = line % VIDEO_FIELD_NUM_LINES;

    if (field_line < 3) {
      next_vid_line = vsync;
    }
    else if (field_line < 5 || field_line >= (VIDEO_FIELD_NUM_LINES - 3))
    {
      next_vid_line = burstline[line % 2];
    }

#ifdef VIDEO_TEST_PATTERN
    else if (field_line > 20)
    {
      lptr = next_line = vid_data[line % 2];

      for (i = 34+5+17+11; i < (444); i++)
      {
        uint16_t value;

        value = SYNC + 4; //((i >> 4) % (GPIO_VALUE_SIZE - SYNC)); //SYNC + 4;

        if ((line % 2) == 0)
        {
          if ((i%2) == 1)
            value += 2;
          else
            value -= 2;
        }
        else
        {
          if ((i%2) == 1)
            value -= 2;
          else
            value += 2;
        }

        // if (i > 100 && i < 150)
        //   value = SYNC + 6;

        // if (value >= GPIO_VALUE_SIZE)
        //   value = GPIO_VALUE_SIZE - 1;
        // else if (value < 0)
        //   value = 0;

        lptr[i] = GPIO_VALUES[value];
      }
    }
#else

    else if (field_line >= VIDEO_FIRST_VISIBLE_LINE &&
             field_line < (VIDEO_FIRST_VISIBLE_LINE + VIDEO_VISIBLE_LINES))
    {
      lepton_buffer *last_buffer;
      uint16_t lepton_row;
#ifdef Y16
      uint16_t *image_data;
#else
      rgb_t *image_data;
#endif
      get_lepton_buffer(&last_buffer);

      lepton_row = (field_line - VIDEO_FIRST_VISIBLE_LINE) / VIDEO_IMAGE_VERTICAL_UPSAMPLE;
#ifdef Y16
      image_data = last_buffer->lines[IMAGE_OFFSET_LINES + lepton_row].data.image_data;
#else
      image_data = last_buffer->lines[IMAGE_OFFSET_LINES + lepton_row].data.image_data;
#endif

      lptr = next_vid_line = vid_data[line % 2];
      lptr = lptr + 34+5+17+11+25;

      for (i = 0; i < FRAME_LINE_LENGTH; i++)
      {
        int j = VIDEO_IMAGE_HORIZONTAL_UPSAMPLE;

#ifdef Y16
        uint16_t val = image_data[i];
        uint16_t gpio_val = GPIO_VALUES[(val / 20) + 3];
#else
        rgb_t val = image_data[i];
        uint16_t gpio_val = GPIO_VALUES[((val.r + val.b + val.g) / 64) + 4];
#endif

        do {
          *lptr++ = gpio_val;
        } while (--j);
      }
    }

#endif
  }

  PT_END(pt);
}

void TIM_DMA_VidM0Cplt(DMA_HandleTypeDef *hdma)
{
  // update m0ar with the next line up
  htim1.hdma[TIM_DMA_ID_UPDATE]->Instance->M0AR = (uint32_t)next_vid_line;

  // kick off the next line computation
  line = ((line + 1) % VIDEO_FRAME_NUM_LINES);
  PT_SEM_SIGNAL(&lepton_task_pt, &sem_vid_next_line);
}

void TIM_DMA_VidM1Cplt(DMA_HandleTypeDef *hdma)
{
  // update m1ar with the next line up
  htim1.hdma[TIM_DMA_ID_UPDATE]->Instance->M1AR = (uint32_t)next_vid_line;

  // kick off the next line computation
  line = ((line + 1) % VIDEO_FRAME_NUM_LINES);
  PT_SEM_SIGNAL(&lepton_task_pt, &sem_vid_next_line);
}

void TIM_DMA_PhaseM0Cplt(DMA_HandleTypeDef *hdma)
{
  // update m0ar with the next line up
  htim1.hdma[TIM_DMA_ID_CC1]->Instance->M0AR = (uint32_t)next_phase_line;
}

void TIM_DMA_PhaseM1Cplt(DMA_HandleTypeDef *hdma)
{
  // update m1ar with the next line up
  htim1.hdma[TIM_DMA_ID_CC1]->Instance->M1AR = (uint32_t)next_phase_line;
}

static void setup_timers_and_dma(void)
{
  DMA_HandleTypeDef *hdma = htim1.hdma[TIM_DMA_ID_UPDATE];
  DMA_HandleTypeDef *hdma_2 = htim1.hdma[TIM_DMA_ID_CC1];

  // Configure DMA Stream data length
  hdma->Instance->NDTR = 455;
  hdma_2->Instance->NDTR = 455;

  // Configure DMA Stream destination address
  hdma->Instance->PAR = (uint32_t)&GPIOB->ODR;
  hdma_2->Instance->PAR = (uint32_t)&htim1.Instance->ARR;

  // Configure DMA Stream source address
  hdma->Instance->M0AR = (uint32_t)vsync;
  hdma->Instance->M1AR = (uint32_t)vsync;

  hdma_2->Instance->M0AR = (uint32_t)phase_data[0];
  hdma_2->Instance->M1AR = (uint32_t)phase_data[1];

  // Enable the transfer complete interrupt and dma peripheral
  hdma->Instance->CR |= (DMA_IT_TC | DMA_SxCR_EN | DMA_SxCR_CIRC | DMA_SxCR_DBM);
  hdma_2->Instance->CR |= (DMA_IT_TC | DMA_SxCR_EN | DMA_SxCR_CIRC | DMA_SxCR_DBM);

  // enable tim1 dma
  htim1.Instance->DIER |= TIM_DMA_UPDATE;
  htim1.Instance->DIER |= TIM_DMA_CC1;

  // only generate update on overflow
  htim1.Instance->CR1 |= TIM_CR1_URS;
  htim1.Instance->CR1 |= TIM_CR1_ARPE;

  /* Enable the Output compare channel */
  TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
  
  if(IS_TIM_ADVANCED_INSTANCE(htim1.Instance) != RESET)
  {
    /* Enable the main output */
    __HAL_TIM_MOE_ENABLE(&htim1);
  }

  htim1.hdma[TIM_DMA_ID_UPDATE]->XferCpltCallback = TIM_DMA_VidM0Cplt;
  htim1.hdma[TIM_DMA_ID_UPDATE]->XferM1CpltCallback = TIM_DMA_VidM1Cplt;
  htim1.hdma[TIM_DMA_ID_UPDATE]->XferErrorCallback = TIM_DMAError;

  htim1.hdma[TIM_DMA_ID_CC1]->XferCpltCallback = TIM_DMA_PhaseM0Cplt;
  htim1.hdma[TIM_DMA_ID_CC1]->XferM1CpltCallback = TIM_DMA_PhaseM1Cplt;
  htim1.hdma[TIM_DMA_ID_CC1]->XferErrorCallback = TIM_DMAError;
}

void init_video()
{
  int i;

  for (i = 0; i < 16; i++)
  {
    GPIO_VALUES[i] = 0;

    if (i & 1) {
      GPIO_VALUES[i] |= GPIO_PIN_10;
    }

    if (i & 2) {
      GPIO_VALUES[i] |= GPIO_PIN_6;
    }

    if (i & 4) {
      GPIO_VALUES[i] |= GPIO_PIN_1;
    }

    if (i & 8) {
      GPIO_VALUES[i] |= GPIO_PIN_0;
    }
  }

  build_line(vid_data[0], 0);
  build_line(vid_data[1], 1);
  build_line(burstline[0], 0);
  build_line(burstline[1], 1);
  build_vsync(vsync);

  for (i=0; i<455; i++)
  {
    phase_data[0][i] = 14;
    phase_data[1][i] = 14;
  }

#ifdef VIDEO_TEST_PATTERN
  for (i=0; i<28; i++)
  {
    int arr_idx = 70 + i * 14;
    phase_data[0][arr_idx] = phase_data[0][arr_idx] - 1;
    phase_data[1][arr_idx] = phase_data[1][arr_idx] - 1;
  }

  phase_data[0][445] = phase_data[0][445] + 28;
  phase_data[1][445] = phase_data[1][445] + 28;
#endif

  next_vid_line = vsync;
  next_phase_line = phase_data[0];

  PT_SEM_INIT(&sem_vid_next_line, 0);

  setup_timers_and_dma();

  __HAL_TIM_ENABLE(&htim1);
}
