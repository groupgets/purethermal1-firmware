/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */

#include "lepton.h"
#include "lepton_i2c.h"
#include "tmp007_i2c.h"
#include "usbd_uvc.h"
#include "usbd_uvc_if.h"
DMA_HandleTypeDef hdma_memtomem_dma2_stream0;

//#define THERMAL_DATA_UART

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
  
uint8_t lepton_raw[60*80*2];
uint8_t packet[VIDEO_PACKET_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#define WHITE_LED_TOGGLE  (HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6))
extern volatile int restart_frame;
#ifdef USART_DEBUG
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

void HAL_RCC_CSSCallback(void) {
  DEBUG_PRINTF("Oh no! HAL_RCC_CSSCallback()\r\n");
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  DEBUG_PRINTF("Yay! HAL_GPIO_EXTI_Callback()\r\n");
}

extern volatile uint8_t uvc_stream_status;
extern USBD_UVC_VideoControlTypeDef videoCommitControl;

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  uint16_t val;
  uint16_t count = 0, i,j;
  int frames = 0;
  uint32_t last_tick = HAL_GetTick();
  lepton_buffer *current_buffer;

  uint8_t uvc_header[2] = { 2, 0 };
  uint32_t uvc_xmit_row = 0, uvc_xmit_plane = 0;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

  DEBUG_PRINTF("Hello, Lepton!\n\r");
  fflush(stdout);
  lepton_init();
  DEBUG_PRINTF("Initialized...\n\r");

  HAL_Delay(1000);

  read_lepton_regs();
  read_tmp007_regs();

  // kick off the first transfer
  current_buffer = lepton_transfer();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

    while (complete_lepton_transfer(current_buffer) == LEPTON_STATUS_TRANSFERRING){
      HAL_Delay(1);
      fflush(stdout);
    }

    if ((current_buffer->status & LEPTON_STATUS_RESYNC) == LEPTON_STATUS_RESYNC)
    {
      //DEBUG_PRINTF("Synchronization lost\r\n");
      read_tmp007_regs();
      HAL_Delay(200);
    }

    if ((frames % 30) == 0)
    {
      uint32_t curtick = HAL_GetTick();
      uint32_t ticks = curtick - last_tick;
      last_tick = curtick;
      DEBUG_PRINTF("ms / frame: %lu, last end line: %d\r\n", ticks / 30, current_buffer->data[82*59] & 0xff);
      read_tmp007_regs();
    }

    current_buffer = lepton_transfer();
    frames++;
    WHITE_LED_TOGGLE;

#ifdef THERMAL_DATA_UART 
    if ((frames % 3) == 0)
    {
      count = 0;
      for (j = 0; j < 60; j++)
      {
        for (i = 2; i < 82; i++)
        {
          val = current_buffer->data[j * 82 + i];

          lepton_raw[count++] = ((val>>8)&0xff);
          lepton_raw[count++] = (val&0xff);
        }
      }

      send_lepton_via_usart(lepton_raw);
    }
#endif

    // perform stream initialization
    if (uvc_stream_status == 1)
    {
      DEBUG_PRINTF("Starting UVC stream...\r\n");

      uvc_header[0] = 2;
      uvc_header[1] = 0;
      UVC_Transmit_FS(uvc_header, 2);

      uvc_stream_status = 2;
      uvc_xmit_row = 0;
      uvc_xmit_plane = 0;
    }

    // put image on stream as long as stream is open
    while (uvc_stream_status == 2)
    {
      count = 0;

      packet[count++] = uvc_header[0];
      packet[count++] = uvc_header[1];

      switch (videoCommitControl.bFormatIndex[0])
      {
        // HACK: NV12 is semi-planar but YU12/I420 is planar. Deal with it when we have actual color.
        default:
        case FMT_INDEX_NV12:
        case FMT_INDEX_YU12:
        {
          // printf("Writing format 1, plane %d...\r\n", uvc_xmit_plane);

          switch (uvc_xmit_plane)
          {
            default:
            case 0: // Y
            {
              // while (uvc_xmit_row < 60 && count < VALDB(videoCommitControl.dwMaxPayloadTransferSize))
              while (uvc_xmit_row < 60 && count < VIDEO_PACKET_SIZE)
              {
                for (i = 2; i < 82; i++)
                {
                  uint16_t val = current_buffer->data[uvc_xmit_row * 82 + i];

                  // Don't bother scaling the data, just center around 8192 (lepton core temperature)
                  if (val <= 8064)
                    val = 0;
                  else if (val >= 8320)
                    val = 255;
                  else
                    val -= 8064;

                  packet[count++] = (uint8_t)val;
                }

                uvc_xmit_row++;
              }

              if (uvc_xmit_row == 60)
              {
                uvc_xmit_plane = 1;
                uvc_xmit_row = 0;
              }

              break;
            }
            case 1: // VU
            case 2:
            {
              if (videoCommitControl.bFormatIndex[0] == FMT_INDEX_NV12)
              {
                while (uvc_xmit_row < 30 && count < VIDEO_PACKET_SIZE)
                {
                  for (i = 0; i < 80 && uvc_xmit_row < 30 && count < VIDEO_PACKET_SIZE; i++)
                    packet[count++] = 128;

                  uvc_xmit_row++;
                }

                if (uvc_xmit_row == 30)
                  packet[1] |= 0x2; // Flag end of frame
              }
              else
              {
                while (uvc_xmit_row < 30 && count < VIDEO_PACKET_SIZE)
                {
                  for (i = 0; i < 40 && uvc_xmit_row < 30 && count < VIDEO_PACKET_SIZE; i++)
                    packet[count++] = 128;

                  uvc_xmit_row++;
                }

                // image is done
                if (uvc_xmit_row == 30)
                {
                  if (uvc_xmit_plane == 1)
                  {
                    uvc_xmit_plane = 2;
                    uvc_xmit_row = 0;
                  }
                  else
                  {
                    packet[1] |= 0x2; // Flag end of frame
                  }
                }
              }
              break;
            }
          }
  
          break;
        }
        case FMT_INDEX_GREY:
        {
          // while (uvc_xmit_row < 60 && count < VALDB(videoCommitControl.dwMaxPayloadTransferSize))
          while (uvc_xmit_row < 60 && count < VIDEO_PACKET_SIZE)
          {
            for (i = 2; i < 82; i++)
            {
              uint16_t val = current_buffer->data[uvc_xmit_row * 82 + i];

              // Don't bother scaling the data, just center around 8192 (lepton core temperature)
              if (val <= 8064)
                val = 0;
              else if (val >= 8320)
                val = 255;
              else
                val -= 8064;

              packet[count++] = (uint8_t)val;
            }

            uvc_xmit_row++;
          }

          // image is done
          if (uvc_xmit_row == 60)
          {
            packet[1] |= 0x2; // Flag end of frame
          }

          break;
        }
        case FMT_INDEX_Y16:
        {
          // while (uvc_xmit_row < 60 && count < VALDB(videoCommitControl.dwMaxPayloadTransferSize))
          while (uvc_xmit_row < 60 && count < VIDEO_PACKET_SIZE)
          {
            for (i = 2; i < 82; i++)
            {
              uint16_t val = current_buffer->data[uvc_xmit_row * 82 + i];
              packet[count++] = (uint8_t)((val >> 0) & 0xFF);
              packet[count++] = (uint8_t)((val >> 8) & 0xFF);
            }

            uvc_xmit_row++;
          }

          // image is done
          if (uvc_xmit_row == 60)
          {
            packet[1] |= 0x2; // Flag end of frame
          }

          break;
        }
        case FMT_INDEX_YUYV:
        {
          // while (uvc_xmit_row < 60 && count < VALDB(videoCommitControl.dwMaxPayloadTransferSize))
          while (uvc_xmit_row < 60 && count < VIDEO_PACKET_SIZE)
          {
            for (i = 2; i < 82; i++)
            {
              uint16_t val = current_buffer->data[uvc_xmit_row * 82 + i];

              // Don't bother scaling the data, just center around 8192 (lepton core temperature)
              if (val <= 8064)
                val = 0;
              else if (val >= 8320)
                val = 255;
              else
                val -= 8064;

              packet[count++] = (uint8_t)val;
              packet[count++] = 128;
            }

            uvc_xmit_row++;
          }

          // image is done
          if (uvc_xmit_row == 60)
          {
            packet[1] |= 0x2; // Flag end of frame
          }

          break;
        }
      }

      // printf("UVC_Transmit_FS(): packet=%p, count=%d\r\n", packet, count);
      // fflush(stdout);

      while (UVC_Transmit_FS(packet, count) == USBD_BUSY && uvc_stream_status == 2)
        ;

      if (packet[1] & 0x2)
      {
        uvc_header[1] ^= 1; // toggle bit 0 for next new frame
        uvc_xmit_row = 0;
        uvc_xmit_plane = 0;
        // DEBUG_PRINTF("Frame complete\r\n");
        break;
      }
    }

    fflush(stdout);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 11;
  RCC_OscInitStruct.PLL.PLLN = 295;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);

  HAL_RCC_EnableCSS();

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* CRC init function */
void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  HAL_CRC_Init(&hcrc);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

}

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi2.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi2);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
#ifdef USART_DEBUG
  huart2.Init.BaudRate = USART_DEBUG_SPEED;
#else
  huart2.Init.BaudRate = 921600;
#endif
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** 
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();
  __DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
  hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
  hdma_memtomem_dma2_stream0.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_MEDIUM;
  hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
  HAL_DMA_Init(&hdma_memtomem_dma2_stream0);

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PB0   ------> S_TIM3_CH3
     PB1   ------> S_TIM3_CH4
     PB10   ------> S_TIM2_CH3
     PB6   ------> S_TIM4_CH1
     PB7   ------> S_TIM4_CH2
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA5 PA6 
                           PA7 PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
