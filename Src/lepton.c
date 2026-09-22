#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"

#include "lepton.h"
#include "dbg_counters.h"

#include "project_config.h"

#if defined(USART_DEBUG) || defined(GDB_SEMIHOSTING)
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

#define LEPTON_USART_PORT (USART2)

#define LEPTON_RESET_L_HIGH	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET)
#define LEPTON_RESET_L_LOW	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET)

#define LEPTON_PW_DWN_HIGH	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define LEPTON_PW_DWN_LOW	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)

extern SPI_HandleTypeDef hspi2;

// These replace HAL library functions as they're a lot shorter and more specialized
static inline HAL_StatusTypeDef start_lepton_spi_dma(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
static inline HAL_StatusTypeDef setup_lepton_spi_rx(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
static void lepton_spi_rx_dma_cplt(DMA_HandleTypeDef *hdma);

void 	init_lepton_task();

lepton_status complete_lepton_transfer(lepton_buffer* buffer)
{
  // TODO: additional synchronization desired?
  return buffer->status;
}

void lepton_transfer(lepton_buffer *buf, int nlines)
{
  HAL_StatusTypeDef status;

  // DEBUG_PRINTF("Transfer starting: %p@%p\r\n", buf, packet);

  int packet_size = FRAME_HEADER_LENGTH +
		  ((g_format_y16 ? sizeof(uint16_t) : sizeof(rgb_t)) * FRAME_LINE_LENGTH) / sizeof(uint16_t);
  status = setup_lepton_spi_rx(&hspi2, buf->lines.data, packet_size * nlines);

  if (status != HAL_OK)
  {
    DEBUG_PRINTF("Error setting up SPI DMA receive: %d\r\n", status);
    buf->status = LEPTON_STATUS_RESYNC;
    return;
  }

  buf->status = LEPTON_STATUS_TRANSFERRING;
}

/* /CS is PB12, configured as SPI2_NSS in hardware-output mode. On the F4 that
 * holds /CS low for as long as SPE = 1, and SPE is set once in lepton_init()
 * and never cleared, so without this nothing after boot ever deasserts /CS -
 * and /CS high with SCK idle for > 185 ms is the only documented way to put
 * the Lepton back into a state where it can establish VoSPI sync.
 *
 * lepton_cs_release() borrows PB12 as a GPIO output driven high. SPI2 stays
 * enabled and configured and PB12's alternate-function selection (AF5) is
 * untouched, so lepton_cs_restore() only has to hand the pin back, at which
 * point /CS drops low again. GPIOB also carries the sensor's power enables
 * (PB5, PB7) and I2C1 (PB8, PB9): only PB12's MODER bits are modified.
 *
 * The caller keeps SCK idle between the two calls (no lepton_transfer()). */
#define LEPTON_CS_PIN_POS  (12u)
#define LEPTON_CS_MODER_MASK (3u << (2u * LEPTON_CS_PIN_POS))

void lepton_cs_release(void)
{
  uint32_t primask;
  uint32_t t0 = HAL_GetTick();

  /* Nothing should be in flight when this is called. If something is, let
   * it finish rather than cut a packet in half; bounded so it cannot hang. */
  while (((hspi2.hdmarx->Instance->CR & DMA_SxCR_EN) ||
          (hspi2.Instance->SR & SPI_SR_BSY)) &&
         (HAL_GetTick() - t0) < 5) {}

  primask = __get_PRIMASK();
  __disable_irq();
  GPIOB->BSRR = (1u << LEPTON_CS_PIN_POS);                     /* ODR12 = 1 first */
  GPIOB->MODER = (GPIOB->MODER & ~LEPTON_CS_MODER_MASK)
               | (1u << (2u * LEPTON_CS_PIN_POS));             /* then output: /CS high */
  __set_PRIMASK(primask);

  /* Prove the pin actually went high. If something outside the MCU holds /CS
   * low this whole recovery is a no-op, and silence about that would send the
   * next person hunting the wrong fault. */
  for (volatile int i = 0; i < 32; i++) {}
  if ((GPIOB->IDR & (1u << LEPTON_CS_PIN_POS)) == 0)
    g_dbg.cs_stuck_low++;
}

void lepton_cs_restore(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  GPIOB->MODER = (GPIOB->MODER & ~LEPTON_CS_MODER_MASK)
               | (2u << (2u * LEPTON_CS_PIN_POS));             /* AF5 SPI2_NSS: /CS low */
  __set_PRIMASK(primask);
}

/* Hardware reset of the sensor, the same pin sequence lepton_init() uses at
 * boot: RESET_L and PWR_DWN_L low, PWR_DWN_L high after >= 190 ms, RESET_L
 * high after another >= 190 ms. Split into steps so lepton_task can yield
 * between them. The sensor loses all CCI configuration; see
 * lepton_reinit_after_reset(). */
void lepton_hw_reset_assert(void)
{
  LEPTON_RESET_L_LOW;
  LEPTON_PW_DWN_LOW;
}

void lepton_hw_pwdn_release(void)
{
  LEPTON_PW_DWN_HIGH;
}

void lepton_hw_reset_release(void)
{
  LEPTON_RESET_L_HIGH;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  DEBUG_PRINTF("SPI error!\n\r");
}

static void lepton_spi_rx_dma_cplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  lepton_buffer *buffer = (lepton_buffer*)hspi->pRxBuffPtr;

  /* Disable Rx/Tx DMA Requests and reset some peripheral state */
  hspi->Instance->CR2 &= (uint32_t)(~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN));
  hspi->TxXferCount = hspi->RxXferCount = 0;
  hspi->State = HAL_SPI_STATE_READY;

  buffer->status = LEPTON_STATUS_OK;
}

void lepton_init(void )
{
	LEPTON_RESET_L_LOW;
  LEPTON_PW_DWN_LOW;

  HAL_Delay(190);
  LEPTON_PW_DWN_HIGH;

	HAL_Delay(190);
  LEPTON_RESET_L_HIGH;

  hspi2.hdmarx->XferCpltCallback = lepton_spi_rx_dma_cplt;

  /* Set the SPI Tx DMA transfer complete callback as NULL because the communication closing
  is performed in DMA reception complete callback  */
  hspi2.hdmatx->XferCpltCallback = NULL;
  hspi2.hdmatx->XferErrorCallback = NULL;

  /* Clear DBM bit */
  hspi2.hdmarx->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);
  hspi2.hdmatx->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

  /*Init field not used in handle to zero */
  hspi2.RxISR = 0;
  hspi2.TxISR = 0;

  /* Enable SPI peripheral */
  __HAL_SPI_ENABLE(&hspi2);

  init_lepton_task();
}

static inline HAL_StatusTypeDef start_lepton_spi_dma(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  hdma->Instance->CR &= ~DMA_SxCR_EN;

  /* Configure DMA Stream data length */
  hdma->Instance->NDTR = DataLength;

  /* Memory to Peripheral */
  if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
  {
    /* Configure DMA Stream destination address */
    hdma->Instance->PAR = DstAddress;

    /* Configure DMA Stream source address */
    hdma->Instance->M0AR = SrcAddress;
  }
  /* Peripheral to Memory */
  else
  {
    /* Configure DMA Stream source address */
    hdma->Instance->PAR = SrcAddress;

    /* Configure DMA Stream destination address */
    hdma->Instance->M0AR = DstAddress;
  }

  hdma->Instance->CR |= (DMA_IT_TC | DMA_SxCR_EN);

  return HAL_OK;
}

static inline HAL_StatusTypeDef setup_lepton_spi_rx(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
  /* Configure communication */
  hspi->State       = HAL_SPI_STATE_BUSY_RX;
  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;

  hspi->pTxBuffPtr  = hspi->pRxBuffPtr  = (uint8_t*)pData;
  hspi->TxXferSize  = hspi->RxXferSize  = Size;
  hspi->TxXferCount = hspi->RxXferCount = Size;

  /* Enable the Tx DMA Stream */
  start_lepton_spi_dma(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->DR, hspi->TxXferCount);

  /* Enable the Rx DMA Stream */
  start_lepton_spi_dma(hspi->hdmarx, (uint32_t)&hspi->Instance->DR, (uint32_t)hspi->pRxBuffPtr, hspi->RxXferCount);

  /* Enable Rx DMA Request */
  /* Enable Tx DMA Request */
  hspi->Instance->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;

  return HAL_OK;

}
