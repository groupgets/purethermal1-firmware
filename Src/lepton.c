#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"

#include "lepton.h"

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
