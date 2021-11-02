/*
 * stm32_spi.c
 *
 *  Created on: Sep 4, 2020
 *      Author: DFMPC-01201117
 */

#include "spi.h"

#define SPI_TIMEOUT          0x1000U

SPI_HandleTypeDef hspi1;

void SPI_Initialize();;
void SPI_Deinitialize();
void SPI_Transmit(uint8_t *data, uint32_t size);
void SPI_Receive(uint8_t *data, uint32_t size);
void SPI_TransmitReceive(uint8_t *Txdata, uint8_t *Rxdata, uint8_t size);

void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;


  if (HAL_SPI_Init(&hspi1) != HAL_OK)
   {
     Error_Handler();
   }
   /* USER CODE BEGIN SPI1_Init 2 */

   /* USER CODE END SPI1_Init 2 */
}
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA15     ------> SPI1_NSS
    PB3     ------> SPI1_SCK
    PB4     ------> SPI1_MISO
    PA5     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

void SPI_Transfer(uint8_t *Txdata, uint32_t size)
{
	HAL_SPI_Transmit(&hspi1, Txdata, size, SPI_TIMEOUT);
}

void SPI_Receive(uint8_t *data, uint32_t size)
{

}

void SPI_TransmitReceive(uint8_t *Txdata, uint8_t *Rxdata, uint8_t size)
{
	HAL_SPI_TransmitReceive(&hspi1, Txdata, Rxdata, size, SPI_TIMEOUT);
}

void SPI_ConfigureBusTxFer(uint8_t *data, uint32_t size)
{

}
