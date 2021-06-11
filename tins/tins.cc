#include <stm32f4xx_hal.h>

extern "C" {
void NMI_Handler(void) {
  while (1) {
  }
}

void HardFault_Handler(void) {
  while (1) {
  }
}

void MemManage_Handler(void) {
  while (1) {
  }
}

void BusFault_Handler(void) {
  while (1) {
  }
}

void UsageFault_Handler(void) {
  while (1) {
  }
}

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}
void Error_Handler(void) {
  for (;;)
    ;
}

void SysTick_Handler(void) { HAL_IncTick(); }
}

// TEST
//
//

#define SD_DETECT_Pin GPIO_PIN_13
#define SD_DETECT_GPIO_Port GPIOC
#define ENCODER_SWITCH_Pin GPIO_PIN_14
#define ENCODER_SWITCH_GPIO_Port GPIOC
#define POT_4_Pin GPIO_PIN_0
#define POT_4_GPIO_Port GPIOC
#define POT_3_Pin GPIO_PIN_1
#define POT_3_GPIO_Port GPIOC
#define POT_2_Pin GPIO_PIN_2
#define POT_2_GPIO_Port GPIOC
#define POT_1_Pin GPIO_PIN_3
#define POT_1_GPIO_Port GPIOC
#define SPI_SS_EXT_Pin GPIO_PIN_4
#define SPI_SS_EXT_GPIO_Port GPIOA
#define SPI_SS_CODEC_Pin GPIO_PIN_4
#define SPI_SS_CODEC_GPIO_Port GPIOC
#define RESET_CODEC_Pin GPIO_PIN_1
#define RESET_CODEC_GPIO_Port GPIOB
#define RESET_DAC_Pin GPIO_PIN_2
#define RESET_DAC_GPIO_Port GPIOB
#define SPI_SS_DAC_Pin GPIO_PIN_11
#define SPI_SS_DAC_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_7
#define LCD_RS_GPIO_Port GPIOC
#define LCD_E_Pin GPIO_PIN_9
#define LCD_E_GPIO_Port GPIOC
#define MIDI_OUT_Pin GPIO_PIN_9
#define MIDI_OUT_GPIO_Port GPIOA
#define MIDI_IN_Pin GPIO_PIN_10
#define MIDI_IN_GPIO_Port GPIOA
#define ENCODER_A_Pin GPIO_PIN_10
#define ENCODER_A_GPIO_Port GPIOC
#define ENCODER_B_Pin GPIO_PIN_11
#define ENCODER_B_GPIO_Port GPIOC
#define LCD_D1_Pin GPIO_PIN_4
#define LCD_D1_GPIO_Port GPIOB
#define LCD_D2_Pin GPIO_PIN_5
#define LCD_D2_GPIO_Port GPIOB
#define LCD_D3_Pin GPIO_PIN_6
#define LCD_D3_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_7
#define LCD_D4_GPIO_Port GPIOB
#define SPI_SS_SWITCHES_Pin GPIO_PIN_8
#define SPI_SS_SWITCHES_GPIO_Port GPIOB
#define SPI_SS_LEDS_Pin GPIO_PIN_9
#define SPI_SS_LEDS_GPIO_Port GPIOB

ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;
TIM_HandleTypeDef htim2;
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_i2s2_ext_rx;
SPI_HandleTypeDef hspi1;

const size_t kBufferLength = 32;

int32_t buffer[kBufferLength * 2 * 2];      // 2 channels, 2 buffers
uint32_t in_buffer[kBufferLength * 2 * 2];  // 2 channels, 2 buffers

extern "C" {
void DMA1_Stream3_IRQHandler(void) {
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2s2_ext_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream4 global interrupt.
 */
void DMA1_Stream4_IRQHandler(void) {
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
 * @brief This function handles TIM2 global interrupt.
 */
void TIM2_IRQHandler(void) {
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
 * @brief This function handles SPI2 global interrupt.
 */
void SPI2_IRQHandler(void) {
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_I2S_IRQHandler(&hi2s2);
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/**
 * @brief This function handles DMA2 stream0 global interrupt.
 */
void DMA2_Stream0_IRQHandler(void) {
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}
}

/* I2S2 init function */
void MX_I2S2_Init(void) {
  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_LSB;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_32B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */
}

uint16_t pots[4];
int32_t swap16(int32_t x) {
  return x;
  return __ROR(x, 16);
}

#include <math.h>

#include <climits>

float ramp = 0;
void FillBuffer(int32_t* buffer, size_t length) {
  for (size_t i = 0; i < length; i += 2) {
    float val = sin(3.141 * ramp);

    buffer[i + 0] = swap16(
        (int32_t)(val * (INT_MAX >> 1)));  // 24 bit, last 8 bit are discarded
    buffer[i + 1] = swap16(
        (int32_t)(-ramp * (INT_MAX >> 1)));  // 24 bit, last 8 bit are discarded
    ramp += pots[3] * 0.000001;
    if (ramp > 1) {
      ramp = -1;
    }
  }
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef* handle) {
  FillBuffer(&buffer[kBufferLength * 2], kBufferLength * 2);
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef* handle) {
  FillBuffer(&buffer[0], kBufferLength * 2);
}

void HAL_I2S_MspInit(I2S_HandleTypeDef* i2sHandle) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (i2sHandle->Instance == SPI2) {
    /* USER CODE BEGIN SPI2_MspInit 0 */

    /* USER CODE END SPI2_MspInit 0 */
    /* I2S2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2S2 GPIO Configuration
    PB12     ------> I2S2_WS
    PB13     ------> I2S2_CK
    PB14     ------> I2S2_ext_SD
    PB15     ------> I2S2_SD
    PC6     ------> I2S2_MCK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2S2ext;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* I2S2 DMA Init */
    /* SPI2_TX Init */
    hdma_spi2_tx.Instance = DMA1_Stream4;
    hdma_spi2_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi2_tx.Init.Mode = DMA_CIRCULAR;
    hdma_spi2_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_spi2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK) {
      Error_Handler();
    }

    __HAL_LINKDMA(i2sHandle, hdmatx, hdma_spi2_tx);

    /* I2S2_EXT_RX Init */
    hdma_i2s2_ext_rx.Instance = DMA1_Stream3;
    hdma_i2s2_ext_rx.Init.Channel = DMA_CHANNEL_3;
    hdma_i2s2_ext_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2s2_ext_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2s2_ext_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2s2_ext_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_i2s2_ext_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_i2s2_ext_rx.Init.Mode = DMA_CIRCULAR;
    hdma_i2s2_ext_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_i2s2_ext_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2s2_ext_rx) != HAL_OK) {
      Error_Handler();
    }

    __HAL_LINKDMA(i2sHandle, hdmarx, hdma_i2s2_ext_rx);

    /* I2S2 interrupt Init */
    HAL_NVIC_SetPriority(SPI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);

    /* USER CODE BEGIN SPI2_MspInit 1 */

    /* USER CODE END SPI2_MspInit 1 */
  }
}

void MX_DMA_Init(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}
/* ADC3 init function */
void MX_ADC3_Init(void) {
  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data
   * Alignment and number of conversion)
   */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc3.Init.NbrOfConversion = 4;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
    Error_Handler();
  } /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (adcHandle->Instance == ADC3) {
    /* USER CODE BEGIN ADC3_MspInit 0 */

    /* USER CODE END ADC3_MspInit 0 */
    /* ADC3 clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC3 GPIO Configuration
    PC0     ------> ADC3_IN10
    PC1     ------> ADC3_IN11
    PC2     ------> ADC3_IN12
    PC3     ------> ADC3_IN13
    */
    GPIO_InitStruct.Pin = POT_4_Pin | POT_3_Pin | POT_2_Pin | POT_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    hdma_adc3.Instance = DMA2_Stream0;
    hdma_adc3.Init.Channel = DMA_CHANNEL_2;
    hdma_adc3.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc3.Init.Mode = DMA_CIRCULAR;
    hdma_adc3.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc3.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc3) != HAL_OK) {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc3);
    /* USER CODE BEGIN ADC3_MspInit 1 */

    /* USER CODE END ADC3_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle) {
  if (adcHandle->Instance == ADC3) {
    /* USER CODE BEGIN ADC3_MspDeInit 0 */

    /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();

    /**ADC3 GPIO Configuration
    PC0     ------> ADC3_IN10
    PC1     ------> ADC3_IN11
    PC2     ------> ADC3_IN12
    PC3     ------> ADC3_IN13
    */
    HAL_GPIO_DeInit(GPIOC, POT_4_Pin | POT_3_Pin | POT_2_Pin | POT_1_Pin);

    HAL_DMA_DeInit(adcHandle->DMA_Handle);
    /* USER CODE BEGIN ADC3_MspDeInit 1 */

    /* USER CODE END ADC3_MspDeInit 1 */
  }
}

/* SPI1 init function */
void MX_SPI1_Init(void) {
  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (spiHandle->Instance == SPI1) {
    /* USER CODE BEGIN SPI1_MspInit 0 */

    /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN SPI1_MspInit 1 */

    /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle) {
  if (spiHandle->Instance == SPI1) {
    /* USER CODE BEGIN SPI1_MspDeInit 0 */

    /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    /* USER CODE BEGIN SPI1_MspDeInit 1 */

    /* USER CODE END SPI1_MspDeInit 1 */
  }
}

void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_E_Pin | SPI_SS_CODEC_Pin | LCD_RS_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_SS_EXT_GPIO_Port, SPI_SS_EXT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB,
                    RESET_CODEC_Pin | GPIO_PIN_2 | SPI_SS_DAC_Pin | LCD_D1_Pin |
                        LCD_D2_Pin | LCD_D3_Pin | LCD_D4_Pin |
                        SPI_SS_SWITCHES_Pin | SPI_SS_LEDS_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PCPin PCPin PCPin */
  GPIO_InitStruct.Pin =
      GPIO_PIN_13 | ENCODER_SWITCH_Pin | ENCODER_A_Pin | ENCODER_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = LCD_E_Pin | SPI_SS_CODEC_Pin | LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = SPI_SS_EXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_SS_EXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PB2 PBPin PBPin
                           PBPin PBPin PBPin PBPin
                           PBPin */
  GPIO_InitStruct.Pin = RESET_CODEC_Pin | GPIO_PIN_2 | SPI_SS_DAC_Pin |
                        LCD_D1_Pin | LCD_D2_Pin | LCD_D3_Pin | LCD_D4_Pin |
                        SPI_SS_SWITCHES_Pin | SPI_SS_LEDS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection =
      RCC_PERIPHCLK_I2S | RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 129;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLLI2SCLK, RCC_MCODIV_4);
}

int main() {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_I2S2_Init();
  MX_ADC3_Init();

  // CODEC
  HAL_I2SEx_TransmitReceive_DMA(&hi2s2, (uint16_t*)buffer, (uint16_t*)in_buffer,
                                kBufferLength * 2 * 2);
  HAL_GPIO_WritePin(RESET_CODEC_GPIO_Port, RESET_CODEC_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(RESET_CODEC_GPIO_Port, RESET_CODEC_Pin, GPIO_PIN_SET);

  // HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)buffer, kBufferLength * 2);

  uint8_t reg[2];

  HAL_GPIO_WritePin(SPI_SS_CODEC_GPIO_Port, SPI_SS_CODEC_Pin, GPIO_PIN_RESET);
  reg[0] = 65;
  reg[1] = 0xFF;  // attl
  HAL_SPI_Transmit(&hspi1, reg, 2, 1000);
  HAL_GPIO_WritePin(SPI_SS_CODEC_GPIO_Port, SPI_SS_CODEC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI_SS_CODEC_GPIO_Port, SPI_SS_CODEC_Pin, GPIO_PIN_RESET);
  reg[0] = 66;
  reg[1] = 0xFF;  // attr
  HAL_SPI_Transmit(&hspi1, reg, 2, 1000);
  HAL_GPIO_WritePin(SPI_SS_CODEC_GPIO_Port, SPI_SS_CODEC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI_SS_CODEC_GPIO_Port, SPI_SS_CODEC_Pin, GPIO_PIN_RESET);
  reg[0] = 67;
  reg[1] = 0b00000011;
  HAL_SPI_Transmit(&hspi1, reg, 2, 1000);
  HAL_GPIO_WritePin(SPI_SS_CODEC_GPIO_Port, SPI_SS_CODEC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI_SS_CODEC_GPIO_Port, SPI_SS_CODEC_Pin, GPIO_PIN_RESET);
  reg[0] = 68;
  reg[1] = 0b00000100;
  HAL_SPI_Transmit(&hspi1, reg, 2, 1000);
  HAL_GPIO_WritePin(SPI_SS_CODEC_GPIO_Port, SPI_SS_CODEC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI_SS_CODEC_GPIO_Port, SPI_SS_CODEC_Pin, GPIO_PIN_RESET);
  reg[0] = 64;
  reg[1] = 0b11000001;
  HAL_SPI_Transmit(&hspi1, reg, 2, 1000);
  HAL_GPIO_WritePin(SPI_SS_CODEC_GPIO_Port, SPI_SS_CODEC_Pin, GPIO_PIN_SET);

  // DAC

  HAL_GPIO_WritePin(RESET_DAC_GPIO_Port, RESET_DAC_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RESET_DAC_GPIO_Port, RESET_DAC_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(SPI_SS_DAC_GPIO_Port, SPI_SS_DAC_Pin, GPIO_PIN_RESET);
  uint8_t dac[] = {0b0100000, 0, 0};
  HAL_SPI_Transmit(&hspi1, dac, 3, 1000);
  HAL_GPIO_WritePin(SPI_SS_DAC_GPIO_Port, SPI_SS_DAC_Pin, GPIO_PIN_SET);

  for (uint8_t i = 0; i < 8; i++) {
    HAL_GPIO_WritePin(SPI_SS_DAC_GPIO_Port, SPI_SS_DAC_Pin, GPIO_PIN_RESET);

    dac[0] = 0b00010000 + i;
    dac[1] = i == 2 || i == 4 ? 0 : 255;
    dac[2] = i == 2 || i == 4 ? 0 : 255;
    HAL_SPI_Transmit(&hspi1, dac, 3, 1000);
    HAL_GPIO_WritePin(SPI_SS_DAC_GPIO_Port, SPI_SS_DAC_Pin, GPIO_PIN_SET);
  }

  // ADC
  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)pots, 4);

  for (;;)
    for (uint8_t i = 0; i < 255; i++) {
      for (uint8_t j = 0; j < 4; j++) {
        uint16_t raw = pots[j];
        HAL_GPIO_WritePin(SPI_SS_DAC_GPIO_Port, SPI_SS_DAC_Pin, GPIO_PIN_RESET);

        dac[0] = 0b00010000 + j;
        dac[1] = raw >> 8;
        dac[2] = raw & 0xFF;
        HAL_SPI_Transmit(&hspi1, dac, 3, 1000);
        HAL_GPIO_WritePin(SPI_SS_DAC_GPIO_Port, SPI_SS_DAC_Pin, GPIO_PIN_SET);
      }

      uint8_t data = 0;
      HAL_GPIO_WritePin(SPI_SS_SWITCHES_GPIO_Port, SPI_SS_SWITCHES_Pin,
                        GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SPI_SS_SWITCHES_GPIO_Port, SPI_SS_SWITCHES_Pin,
                        GPIO_PIN_SET);
      HAL_SPI_Receive(&hspi1, (uint8_t*)&data, 1, 100);
      data = pots[0] >> 8;
      HAL_GPIO_WritePin(SPI_SS_LEDS_GPIO_Port, SPI_SS_LEDS_Pin, GPIO_PIN_RESET);
      HAL_SPI_Transmit(&hspi1, (uint8_t*)&data, 1, 100);
      HAL_GPIO_WritePin(SPI_SS_LEDS_GPIO_Port, SPI_SS_LEDS_Pin, GPIO_PIN_SET);

      uint8_t routing = 0;

      HAL_GPIO_WritePin(SPI_SS_EXT_GPIO_Port, SPI_SS_EXT_Pin, GPIO_PIN_RESET);
      HAL_SPI_Transmit(&hspi1, (uint8_t*)&routing, 1, 100);
      HAL_GPIO_WritePin(SPI_SS_EXT_GPIO_Port, SPI_SS_EXT_Pin, GPIO_PIN_SET);

      for (uint32_t j = 0; j < 20000; j++) {
      }
    };
  for (;;)
    ;
}
