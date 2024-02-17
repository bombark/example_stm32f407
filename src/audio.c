// =============================================================================
//  Header
// =============================================================================

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2s.h"
#include "cs43l22.h"
#include "led_blue.h"
#include "led_red.h"

const uint32_t I2SFreq[8] = {8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000};
const uint32_t I2SPLLN[8] = {256, 429, 213, 429, 426, 271, 258, 344};
const uint32_t I2SPLLR[8] = {5, 4, 4, 4, 4, 6, 3, 1};

// Handle
I2S_HandleTypeDef      hAudioOutI2s;



/* Audio status definition */     
#define AUDIO_OK                        0
#define AUDIO_ERROR                     1
#define AUDIO_TIMEOUT                   2


#define AUDIODATA_SIZE                  2
#define I2S_STANDARD                  I2S_STANDARD_PHILIPS


/* I2S peripheral configuration defines */
#define I2S3                            SPI3
#define I2S3_CLK_ENABLE()               __SPI3_CLK_ENABLE()
#define I2S3_SCK_SD_WS_AF               GPIO_AF6_SPI3
#define I2S3_SCK_SD_CLK_ENABLE()        __GPIOC_CLK_ENABLE()
#define I2S3_MCK_CLK_ENABLE()           __GPIOC_CLK_ENABLE()
#define I2S3_WS_CLK_ENABLE()            __GPIOA_CLK_ENABLE()
#define I2S3_WS_PIN                     GPIO_PIN_4
#define I2S3_SCK_PIN                    GPIO_PIN_10
#define I2S3_SD_PIN                     GPIO_PIN_12
#define I2S3_MCK_PIN                    GPIO_PIN_7
#define I2S3_SCK_SD_GPIO_PORT           GPIOC
#define I2S3_WS_GPIO_PORT               GPIOA
#define I2S3_MCK_GPIO_PORT              GPIOC

/* I2S DMA Stream definitions */
#define I2S3_DMAx_CLK_ENABLE()          __DMA1_CLK_ENABLE()
#define I2S3_DMAx_STREAM                DMA1_Stream7
#define I2S3_DMAx_CHANNEL               DMA_CHANNEL_0
#define I2S3_DMAx_IRQ                   DMA1_Stream7_IRQn
#define I2S3_DMAx_PERIPH_DATA_SIZE      DMA_PDATAALIGN_HALFWORD
#define I2S3_DMAx_MEM_DATA_SIZE         DMA_MDATAALIGN_HALFWORD
#define DMA_MAX_SZE                     0xFFFF

#define I2S3_IRQHandler                 DMA1_Stream7_IRQHandler

/* Select the interrupt preemption priority and subpriority for the DMA interrupt */
#define AUDIO_OUT_IRQ_PREPRIO           5   /* Select the preemption priority level(0 is the highest) */

#define DMA_MAX(_X_)                (((_X_) <= DMA_MAX_SZE)? (_X_):DMA_MAX_SZE)

// =============================================================================
//  Functions
// =============================================================================

/**
  * @brief  AUDIO OUT I2S MSP Init.
  * @param  None
  * @retval None
  */
static void I2S3_MspInit(void)
{
  static DMA_HandleTypeDef hdma_i2sTx;
  GPIO_InitTypeDef  GPIO_InitStruct;
  I2S_HandleTypeDef *hi2s = &hAudioOutI2s;
  
  /* Enable I2S3 clock */
  I2S3_CLK_ENABLE();
  
  /*** Configure the GPIOs ***/  
  /* Enable I2S GPIO clocks */
  I2S3_SCK_SD_CLK_ENABLE();
  I2S3_WS_CLK_ENABLE();
  
  /* I2S3 pins configuration: WS, SCK and SD pins ----------------------------*/
  GPIO_InitStruct.Pin         = I2S3_SCK_PIN | I2S3_SD_PIN; 
  GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull        = GPIO_NOPULL;
  GPIO_InitStruct.Speed       = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate   = I2S3_SCK_SD_WS_AF;
  HAL_GPIO_Init(I2S3_SCK_SD_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin         = I2S3_WS_PIN ;
  HAL_GPIO_Init(I2S3_WS_GPIO_PORT, &GPIO_InitStruct); 
  
  /* I2S3 pins configuration: MCK pin */
  I2S3_MCK_CLK_ENABLE();
  GPIO_InitStruct.Pin         = I2S3_MCK_PIN; 
  HAL_GPIO_Init(I2S3_MCK_GPIO_PORT, &GPIO_InitStruct);   
  
  /* Enable the I2S DMA clock */
  I2S3_DMAx_CLK_ENABLE(); 
  
  if(hi2s->Instance == I2S3)
  {
    /* Configure the hdma_i2sTx handle parameters */   
    hdma_i2sTx.Init.Channel             = I2S3_DMAx_CHANNEL;  
    hdma_i2sTx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_i2sTx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2sTx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2sTx.Init.PeriphDataAlignment = I2S3_DMAx_PERIPH_DATA_SIZE;
    hdma_i2sTx.Init.MemDataAlignment    = I2S3_DMAx_MEM_DATA_SIZE;
    hdma_i2sTx.Init.Mode                = DMA_NORMAL;
    hdma_i2sTx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_i2sTx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;         
    hdma_i2sTx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_i2sTx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_i2sTx.Init.PeriphBurst         = DMA_PBURST_SINGLE; 
    
    hdma_i2sTx.Instance                 = I2S3_DMAx_STREAM;
    
    /* Associate the DMA handle */
    __HAL_LINKDMA(hi2s, hdmatx, hdma_i2sTx);
    
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_i2sTx);
    
    /* Configure the DMA Stream */
    HAL_DMA_Init(&hdma_i2sTx);
  }
  
  /* I2S DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(I2S3_DMAx_IRQ, AUDIO_OUT_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ(I2S3_DMAx_IRQ); 
}

static void I2S3_Init(uint32_t AudioFreq) {
    /* Initialize the hAudioOutI2s Instance parameter */
    hAudioOutI2s.Instance = I2S3;

    /* Disable I2S block */
    __HAL_I2S_DISABLE(&hAudioOutI2s);

    /* I2S3 peripheral configuration */
    hAudioOutI2s.Init.AudioFreq   = AudioFreq;
    hAudioOutI2s.Init.ClockSource = I2S_CLOCK_PLL;
    hAudioOutI2s.Init.CPOL        = I2S_CPOL_LOW;
    hAudioOutI2s.Init.DataFormat  = I2S_DATAFORMAT_16B;
    hAudioOutI2s.Init.MCLKOutput  = I2S_MCLKOUTPUT_ENABLE;
    hAudioOutI2s.Init.Mode        = I2S_MODE_MASTER_TX;
    hAudioOutI2s.Init.Standard    = I2S_STANDARD;

    /* Initialize the I2S peripheral with the structure above */
    if(HAL_I2S_GetState(&hAudioOutI2s) == HAL_I2S_STATE_RESET) { 
        I2S3_MspInit();
    }

    HAL_I2S_Init(&hAudioOutI2s);
}


/**
  * @brief  Configures the audio peripherals.
  * @param  OutputDevice: OUTPUT_DEVICE_SPEAKER, OUTPUT_DEVICE_HEADPHONE,
  *                       OUTPUT_DEVICE_BOTH or OUTPUT_DEVICE_AUTO .
  * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t audio_out_init(uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq)
{    
  uint8_t ret = AUDIO_ERROR;
  uint32_t deviceid = 0x00;
  RCC_PeriphCLKInitTypeDef rccclkinit;
  uint8_t index = 0, freqindex = 0xFF;
  
  for(index = 0; index < 8; index++) {
    if(I2SFreq[index] == AudioFreq) {
      freqindex = index;
    }
  }
  /* Enable PLLI2S clock */
  HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit);
  /* PLLI2S_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
  if ((freqindex & 0x7) == 0)
  {
    /* I2S clock config 
    PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) � (PLLI2SN/PLLM)
    I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SR */
    rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    rccclkinit.PLLI2S.PLLI2SN = I2SPLLN[freqindex];
    rccclkinit.PLLI2S.PLLI2SR = I2SPLLR[freqindex];
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
  }
  else 
  {
    /* I2S clock config 
    PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) � (PLLI2SN/PLLM)
    I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SR */
    rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    rccclkinit.PLLI2S.PLLI2SN = 258;
    rccclkinit.PLLI2S.PLLI2SR = 3;
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
  }

  deviceid = cs43l22_ReadID();

  if((deviceid & CS43L22_ID_MASK) == CS43L22_ID) {  
    ret = AUDIO_OK;
  } else {
    ret = AUDIO_ERROR;
  }
  
  if(ret == AUDIO_OK)
  {
    cs43l22_init(OutputDevice, Volume, AudioFreq);
    /* I2S data transfer preparation:
       Prepare the Media to be used for the audio transfer from memory to I2S peripheral */
    /* Configure the I2S peripheral */
    I2S3_Init(AudioFreq);
  }
  
    __HAL_I2S_ENABLE(&hAudioOutI2s);

  return ret;
}

/**
  * @brief  Update the audio frequency.
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @retval None
  * @note   This API should be called after the BSP_AUDIO_OUT_Init() to adjust the
  *         audio frequency. 
  */
void audio_out_set_frequency(uint32_t AudioFreq)
{ 
  RCC_PeriphCLKInitTypeDef rccclkinit;
  
  /* Enable PLLI2S clock */
  HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit);
  /* PLLI2S_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
  if ((AudioFreq & 0x7) == 0)
  {
    /* Audio frequency multiple of 8 (8/16/32/48/96/192) */
    /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN = 192 Mhz */
    /* I2SCLK = PLLI2S_VCO Output/PLLI2SR = 192/6 = 32 Mhz */
    rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    rccclkinit.PLLI2S.PLLI2SN = 192;
    rccclkinit.PLLI2S.PLLI2SR = 6;
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
  }
  else
  {
    /* Other Frequency (11.025/22.500/44.100) */
    /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN = 290 Mhz */
    /* I2SCLK = PLLI2S_VCO Output/PLLI2SR = 290/2 = 145 Mhz */
    rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    rccclkinit.PLLI2S.PLLI2SN = 290;
    rccclkinit.PLLI2S.PLLI2SR = 2;
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
  }
  
  /* Update the I2S audio frequency configuration */
  I2S3_Init(AudioFreq);
}

/**
  * @brief  Starts playing audio stream from a data buffer for a determined size. 
  * @param  pBuffer: Pointer to the buffer 
  * @param  Size: Number of audio data BYTES.
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t audio_out_play(uint16_t* pBuffer, uint32_t Size) {
    /* Call the audio Codec Play function */
    if( cs43l22_Play(pBuffer, Size) != 0) {
        return AUDIO_ERROR;
    }

    /* Update the Media layer and enable it for play */  
    // HAL_I2S_Transmit_DMA(&hAudioOutI2s, pBuffer, DMA_MAX(Size/AUDIODATA_SIZE)); 

    HAL_I2S_Transmit(&hAudioOutI2s, pBuffer, Size, 2000);
    
    /* Return AUDIO_OK when all operations are correctly done */
    return AUDIO_OK;
}



// =============================================================================
//  Interrupcoes
// =============================================================================

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  hi2s: I2S handle
  * @retval None
  */
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if(hi2s->Instance == I2S3)
  {
    /* Call the user function which will manage directly transfer complete */  
    // BSP_AUDIO_OUT_TransferComplete_CallBack();       
    led_blue_toggle();
  }
}

/**
  * @brief  Tx Half Transfer completed callbacks.
  * @param  hi2s: I2S handle
  * @retval None
  */
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if(hi2s->Instance == I2S3)
  {
    /* Manage the remaining file size and new address offset: This function should
       be coded by user (its prototype is already declared in stm32f4_discovery_audio.h) */  
    // BSP_AUDIO_OUT_HalfTransfer_CallBack();
    led_red_toggle();
  }
}