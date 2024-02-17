/**
  ******************************************************************************
  * @file    cs43l22.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-February-2014
  * @brief   This file provides the CS43L22 Audio Codec driver.   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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

// =============================================================================
//  Header
// =============================================================================

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "cs43l22.h"

#if !defined (VERIFY_WRITTENDATA)  
/* #define VERIFY_WRITTENDATA */
#endif /* VERIFY_WRITTENDATA */

#define I2C_SPEED                            100000

#define DISCOVERY_I2Cx                            I2C1
#define DISCOVERY_I2Cx_CLK_ENABLE()               __I2C1_CLK_ENABLE()
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define DISCOVERY_I2Cx_SCL_SDA_AF                 GPIO_AF4_I2C1
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT          GPIOB
#define DISCOVERY_I2Cx_SCL_PIN                    GPIO_PIN_6
#define DISCOVERY_I2Cx_SDA_PIN                    GPIO_PIN_9
#define DISCOVERY_I2Cx_FORCE_RESET()              __I2C1_FORCE_RESET()
#define DISCOVERY_I2Cx_RELEASE_RESET()            __I2C1_RELEASE_RESET()
#define DISCOVERY_I2Cx_EV_IRQn                    I2C1_EV_IRQn
#define DISCOVERY_I2Cx_ER_IRQn                    I2C1_ER_IRQn

#define AUDIO_RESET_GPIO_CLK_ENABLE()   __GPIOD_CLK_ENABLE()
#define AUDIO_RESET_PIN                 GPIO_PIN_4
#define AUDIO_RESET_GPIO                GPIOD

#define CODEC_STANDARD                0x04

// Configuration
static uint8_t Is_cs43l22_Stop = 1;
volatile uint8_t OutputDev = 0;
const uint32_t I2cxTimeout = 100;
const uint16_t DeviceAddr = 0x94;

// private variable
I2C_HandleTypeDef I2cHandle;

// =============================================================================
//  I2Cx
// =============================================================================

void I2Cx_MspInit(void) {
    GPIO_InitTypeDef  GPIO_InitStruct;

    /* Enable I2C GPIO clocks */
    DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();

    /* DISCOVERY_I2Cx SCL and SDA pins configuration ---------------------------*/
    GPIO_InitStruct.Pin = DISCOVERY_I2Cx_SCL_PIN | DISCOVERY_I2Cx_SDA_PIN; 
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Alternate  = DISCOVERY_I2Cx_SCL_SDA_AF;
    HAL_GPIO_Init(DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);     

    /* Enable the DISCOVERY_I2Cx peripheral clock */
    DISCOVERY_I2Cx_CLK_ENABLE();

    /* Force the I2C peripheral clock reset */
    DISCOVERY_I2Cx_FORCE_RESET();

    /* Release the I2C peripheral clock reset */
    DISCOVERY_I2Cx_RELEASE_RESET();

    /* Enable and set I2Cx Interrupt to the highest priority */
    HAL_NVIC_SetPriority(DISCOVERY_I2Cx_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DISCOVERY_I2Cx_EV_IRQn);

    /* Enable and set I2Cx Interrupt to the highest priority */
    HAL_NVIC_SetPriority(DISCOVERY_I2Cx_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DISCOVERY_I2Cx_ER_IRQn); 
}

void I2Cx_Init(void) {
    if( HAL_I2C_GetState(&I2cHandle) == HAL_I2C_STATE_RESET) {
        // DISCOVERY_I2Cx peripheral configuration
        I2cHandle.Init.ClockSpeed = I2C_SPEED;
        I2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
        I2cHandle.Init.OwnAddress1 = 0x33;
        I2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
        I2cHandle.Instance = DISCOVERY_I2Cx;
            
        // Init the I2C
        I2Cx_MspInit();
        HAL_I2C_Init(&I2cHandle);
    }
}

// =============================================================================
//  AUDIO_IO
// =============================================================================

/**
  * @brief  Initializes Audio low level.
  * @param  None
  * @retval None
  */
void AUDIO_IO_Init(void) {
    GPIO_InitTypeDef  GPIO_InitStruct;

    /* Enable Reset GPIO Clock */
    AUDIO_RESET_GPIO_CLK_ENABLE();

    /* Audio reset pin configuration */
    GPIO_InitStruct.Pin = AUDIO_RESET_PIN; 
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(AUDIO_RESET_GPIO, &GPIO_InitStruct);    

    I2Cx_Init();

    /* Power Down the codec */
    HAL_GPIO_WritePin(AUDIO_RESET_GPIO, AUDIO_RESET_PIN, GPIO_PIN_RESET);

    /* Wait for a delay to insure registers erasing */
    HAL_Delay(5); 

    /* Power on the codec */
    HAL_GPIO_WritePin(AUDIO_RESET_GPIO, AUDIO_RESET_PIN, GPIO_PIN_SET);

    /* Wait for a delay to insure registers erasing */
    HAL_Delay(5); 
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  Value: Data to be written
  * @retval None
  */
void AUDIO_IO_Write (uint8_t Addr, uint8_t Reg, uint8_t Value) {
    HAL_StatusTypeDef status = HAL_OK;

    status = HAL_I2C_Mem_Write(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2cxTimeout); 
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @retval Data to be read
  */
uint8_t AUDIO_IO_Read(uint8_t Addr, uint8_t Reg) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t value = 0;

    status = HAL_I2C_Mem_Read(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &value, 1,I2cxTimeout);

    return value;
}

// =============================================================================
//  CODEC_IO
// =============================================================================

/**
  * @brief  Writes/Read a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  Value: Data to be written
  * @retval None
  */
static 
uint8_t CODEC_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value) {
    uint32_t result = 0;
    AUDIO_IO_Write(Addr, Reg, Value); 

// Verify that the data has been correctly written
#ifdef VERIFY_WRITTENDATA
    result = (AUDIO_IO_Read(Addr, Reg) == Value)? 0:1;
#endif

    return result;
}

// =============================================================================
//  cs43l22
// =============================================================================

/**
  * @brief Initializes the audio codec and the control interface.
  * @param DeviceAddr: Device address on communication Bus.   
  * @param OutputDevice: can be OUTPUT_DEVICE_SPEAKER, OUTPUT_DEVICE_HEADPHONE,
  *                       OUTPUT_DEVICE_BOTH or OUTPUT_DEVICE_AUTO .
  * @param Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t cs43l22_init(uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq) {
    uint32_t counter = 0;

    /* Initialize the Control interface of the Audio Codec */
    AUDIO_IO_Init();     

    /* Keep Codec powered OFF */
    counter += CODEC_IO_Write(DeviceAddr, 0x02, 0x01);  
  
    /*Save Output device for mute ON/OFF procedure*/
    switch (OutputDevice) {
        case OUTPUT_DEVICE_SPEAKER:
            OutputDev = 0xFA;
            break;

        case OUTPUT_DEVICE_HEADPHONE:
            OutputDev = 0xAF;
            break;

        case OUTPUT_DEVICE_BOTH:
            OutputDev = 0xAA;
            break;

        case OUTPUT_DEVICE_AUTO:
            OutputDev = 0x05;
            break;    

        default:
            OutputDev = 0x05;
            break;    
    }
  
    counter += CODEC_IO_Write(DeviceAddr, 0x04, OutputDev);

    /* Clock configuration: Auto detection */  
    counter += CODEC_IO_Write(DeviceAddr, 0x05, 0x81);

    /* Set the Slave Mode and the audio Standard */  
    counter += CODEC_IO_Write(DeviceAddr, 0x06, CODEC_STANDARD);

    /* Set the Master volume */
    counter += cs43l22_SetVolume(Volume);
  
    /* If the Speaker is enabled, set the Mono mode and volume attenuation level */
    if(OutputDevice != OUTPUT_DEVICE_HEADPHONE) {
        /* Set the Speaker Mono mode */  
        counter += CODEC_IO_Write(DeviceAddr, 0x0F , 0x06);

        /* Set the Speaker attenuation level */  
        counter += CODEC_IO_Write(DeviceAddr, 0x24, 0x00);
        counter += CODEC_IO_Write(DeviceAddr, 0x25, 0x00);
    }
  
    /* Additional configuration for the CODEC. These configurations are done to reduce
    the time needed for the Codec to power off. If these configurations are removed, 
    then a long delay should be added between powering off the Codec and switching 
    off the I2S peripheral MCLK clock (which is the operating clock for Codec).
    If this delay is not inserted, then the codec will not shut down properly and
    it results in high noise after shut down. */

    /* Disable the analog soft ramp */
    counter += CODEC_IO_Write(DeviceAddr, 0x0A, 0x00);
    /* Disable the digital soft ramp */
    counter += CODEC_IO_Write(DeviceAddr, 0x0E, 0x04);
    /* Disable the limiter attack level */
    counter += CODEC_IO_Write(DeviceAddr, 0x27, 0x00);
    /* Adjust Bass and Treble levels */
    counter += CODEC_IO_Write(DeviceAddr, 0x1F, 0x0F);
    /* Adjust PCM volume level */
    counter += CODEC_IO_Write(DeviceAddr, 0x1A, 0x0A);
    counter += CODEC_IO_Write(DeviceAddr, 0x1B, 0x0A);

    /* Return communication control value */
    return counter;  
}

/**
  * @brief  Get the CS43L22 ID.
  * @param DeviceAddr: Device address on communication Bus.   
  * @retval The CS43L22 ID 
  */
uint32_t cs43l22_ReadID() {
    /* Initialize the Control interface of the Audio Codec */
    AUDIO_IO_Init(); 

    return ((uint32_t)AUDIO_IO_Read(DeviceAddr, CS43L22_CHIPID_ADDR));
}

/**
  * @brief Start the audio Codec play feature.
  * @note For this codec no Play options are required.
  * @param DeviceAddr: Device address on communication Bus.   
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t cs43l22_Play(uint16_t* pBuffer, uint16_t Size) {
    uint32_t counter = 0;

    if(Is_cs43l22_Stop == 1) {
        /* Enable Output device */  
        counter += cs43l22_SetMute(AUDIO_MUTE_OFF);

        /* Power on the Codec */
        counter += CODEC_IO_Write(DeviceAddr, 0x02, 0x9E);  
        Is_cs43l22_Stop = 0;
    }

    /* Return communication control value */
    return counter;  
}

/**
  * @brief Pauses playing on the audio codec.
  * @param DeviceAddr: Device address on communication Bus. 
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t cs43l22_Pause() {  
    uint32_t counter = 0;

    /* Pause the audio file playing */
    /* Mute the output first */
    counter += cs43l22_SetMute(AUDIO_MUTE_ON);

    /* Put the Codec in Power save mode */    
    counter += CODEC_IO_Write(DeviceAddr,0x02, 0x01);

    return counter;
}

/**
  * @brief Resumes playing on the audio codec.
  * @param DeviceAddr: Device address on communication Bus. 
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t cs43l22_Resume() {
    uint32_t counter = 0;
    volatile uint32_t index = 0x00;
    /* Resumes the audio file playing */  
    /* Unmute the output first */
    counter += cs43l22_SetMute(AUDIO_MUTE_OFF);

    for(index = 0x00; index < 0xFF; index++)
        ;

    counter += CODEC_IO_Write(DeviceAddr,0x04, OutputDev);

    /* Exit the Power save mode */
    counter += CODEC_IO_Write(DeviceAddr,0x02, 0x9E); 

    return counter;
}

/**
  * @brief Stops audio Codec playing. It powers down the codec.
  * @param DeviceAddr: Device address on communication Bus. 
  * @param CodecPdwnMode: selects the  power down mode.
  *          - CODEC_PDWN_HW: Physically power down the codec. When resuming from this
  *                           mode, the codec is set to default configuration 
  *                           (user should re-Initialize the codec in order to 
  *                            play again the audio stream).
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t cs43l22_Stop(uint32_t CodecPdwnMode) {
    uint32_t counter = 0;

    /* Mute the output first */
    counter += cs43l22_SetMute(AUDIO_MUTE_ON);

    /* Power down the DAC and the speaker (PMDAC and PMSPK bits)*/
    counter += CODEC_IO_Write(DeviceAddr, 0x02, 0x9F);

    Is_cs43l22_Stop = 1;
    return counter;    
}

/**
  * @brief Sets higher or lower the codec volume level.
  * @param DeviceAddr: Device address on communication Bus.   
  * @param Volume: a byte value from 0 to 255 (refer to codec registers 
  *         description for more details).
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t cs43l22_SetVolume(uint8_t Volume) {
    uint32_t counter = 0;
    const uint8_t convertedvol = VOLUME_CONVERT(Volume);

    /* Set the Master volume */
    if(Volume > 0xE6) {
        counter += CODEC_IO_Write(DeviceAddr, 0x20, convertedvol - 0xE7); 
        counter += CODEC_IO_Write(DeviceAddr, 0x21, convertedvol - 0xE7);

    /* Set the Master volume */
    } else {
        counter += CODEC_IO_Write(DeviceAddr, 0x20, convertedvol + 0x19); 
        counter += CODEC_IO_Write(DeviceAddr, 0x21, convertedvol + 0x19); 
    }

    return counter;
}

/**
  * @brief Sets new frequency.
  * @param DeviceAddr: Device address on communication Bus.   
  * @param AudioFreq: Audio frequency used to play the audio stream.
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t cs43l22_SetFrequency(uint32_t AudioFreq) {
  return 0;
}

/**
  * @brief Enables or disables the mute feature on the audio codec.
  * @param DeviceAddr: Device address on communication Bus.   
  * @param Cmd: AUDIO_MUTE_ON to enable the mute or AUDIO_MUTE_OFF to disable the
  *             mute mode.
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t cs43l22_SetMute(uint32_t Cmd)
{
    uint32_t counter = 0;
    // Set the Mute mode
    if(Cmd == AUDIO_MUTE_ON) {
        counter += CODEC_IO_Write(DeviceAddr, 0x04, 0xFF);
    // AUDIO_MUTE_OFF Disable the Mute
    } else {
        counter += CODEC_IO_Write(DeviceAddr, 0x04, OutputDev);
    }
    return counter;
}

/**
  * @brief Switch dynamically (while audio file is played) the output target 
  *         (speaker or headphone).
  * @note This function modifies a global variable of the audio codec driver: OutputDev.
  * @param DeviceAddr: Device address on communication Bus.
  * @param Output: specifies the audio output target: OUTPUT_DEVICE_SPEAKER,
  *         OUTPUT_DEVICE_HEADPHONE, OUTPUT_DEVICE_BOTH or OUTPUT_DEVICE_AUTO 
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t cs43l22_SetOutputMode(uint8_t Output) {
    uint32_t counter = 0; 

    switch (Output)  {
        /* SPK always ON & HP always OFF */
        case OUTPUT_DEVICE_SPEAKER:
            counter += CODEC_IO_Write(DeviceAddr, 0x04, 0xFA); 
            OutputDev = 0xFA;
            break;

        /* SPK always OFF & HP always ON */
        case OUTPUT_DEVICE_HEADPHONE:
            counter += CODEC_IO_Write(DeviceAddr, 0x04, 0xAF); 
            OutputDev = 0xAF;
            break;

        /* SPK always ON & HP always ON */
        case OUTPUT_DEVICE_BOTH:
            counter += CODEC_IO_Write(DeviceAddr, 0x04, 0xAA); 
            OutputDev = 0xAA;
            break;

        /* Detect the HP or the SPK automatically */
        case OUTPUT_DEVICE_AUTO:
            counter += CODEC_IO_Write(DeviceAddr, 0x04, 0x05); 
            OutputDev = 0x05;
            break;    

        /* Detect the HP or the SPK automatically */
        default:
            counter += CODEC_IO_Write(DeviceAddr, 0x04, 0x05); 
            OutputDev = 0x05;
            break;
  }  
  return counter;
}



