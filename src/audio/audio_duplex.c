/*
 * Filename: audio_duplex.c
 *
 * Description: Implements bidirectional VoGP audio/voice communication
 * This module connects data from the user application task(buttons),
 * data from the I2S driver (local audio frames), and data from the
 * BLE-Stack Audio Profile (incoming audio frames).
 *
 * Calls to this module run in the application task context. A callback
 * must be plugged for the module to handle higher priority CBs such as
 * Audio data and I2S callbacks
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*********************************************************************
 * INCLUDES
 */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <ti/drivers/PIN.h>
#include <ti/devices/cc26x0r2/driverlib/ioc.h>
#include <ti/mw/extflash/ExtFlash.h>

#include <string.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <driverlib/vims.h>
#include <driverlib/flash.h>
#include <driverlib/prcm.h>
#include <ti/display/Display.h>
//#include <profiles/audio_dle/audio_profile_dle.h>
#include <audiocodec.h>
#include <I2SCC26XX.h>
//#include <ti/drivers/pdm/Codec1.h>

#include "audio_duplex.h"
//#include "icall_ble_api.h"
#include "hal_flash.h"
#include "board_key.h"

#include "hiddev.h"

/*********************************************************************
 * CONSTANTS
 */

#define BLEAUDIO_NUM_NOT_PER_FRAME_ADPCM  1
#define BLEAUDIO_NUM_NOT_PER_FRAME_MSBC   1

/*
 * Required Memory for Bi-directional audio streaming:
 * The I2S driver requires two blocks of memory to be
 * passed in at its open call.
 *  1. Queue memory for TI-RTOS Queue elements
 *  2. Sample memory for sample buffers
 *
 * The amount of memory required while streaming is:
 * (uncompressed buffer)*2 (bidirectional)*sizeof(unit16_t)*I2SCC26XX_QUEUE_SIZE
 *
 * An MSBC frame is larger than ADPCM so in this case we use the worst case
 */
#define I2S_BUF     sizeof(int16_t) * (AUDIO_DUPLEX_MSBC_SAMPLES_PER_FRAME *   \
                                        I2SCC26XX_QUEUE_SIZE * 2)

#define I2S_MEM_BASE (GPRAM_BASE + FlashSectorSizeGet())

/*********************************************************************
 * TYPEDEFS
 */

#define CC2640R2_LAUNCHXL_I2S_ADO               IOID_24//IOID_0
#define CC2640R2_LAUNCHXL_I2S_ADI               IOID_26//IOID_1

/* I2S */
#define CC2640R2_LAUNCHXL_I2S_BCLK              IOID_27//IOID_30
#define CC2640R2_LAUNCHXL_I2S_MCLK              IOID_28//(PIN_Id)IOID_UNUSED
#define CC2640R2_LAUNCHXL_I2S_WCLK              IOID_25//IOID_29

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void AudioDuplex_i2sCallbackFxn(I2SCC26XX_Handle handle,
                                        I2SCC26XX_StreamNotification *notification);
static void AudioDuplex_enableCache();
static void AudioDuplex_disableCache();
//static bStatus_t AudioDuplex_transmitAudioFrame(uint8_t *buf);
static I2SCC26XX_Handle AudioDuplex_openI2S(void);
//static void AudioDuplex_startI2Sstream(void);
//static void AudioDuplex_stopI2Sstream(void);
//static void AudioDuplex_sendStartCmd(void);
//static void AudioDuplex_sendStopCmd(void);
//static void AudioDuplex_shiftEncodedBuffer(uint8_t *encodedBuf, uint8_t len,uint8_t streamType);
void eBuff(uint16_t* dst, uint8_t* src, unsigned srcSize);

/*********************************************************************
 * LOCAL VARIABLES
 */

// Audio Buffer variables
static int16_t *audio_decoded = NULL;
static uint8_t *i2sContMgtBuffer = NULL;
//static size_t written = 0;

// I2S Variables
static I2SCC26XX_Handle i2sHandle = NULL;
static I2SCC26XX_StreamNotification i2sStream;
static bool i2sStreamInProgress = false;
static uint8_t volume = AUDIO_DUPLEX_MAX_VOLUME;
//static uint8_t seqNum = 0;

// BLE Connection handle of stream
uint16_t audioConnectionHandle = NULL;
// Display Interface
//static Display_Handle hDisp = NULL;
// A function pointer to call in the application to set an event
static pfnAudioDuplexCB_t appAudioCB = NULL;

static AudioDuplex_streamVars streamVariables =
{
    AudioDuplex_stream_idle,				//streamState
    AudioDuplex_stream_idle,				//requestedStreamState
    AUDIO_DUPLEX_STREAM_TYPE_NONE,			//streamType
    AUDIO_DUPLEX_STREAM_TYPE_NONE,			//requestedStreamType
    100,						//samplesPerFrame
    0,							//notificationsPerFrame
    7,				//numOfFramesToBuffer
    100,						//rfFrameSize
    0, // si
    0, // pv
    0, // si_rx
    0, // pv_rx
    AUDIO_DUPLEX_MAX_VOLUME, // maxVolume
};

static I2SCC26XX_Params i2sParams =
{
    .requestMode            = I2SCC26XX_CALLBACK_MODE,
    .ui32requestTimeout     = BIOS_WAIT_FOREVER,
    .callbackFxn            = AudioDuplex_i2sCallbackFxn,
    .blockSize              = 100, 
    .pvContBuffer           = NULL,
    .ui32conBufTotalSize    = 0,
    .pvContMgtBuffer        = NULL,
    .ui32conMgtBufTotalSize = 0,
    .currentStream          = &i2sStream
};

I2SCC26XX_Object i2sCC26XXObject;

const I2SCC26XX_HWAttrs i2sCC26XXHWAttrs = {
    .baseAddr = I2S0_BASE,
    .intNum = INT_I2S_IRQ,
    .intPriority = ~0,
    .powerMngrId = PowerCC26XX_PERIPH_I2S,
    .mclkPin = CC2640R2_LAUNCHXL_I2S_MCLK,
    .bclkPin = CC2640R2_LAUNCHXL_I2S_BCLK,
    .wclkPin = CC2640R2_LAUNCHXL_I2S_WCLK,
    .ad0Pin = CC2640R2_LAUNCHXL_I2S_ADO,
    .ad1Pin = CC2640R2_LAUNCHXL_I2S_ADI,
};

/* I2S configuration structure */
const I2SCC26XX_Config I2SCC26XX_config[] = {
    {
        .object = &i2sCC26XXObject,
        .hwAttrs = &i2sCC26XXHWAttrs
    },
    {NULL, NULL}
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      AudioDuplex_open
 *
 * @brief   Called during initialization, opens codec and I2S driver
 *          Initializes hardware and adds Audio Profile
 *
 * @param   None.
 *
 * @return  None.
 */
//int8_t AudioDuplex_open(Display_Handle displayHandle, PIN_Handle pinHandle,
//                            pfnAudioDuplexCB_t inputCB)

int8_t AudioDuplex_open()
{
    uint8_t status = AUDIO_DUPLEX_SUCCESS;
 //   hDisp = displayHandle;

    // Store app callback if not null

    /* Initialize I2S driver */
    i2sHandle = (I2SCC26XX_Handle)&(I2SCC26XX_config);
    I2SCC26XX_init(i2sHandle);

    // Initialize TLV320AIC3254 Codec on Audio BP
    status = AudioCodecOpen();
    if( AUDIO_CODEC_STATUS_SUCCESS != status)
    {
        Display_print0(hDisp, 4, 0, "Fail: Can't open codec");
    }
    // Configure Codec
    status =  AudioCodecConfig(AUDIO_CODEC_TI_3254, AUDIO_CODEC_16_BIT,
                                AUDIO_DUPLEX_SAMPLE_RATE, AUDIO_DUPLEX_NUM_CHAN,
                                AUDIO_DUPLEX_OUTPUT_OPTION,
                                AUDIO_DUPLEX_INPUT_OPTION);
    if( AUDIO_CODEC_STATUS_SUCCESS != status)
    {
        Display_print0(hDisp, 4, 0, "Fail: Can't configure BP");
    }

    // Add the Audio service

    return (status);
}

/*********************************************************************
 * @fn      AudioDuplex_close
 *
 * @brief   Closes hardware, stops streaming
 *
 * @param   None.
 *
 * @return  None.
 */
void AudioDuplex_close(void)
{
    // Close the interface to the hardware codec
    AudioCodecClose();

    // Reset stream vars
    streamVariables.streamState =  AudioDuplex_stream_idle;
    streamVariables.requestedStreamState =  AudioDuplex_stream_idle;
    streamVariables.streamType =  AUDIO_DUPLEX_STREAM_TYPE_NONE;
    streamVariables.requestedStreamType =  AUDIO_DUPLEX_STREAM_TYPE_NONE;
    streamVariables.samplesPerFrame =  0;
    streamVariables.notificationsPerFrame =  0;
    streamVariables.numOfFramesToBuffer =  I2SCC26XX_QUEUE_SIZE;
    streamVariables.rfFrameSize =  100;
    streamVariables.si =  0; // si;
    streamVariables.pv =  0; // pv;
    streamVariables.si_rx =  0; // si_rx;
    streamVariables.pv_rx =  0; // pv_rx;
    streamVariables.maxVolume =  AUDIO_DUPLEX_MAX_VOLUME; // maxVolume;
}

/*********************************************************************
 * @fn      AudioDuplex_setConnectionHandle
 *
 * @brief   Set the connection handle of audio streaming
 *
 * @param   connectionHandle - Connection handle.
 *
 * @return  None
 */
void AudioDuplex_setConnectionHandle(uint16_t connectionHandle)
{
    audioConnectionHandle = connectionHandle;
}

/*********************************************************************
 * @fn      AudioDuplex_getConnectionHandle
 *
 * @brief   Get the connection handle of audio streaming
 *
 * @return  connectionHandle- the handle used to stream
 */
uint16_t AudioDuplex_getConnectionHandle(void)
{
    return (audioConnectionHandle);
}

/*********************************************************************
 * @fn      AudioDuplex_eventHandler
 *
 * @brief   Processes Events called from I2S or BLE-Stack callbacks
 *
 * @param   events - Events to process.
 *
 * @return  None
 */
void AudioDuplex_eventHandler(uint8_t events)
{
    // Read in samples from I2S driver and encode
  
    // Set the state vars to cleanup I2S streaming

    // Send stop command, cleanup stream

    // Startup I2S sampling
	
    // Startup BLE Streaming

    // Handle error events from I2S driver
}

/*********************************************************************
 * @fn      AudioDuplex_processData
 *
 * @brief   Proces incoming audio data
 *
 * @return  None
 */
void AudioDuplex_processData(AudioDuplex_dataType data_type,
                              uint32_t address)
{
    uint32_t lens;     //数据量
    uint8_t  buf[200]; //缓存
    uint32_t ad_offset;//地址偏移
    uint8_t ret=1;
      
    ad_offset = address;
    
    i2sHandle = AudioDuplex_openI2S();    
    i2sStreamInProgress = I2SCC26XX_startStream(i2sHandle,I2S_SAMPLE_RATE_16K);
	
    // Check to see if notification is from audio data or control char
    if (data_type == AudioDuplex_data)
    {
        // If we have received a complete frame OTA decode it and send to I2S
        // for playback
        if (i2sStreamInProgress)
        { 
        ret = ExtFlash_read((address-4), 4, buf);
        lens=(uint32_t)buf[0]&0x000000ff|(buf[1]<<8)&0x0000ff00|(buf[2]<<16)&0x00ff0000|(buf[3]<<24)&0xff000000;
        
	while((int32_t)lens>0){			
            I2SCC26XX_BufferRequest bufferRequest;
            I2SCC26XX_BufferRelease bufferRelease;
            bufferRequest.buffersRequested = I2SCC26XX_BUFFER_OUT;
            // Indicate that we request buffer from output stream           
            bool gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
            if (gotBuffer)
            {			                
                ret = ExtFlash_read(ad_offset, (lens>200)?200:lens, buf);          
                eBuff(bufferRequest.bufferOut,buf,100);
                ad_offset+=200;
                lens-=200;
                // Send the buffer to the BoosterPack
                bufferRelease.bufferHandleOut = bufferRequest.bufferHandleOut;
                bufferRelease.bufferHandleIn = NULL;
                I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);
            }
            if(ret==0)break;
            Task_sleep(4000/Clock_tickPeriod);
        }}
    }	
	I2SCC26XX_stopStream(i2sHandle);
	I2SCC26XX_close(i2sHandle);	
	AudioDuplex_enableCache();
}

void eBuff(uint16_t* dst, uint8_t* src, unsigned srcSize)
{
    // calculate pointers to iterate output buffer
    uint16_t* out = dst;
    uint16_t* end = out+srcSize;

    while(out<end)
    {
        *out = (uint16_t)(*(src+1)<<8)&0xff00|*src&0x00ff;
        out+=1;
        src+=2; 
    }
    
}
/*********************************************************************
 * @fn      AudioDuplex_stopI2Sstream
 *
 * @brief   Stop I2S stream
 *
 * @param   None.
 *
 * @return  None.
 */
static I2SCC26XX_Handle AudioDuplex_openI2S(void)
{
    I2SCC26XX_Handle i2sHandleTmp = NULL;
    // Allocate memory for decoded PCM data
    i2sParams.ui32conMgtBufTotalSize =  I2S_BLOCK_OVERHEAD_IN_BYTES *  \
                                        streamVariables.numOfFramesToBuffer\
                                        * 2;
//12*7*2
    i2sParams.ui32conBufTotalSize    =  sizeof(int16_t) * (streamVariables.samplesPerFrame * \
                                        streamVariables.numOfFramesToBuffer \
                                        * 2);
//2*120*7*2
    // Disable instruction cache to use for audio buffers
    AudioDuplex_disableCache();
    i2sContMgtBuffer = (uint8_t *)(I2S_MEM_BASE + I2S_BUF + 1);
    audio_decoded = (int16_t *)I2S_MEM_BASE;
    
    if (audio_decoded)
    {
        // Setup I2S Params
        i2sParams.blockSize              = streamVariables.samplesPerFrame;
        i2sParams.pvContBuffer           = (void *) audio_decoded;			
        i2sParams.pvContMgtBuffer        = (void *) i2sContMgtBuffer;

        // Reset I2S handle and attempt to open
        i2sHandle = (I2SCC26XX_Handle)&(I2SCC26XX_config);
        i2sHandleTmp = I2SCC26XX_open(i2sHandle, &i2sParams);

        volume = streamVariables.maxVolume;
    }
    else
    {
        Display_print0(hDisp, 5, 0, "Failed to allocate mem for I2S");
        if (i2sContMgtBuffer)
        {
          i2sContMgtBuffer = NULL;
        }
    }

    AudioCodecSpeakerVolCtrl(AUDIO_CODEC_TI_3254, AUDIO_DUPLEX_OUTPUT_OPTION, volume);
    // Volume control
    AudioCodecMicVolCtrl(AUDIO_CODEC_TI_3254, AUDIO_DUPLEX_INPUT_OPTION, volume);

    return (i2sHandleTmp);
}



/*********************************************************************
 * @fn      AudioDuplex_disableCache
 *
 * @brief   Disables the instruction cache and sets power constaints
 *          This prevents the device from sleeping while streaming
 *
 * @param   None.
 *
 * @return  None.
 */
static void AudioDuplex_disableCache()
{
    uint_least16_t hwiKey = Hwi_disable();
    Power_setConstraint(PowerCC26XX_SB_VIMS_CACHE_RETAIN);
    Power_setConstraint(PowerCC26XX_NEED_FLASH_IN_IDLE);
    VIMSModeSafeSet(VIMS_BASE, VIMS_MODE_DISABLED, true);
    Hwi_restore(hwiKey);
}

/*********************************************************************
 * @fn      AudioDuplex_enableCache
 *
 * @brief   Enables the instruction cache and releases power constaints
 *          Allows device to sleep again
 *
 * @param   None.
 *
 * @return  None.
 */
static void AudioDuplex_enableCache()
{
    uint_least16_t hwiKey = Hwi_disable();
    Power_releaseConstraint(PowerCC26XX_SB_VIMS_CACHE_RETAIN);
    Power_releaseConstraint(PowerCC26XX_NEED_FLASH_IN_IDLE);
    VIMSModeSafeSet(VIMS_BASE, VIMS_MODE_ENABLED, true);
    Hwi_restore(hwiKey);
}

/*********************************************************************
 * @fn      AudioDuplex_i2sCallbackFxn
 *
 * @brief   Callback functtion from I2S driver, sets events to be
 *          processed in the task context
 *
 * @param   None.
 *
 * @return  None.
 */
static void AudioDuplex_i2sCallbackFxn(I2SCC26XX_Handle handle,
                                        I2SCC26XX_StreamNotification *notification)
{
    if (notification->status == I2SCC26XX_STREAM_ERROR)
    {
        if(appAudioCB != NULL)
        {
            // Notify the Application of the event
            (*appAudioCB)(AUDIO_DUPLEX_I2S_ERROR_EVENT);
        }
    }
    else if (notification->status == I2SCC26XX_STREAM_BUFFER_READY)
    {
        if(appAudioCB != NULL)
        {
            // Notify the Application of the event
            (*appAudioCB)(AUDIO_DUPLEX_I2S_FRAME_EVENT);
        }
    }
    else if (notification->status == I2SCC26XX_STREAM_BUFFER_READY_BUT_NO_AVAILABLE_BUFFERS)
    {
        if(appAudioCB != NULL)
        {
            // Notify the Application of the event
            (*appAudioCB)(AUDIO_DUPLEX_I2S_FRAME_EVENT);
        }
    }
}
void Playsound(uint32_t sound)
{
  bool ret;
  ret = ExtFlash_open();
  if(ret)
  {
    AudioDuplex_processData(AudioDuplex_data, sound);
   // AudioDuplex_close();
    ExtFlash_close(); 
  }
}
void soundtask(uint8_t playsoundflag)
{
    switch(playsoundflag)
    {
      case 1:playsoundflag=0;
            Playsound(sound_11); //连接成功
            break;
      case 2:playsoundflag=0;
            Playsound(sound_12); //连接失败
            break;
      case 3:playsoundflag=0;
            Playsound(sound_09); //扫描失败
            break;
      case 4:playsoundflag=0;
            Playsound(sound_10); //扫描成功
            break;
      case 5:playsoundflag=0;
            Playsound(sound_06); //关机
            poweroff();
            break;
      case 6:playsoundflag=0;
            Playsound(sound_07); //电量低
            poweroff();
            break;
      default:
          break;       
    }
}
/*********************************************************************
*********************************************************************/
