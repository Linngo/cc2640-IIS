/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** ============================================================================
 *  @file       ExtFlash.c
 *
 *  @brief      External flash storage implementation.
 *  ============================================================================
 */

/* -----------------------------------------------------------------------------
*  Includes
* ------------------------------------------------------------------------------
*/
#include "Board.h"
#include "ExtFlash.h"
#include "string.h"
#include <ti/drivers/spi/SPICC26XXDMA.h>
#include <ti/drivers/dma/UDMACC26XX.h>
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/ssi.h)

/* -----------------------------------------------------------------------------
*  Constants and macros
* ------------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
*  Private functions
* ------------------------------------------------------------------------------
*/
 bool Spi_open(uint32_t bitRate);
 void Spi_close(void);
 void Spi_flash(void);
 int Spi_read(uint8_t *buf, size_t length);
 int Spi_write(const uint8_t *buf, size_t length);
 void spiSlaveCallback(SPI_Handle handle,SPI_Transaction * transaction);

/* -----------------------------------------------------------------------------
*  Local variables
* ------------------------------------------------------------------------------
*/
static PIN_Config BoardFlashPinTable[] =
{
    CC2640R2_LAUNCHXL_SPI_WIFI_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN, /* Ext. flash chip select */

    PIN_TERMINATE
};

static PIN_Handle hFlashPin = NULL;
static PIN_State pinState;

// SPI interface
static SPI_Handle spiHandle = NULL;
static SPI_Params spiParams;

// Supported flash devices

/*******************************************************************************
*
*   SPI interface
*
*******************************************************************************/

/*******************************************************************************
* @fn          Spi_write
*
* @brief       Write to an SPI device
*
* @param       buf - pointer to data buffer
* @param       len - number of bytes to write
*
* @return      '0' if success, -1 if failed
*/
int Spi_write(const uint8_t *buf, size_t len)
{
    SPI_Transaction masterTransaction;

    masterTransaction.count  = len;
    masterTransaction.txBuf  = (void*)buf;
    masterTransaction.arg    = NULL;
    masterTransaction.rxBuf  = NULL;

    return SPI_transfer(spiHandle, &masterTransaction) ? 0 : -1;
}


/*******************************************************************************
* @fn          Spi_read
*
* @brief       Read from an SPI device
*
* @param       buf - pointer to data buffer
* @param       len - number of bytes to write
*
* @return      '0' if success, -1 if failed
*/
int Spi_read(uint8_t *buf, size_t len)
{
    SPI_Transaction masterTransaction;

    masterTransaction.count = len;
    masterTransaction.txBuf = NULL;
    masterTransaction.arg = NULL;
    masterTransaction.rxBuf = buf;

    return SPI_transfer(spiHandle, &masterTransaction) ? 0 : -1;
}


/*******************************************************************************
* @fn          Spi_open
*
* @brief       Open the RTOS SPI driver
*
* @param       bitRate - transfer speed in bits/sec
*
* @return      true if success
*/
bool Spi_open(uint32_t bitRate)
{
    /*  Configure SPI as master */
    SPI_Params_init(&spiParams);
    spiParams.bitRate = bitRate;
    spiParams.mode = SPI_MASTER;
    spiParams.transferMode = SPI_MODE_CALLBACK;
    spiParams.transferCallbackFxn = spiSlaveCallback;
    spiParams.transferTimeout = SPI_WAIT_FOREVER;

    /* Attempt to open SPI. */
    spiHandle = SPI_open(Board_SPI0, &spiParams);
 
    return spiHandle != NULL;
}

/*******************************************************************************
* @fn          Spi_close
*
* @brief       Close the RTOS SPI driver
*
* @return      none
*/
void Spi_close(void)
{
    if (spiHandle != NULL)
    {
        // Close the RTOS driver
        SPI_close(spiHandle);
        spiHandle = NULL;
    }
}


/*******************************************************************************
* @fn          Spi_flash
*
* @brief       Get rid of garbage from the slave
*
* @param       none
*
* @return      none
*/
void Spi_flash(void)
{
    /* make sure SPI hardware module is done  */
    while(SSIBusy(((SPICC26XXDMA_HWAttrsV1*)spiHandle->hwAttrs)->baseAddr))
    { };
}
bool wifi_open(void)
{
    uint8_t Buf[4];
    uint8_t send[4] = {'A','T',0x0d,0x0a};
  
    hFlashPin = PIN_open(&pinState, BoardFlashPinTable);

    if (hFlashPin == NULL)
    {
        return false;
    }
    
    Spi_open(1000000);
    PIN_setOutputValue(hFlashPin,CC2640R2_LAUNCHXL_SPI_WIFI_CS,0);
    
    Spi_write(send, 4);
    
    Spi_read(Buf, 4);
    return true;
}

void spiSlaveCallback(SPI_Handle handle,SPI_Transaction * transaction)
{
   
   SPI_close(spiHandle);
}