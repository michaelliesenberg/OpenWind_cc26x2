/*******************************************************************************
*  Filename:       bsp_spi.c
*  Revised:        $Date$
*  Revision:       $Revision$
*
*  Description:    Layer added on top of RTOS driver for backward
*                  compatibility with non RTOS SPI driver.
*
*  Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
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
*    documentation and/or other materials provided with the distribution.
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
*******************************************************************************/


/*******************************************************************************
 * INCLUDES
 */
//#include <ti/sysbios/family/arm/cc26xx/Power.h>
//#include <ti/drivers/Power.h>
//#include <ti/drivers/power/PowerCC26XX.h>
//#include <ti/sysbios/knl/Task.h>




#include "bsp_spi.h"
#include "string.h"
//#include "ExtFlash.h"


/*******************************************************************************
 * GLOBAL variables
 */

/*******************************************************************************
 * LOCAL variables
 */
static SPI_Handle spiHandle = NULL;
static SPI_Params spiParams;

static bool SPI_PINS = false;
static uint8_t nUsers = 0;

/*******************************************************************************
 * @fn          bspSpiWrite
 *
 * @brief       Write to an SPI device
 *
 * @param       buf - pointer to data buffer
 * @param       len - number of bytes to write
 *
 * @return      '0' if success, -1 if failed
 */
int bspSpiWrite(const uint8_t *buf, size_t len)
{
  SPI_Transaction masterTransaction;

  masterTransaction.count  = len;
  masterTransaction.txBuf  = (void*)buf;
  masterTransaction.arg    = NULL;
  masterTransaction.rxBuf  = NULL;

  return SPI_transfer(spiHandle, &masterTransaction) ? 0 : -1;
}


/*******************************************************************************
 * @fn          bspSpiRead
 *
 * @brief       Read from an SPI device
 *
 * @param       buf - pointer to data buffer
 * @param       len - number of bytes to write
 *
 * @return      '0' if success, -1 if failed
 */
int bspSpiRead(uint8_t *buf, size_t len)
{
  SPI_Transaction masterTransaction;

  masterTransaction.count  = len;
  masterTransaction.txBuf  = NULL;
  masterTransaction.arg    = NULL;
  masterTransaction.rxBuf  = buf;

  return SPI_transfer(spiHandle, &masterTransaction) ? 0 : -1;
}


/*******************************************************************************
 * @fn          bspSpiWriteRead
 *
 * @brief       Write to and read from an SPI device in the same transaction
 *
 * @param       buf - pointer to data buffer
 * @param       wlen - number of bytes to write
 * @param       rlen - number of bytes to read
 *
 * @return      '0' if success, -1 if failed
 */
int bspSpiWriteRead(uint8_t *buf, uint8_t wlen, uint8_t rlen)
{
  SPI_Transaction masterTransaction;
  bool success;

  masterTransaction.count  = wlen + rlen;
  masterTransaction.txBuf  = buf;
  masterTransaction.arg    = NULL;
  masterTransaction.rxBuf  = buf;

  success = SPI_transfer(spiHandle, &masterTransaction);
  if (success)
  {
    memcpy(buf,buf+wlen,rlen);
  }

  return success ? 0 : -1;
}

/*******************************************************************************
 * @fn          bspSpiOpen
 *
 * @brief       Open the RTOS SPI driver
 *
 * @param       none
 *
 * @return      none
 */
void bspSpiOpen(void)
{
  //if (SPI_PINS)
  {
      GPIO_resetConfig(CONFIG_GPIO_SPI_0_MOSI);
      GPIO_resetConfig(CONFIG_GPIO_SPI_0_MOSI);
      GPIO_resetConfig(CONFIG_GPIO_SPI_0_SCLK);


    SPI_PINS = false;
  }

  if (spiHandle == NULL)
  {
    SPI_init();
    /*  Configure SPI as master, 1 mHz bit rate*/
    SPI_Params_init(&spiParams);
    spiParams.bitRate = 4000000;//8MHz
    spiParams.mode         = SPI_MASTER;
    spiParams.transferMode = SPI_MODE_BLOCKING;
    //spiParams.frameFormat = SPI_POL0_PHA1;

    /* Attempt to open SPI. */
    spiHandle = SPI_open(CONFIG_SPI_0, &spiParams);

    if (spiHandle == NULL)
    {
      //Task_exit();
    }
  }

  nUsers++;
}

/*******************************************************************************
 * @fn          bspSpiClose
 *
 * @brief       Close the RTOS SPI driver
 *
 * @return      none
 */
void bspSpiClose(void)
{
  nUsers--;

  if (nUsers > 0)
  {
    // Don't close the driver if still in use
    return;
  }

  if (spiHandle != NULL)
  {
    // Close the RTOS driver
    SPI_close(spiHandle);
    spiHandle = NULL;
  }

 // if (!SPI_PINS)
  {
    GPIO_setConfig(CONFIG_GPIO_SPI_0_MOSI, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_SPI_0_MOSI, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_SPI_0_SCLK, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);

    GPIO_write(CONFIG_GPIO_SPI_0_MOSI, 0);
    GPIO_write(CONFIG_GPIO_SPI_0_MOSI, 0);
    GPIO_write(CONFIG_GPIO_SPI_0_SCLK, 0);
    SPI_PINS=true;
  }

  nUsers = 0;
}


